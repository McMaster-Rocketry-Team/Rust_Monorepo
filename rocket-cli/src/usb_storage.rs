//! Read flight-data records off a VLF5 over USB-C and write them as CSV.
//!
//! The VLF5 firmware logs tagged [`LogRecord`]s (FAST + SLOW) to its SD card.
//! This module speaks the small vendor protocol in
//! [`firmware_common_new::flight_storage`]: a vendor control transfer carries a
//! [`CliRequest`] in `wValue`, and the device replies on the bulk-IN endpoint
//! with a header followed (for downloads) by the raw SD data blocks.

use anyhow::Context as _;
use anyhow::{Result, anyhow, bail};
use rusb::{Context, DeviceHandle, Direction, Recipient, RequestType, UsbContext};
use std::time::{Duration, Instant};

use firmware_common_new::flight_data_record::{
    FlightDataRecord, NodeStatusRecord, PYRO_DROGUE_CONTINUITY, PYRO_DROGUE_FIRE,
    PYRO_MAIN_CONTINUITY,
    PYRO_MAIN_FIRE, PYRO_SHORT_CIRCUIT, AIRBRAKES_PAD_CALIBRATED,
    AirbrakesState,
    AIRBRAKES_BURNOUT, AIRBRAKES_SUBSONIC_DRAG,
    DEPLOYMENT_BARO_RESYNC, DEPLOYMENT_BARO_GATE_REJECT,
    merge_log_records,
};
use firmware_common_new::can_bus::messages::amp_status::PowerOutputStatus;
use firmware_common_new::flight_storage::{
    BLOCK_SIZE, HEADER_LEN, RESPONSE_MAGIC, STORAGE_VERSION, decode_response_header,
    parse_log_records,
};
use firmware_common_new::vlp::usb::CliRequest;
use packed_struct::PrimitiveEnum as _;

/// USB vendor/product IDs for the WinUSB flight-log interface.
const VLF5_USB_VID: u16 = 0xc0de;
const VLF5_USB_PID: u16 = 0xcafe;
/// Bulk-IN endpoint address (EP 1 IN).
const EP_IN: u8 = 0x81;
/// Vendor interface number.
const INTERFACE: u8 = 0;

/// Find the VLF5 flight-log USB interface and claim it.
fn find_and_open() -> Result<DeviceHandle<Context>> {
    let ctx = Context::new().context("creating libusb context")?;
    for device in ctx.devices().context("listing USB devices")?.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == VLF5_USB_VID && desc.product_id() == VLF5_USB_PID {
            let handle = device.open().context(
                "opening the VLF5 (on Linux you may need a udev rule or to run with sudo)",
            )?;
            #[cfg(target_os = "linux")]
            let _ = handle.set_auto_detach_kernel_driver(true);
            handle
                .claim_interface(INTERFACE)
                .context("claiming the VLF5 interface")?;
            return Ok(handle);
        }
    }
    bail!("VLF5 not found over USB (VID={VLF5_USB_VID:#06x} PID={VLF5_USB_PID:#06x}). Is it plugged in via USB-C and powered on?")
}

/// Send a [`CliRequest`] as a vendor control transfer (the command rides in
/// `wValue`; `bRequest` is unused).
fn send_request(handle: &DeviceHandle<Context>, request: CliRequest) -> Result<()> {
    handle.write_control(
        rusb::request_type(Direction::Out, RequestType::Vendor, Recipient::Interface),
        0,
        request as u16,
        INTERFACE as u16,
        &[],
        Duration::from_secs(2),
    )?;
    Ok(())
}

/// First offset of `needle` within `haystack`, if any.
fn find_subsequence(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// Discard any bulk-IN bytes still queued from a previously interrupted transfer so
/// a fresh command reads its own response, not stale block data.
///
/// A `download` the host stopped reading early (its own timeout, a Ctrl-C, an error)
/// leaves unread bytes in the device endpoint / kernel URB buffer. Without flushing
/// them the next command reads that stale data as its header and fails with "device
/// sent an invalid response header" — and stays broken for every later command until
/// the VLF5 is power-cycled. Reading the endpoint to idle here resyncs the pipe
/// without a device reset. Bounded so a truly-idle endpoint returns promptly.
fn drain_stale(handle: &DeviceHandle<Context>) {
    let mut buf = vec![0u8; BLOCK_SIZE];
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut idle = 0u8;
    while idle < 2 && Instant::now() < deadline {
        match handle.read_bulk(EP_IN, &mut buf, Duration::from_millis(100)) {
            Ok(0) | Err(rusb::Error::Timeout) => idle += 1,
            Ok(_) => idle = 0,
            Err(_) => break,
        }
    }
}

/// Read a full framed response: a [`HEADER_LEN`]-byte header, then (for a
/// download) `block_count` raw 512-byte data blocks.
fn read_response(handle: &DeviceHandle<Context>) -> Result<Vec<u8>> {
    let mut data: Vec<u8> = Vec::new();
    let mut buf = vec![0u8; BLOCK_SIZE];
    let mut expected: Option<usize> = None;
    let overall_deadline = Instant::now() + Duration::from_secs(300);
    let mut idle_since: Option<Instant> = None;

    loop {
        match handle.read_bulk(EP_IN, &mut buf, Duration::from_millis(500)) {
            Ok(n) if n > 0 => {
                data.extend_from_slice(&buf[..n]);
                idle_since = None;
            }
            Ok(_) | Err(rusb::Error::Timeout) => {
                if expected.is_some_and(|e| data.len() >= e) {
                    break;
                }
                let since = *idle_since.get_or_insert_with(Instant::now);
                if since.elapsed() > Duration::from_secs(15) {
                    bail!(
                        "device stopped sending (got {} of {} expected bytes)",
                        data.len(),
                        expected.map_or("?".to_string(), |e| e.to_string())
                    );
                }
            }
            Err(e) => return Err(e).context("reading from the VLF5 bulk endpoint"),
        }

        // Lock onto the response header, resyncing past any stale leading bytes: a
        // previously interrupted transfer can leave block data queued ahead of this
        // response, and treating that as the header is what used to wedge the protocol
        // until a power cycle. Skip everything before the response magic instead.
        if expected.is_none() {
            if let Some(off) = find_subsequence(&data, &RESPONSE_MAGIC) {
                if off > 0 {
                    data.drain(..off);
                }
                if data.len() >= HEADER_LEN {
                    let (_record_count, storage_version, block_count) =
                        decode_response_header(&data[..HEADER_LEN])
                            .ok_or_else(|| anyhow!("device sent an invalid response header"))?;
                    if storage_version != STORAGE_VERSION {
                        bail!(
                            "unsupported storage version {storage_version} (this rocket-cli \
                             reads v{STORAGE_VERSION})"
                        );
                    }
                    expected = Some(HEADER_LEN + block_count as usize * BLOCK_SIZE);
                }
            } else if data.len() >= RESPONSE_MAGIC.len() {
                // No magic yet: keep only a possible partial-magic tail so a long run
                // of stale bytes cannot grow `data` without bound.
                let drop = data.len() - (RESPONSE_MAGIC.len() - 1);
                data.drain(..drop);
            }
        }

        if expected.is_some_and(|e| data.len() >= e) {
            break;
        }
        if Instant::now() > overall_deadline {
            bail!("download exceeded 300s, aborting");
        }
    }

    Ok(data)
}

/// Read just the response header. Used by `List`/`Clear`, which reply with
/// metadata only (no data blocks follow).
fn read_header(handle: &DeviceHandle<Context>) -> Result<[u8; HEADER_LEN]> {
    let mut data: Vec<u8> = Vec::new();
    let mut buf = vec![0u8; 64];
    // Generous deadline: after an interrupted download the device may still be
    // finishing that transfer before it answers this command.
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        // Resync past any stale leading bytes, then lock onto the response magic.
        if let Some(off) = find_subsequence(&data, &RESPONSE_MAGIC) {
            if off > 0 {
                data.drain(..off);
            }
            if data.len() >= HEADER_LEN {
                let mut header = [0u8; HEADER_LEN];
                header.copy_from_slice(&data[..HEADER_LEN]);
                return Ok(header);
            }
        } else if data.len() >= RESPONSE_MAGIC.len() {
            let drop = data.len() - (RESPONSE_MAGIC.len() - 1);
            data.drain(..drop);
        }

        if Instant::now() > deadline {
            bail!("timed out waiting for a response from the VLF5");
        }

        match handle.read_bulk(EP_IN, &mut buf, Duration::from_secs(2)) {
            Ok(n) => data.extend_from_slice(&buf[..n]),
            Err(rusb::Error::Timeout) => {}
            Err(e) => return Err(e).context("reading from the VLF5 bulk endpoint"),
        }
    }
}

/// Split the raw block stream into merged CSV rows.
fn parse_records(data: &[u8]) -> Result<(u32, Vec<FlightDataRecord>)> {
    let (log_record_count, storage_version, block_count) = decode_response_header(data)
        .ok_or_else(|| anyhow!("device sent an invalid response header"))?;
    if storage_version != STORAGE_VERSION {
        bail!(
            "unsupported storage version {storage_version} (this rocket-cli reads \
             v{STORAGE_VERSION})"
        );
    }
    let blocks = &data[HEADER_LEN..];
    let expected_bytes = block_count as usize * BLOCK_SIZE;
    if blocks.len() < expected_bytes {
        bail!(
            "response truncated: {} of {} block bytes arrived",
            blocks.len(),
            expected_bytes
        );
    }

    // A CRC failure no longer aborts the export. One bad block out of thousands
    // must not make a whole flight unrecoverable, so the parser keeps going and
    // tags every record it read out of a bad block; those rows carry
    // `source_block_crc_failed` in the CSV. The warning stays, because the
    // per-row column is only useful to someone who knows to look for it.
    let parsed = parse_log_records(log_record_count, blocks, block_count)
        .ok_or_else(|| anyhow!("failed to decode the log stream — data may be corrupt"))?;
    if parsed.crc_failed_blocks > 0 {
        eprintln!(
            "warning: {} block(s) failed their CRC check — their rows are exported with \
             source_block_crc_failed=true and every value on them may be wrong",
            parsed.crc_failed_blocks
        );
    }
    if parsed.invalid_records > 0 {
        eprintln!(
            "warning: {} record(s) in those blocks held values their own types cannot \
             represent and were skipped entirely",
            parsed.invalid_records
        );
    }
    let merged = merge_log_records(&parsed.records);

    Ok((log_record_count, merged))
}

/// One optional column. Absence writes an empty cell, which is the only
/// rendering a reader cannot mistake for a measurement — a 0, a NaN or a 65535
/// all sit inside the range of values the column legitimately holds. Every
/// column fed by an `Option` goes through here, so nothing downstream has to
/// know which of them can be absent.
fn cell<T: ToString>(value: Option<T>) -> String {
    value.map_or_else(String::new, |v| v.to_string())
}

/// One bit of a status bitmask. Blank when the mask itself is absent — no
/// estimator sample backed this row, or the pyro watch had nothing to report
/// yet. `false` there would be a statement about a sample that never happened,
/// indistinguishable from a gate that looked at a real sample and passed it.
fn bit(mask: Option<u8>, flag: u8) -> String {
    cell(mask.map(|mask| (mask & flag) != 0))
}

/// Decode one AMP output's 2-bit `PowerOutputStatus` from the packed
/// `out_status` byte (out1 in the LSBs). `None` is an AMP that has never sent
/// an `AmpStatusMessage`, which blanks all three output columns rather than
/// reporting them as `Disabled`.
///
/// Every discriminant is printed by name, never folded into `Disabled`. That
/// includes `Unknown` — "the AMP never told us what this output is doing" — and
/// the `Invalid` fallback for a code the enum does not define, both of which
/// mean something entirely different from an output that was commanded off.
fn amp_out(status: Option<u8>, out_index: u8) -> String {
    let Some(status) = status else {
        return String::new();
    };
    match PowerOutputStatus::from_primitive((status >> (out_index * 2)) & 0b11) {
        Some(s) => format!("{:?}", s),
        None => "Invalid".to_string(),
    }
}

/// The five columns describing one CAN node's last heartbeat.
///
/// `None` is a node never heard from at all, and blanks all five. That is a
/// different fact from a node whose last heartbeat said `online: false`, which
/// fills the columns in with what it did report before going quiet — and the
/// blank row is what keeps "never on the bus" from reading as "booted, then
/// dropped".
fn node_cells(node: Option<&NodeStatusRecord>) -> [String; 5] {
    match node {
        Some(node) => [
            node.online.to_string(),
            node.uptime_s.to_string(),
            format!("{:?}", node.health),
            format!("{:?}", node.mode),
            format!("0b{:011b}", node.custom_status),
        ],
        None => ["", "", "", "", ""].map(String::from),
    }
}

fn write_csv(path: &str, records: &[FlightDataRecord]) -> Result<()> {
    let mut w = csv::Writer::from_path(path).with_context(|| format!("creating {}", path))?;
    // The pyro columns come from the fast record, so they update at the full
    // fast rate (±2.3 ms). The airbrakes actuation columns come from the slow
    // snapshot — the control loop only produces one every 100 ms — so they
    // repeat across the ~42 fast rows that share a snapshot.
    //
    // Every column below that can be absent is written by `cell` and is empty
    // when it is, which is why there are no longer any `*_valid` columns: a
    // reader learns the same thing from the data column itself, and cannot end
    // up pairing a validity bit with the wrong row.
    //
    // `source_block_crc_failed` sits second because it qualifies every other
    // cell on the row: the record came out of a 512-byte block whose CRC32 did
    // not match, so any value here may be silently wrong. Such rows are still
    // exported — a bad block must not cost a whole flight — but they are not
    // evidence of anything on their own.
    //
    // `slow_timestamp_us` is the clock of the snapshot the slow columns were
    // copied from. `timestamp_us - slow_timestamp_us` bounds how old the
    // snapshot is (≤ ~100 ms). It does NOT bound the age of the readings inside
    // it: `air_brakes_servo_temp` is read off the servo at 10 Hz, so it can be
    // a further ~100 ms older still. The extension columns are not on the slow
    // snapshot at all — they come off the fast record, and `slow_timestamp_us`
    // says nothing about them.
    w.write_record([
        "record_count",
        "source_block_crc_failed",
        "timestamp_us",
        "slow_timestamp_us",
        "unix_time_us",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "pressure",
        "mag_x",
        "mag_y",
        "mag_z",
        "deployment_kf_altitude_asl",
        "deployment_kf_vertical_velocity",
        "deployment_baro_gate_reject",
        "deployment_baro_resync",
        "airbrakes_kf_altitude_asl",
        "airbrakes_kf_vertical_velocity",
        "airbrakes_kf_tilt_deg",
        "airbrakes_pad_calibrated",
        "airbrakes_subsonic_drag",
        "airbrakes_burnout",
        "airbrakes_state",
        "temperature",
        "battery_voltage",
        "lat",
        "lon",
        "gps_altitude_asl",
        "num_sats",
        "hdop",
        "vdop",
        "pdop",
        "launch_pad_altitude_asl",
        "flight_stage",
        "pyro_main_continuity",
        "pyro_main_fire",
        "pyro_drogue_continuity",
        "pyro_drogue_fire",
        "pyro_short_circuit",
        "air_brakes_commanded_extension",
        "air_brakes_actual_extension",
        "air_brakes_servo_temp",
        "air_brakes_validation_deploy",
        "mpc_predicted_apogee_asl",
        "air_brakes_target_apogee_asl",
        "amp_online",
        "amp_uptime_s",
        "amp_health",
        "amp_mode",
        "amp_custom_status",
        "icarus_online",
        "icarus_uptime_s",
        "icarus_health",
        "icarus_mode",
        "icarus_custom_status",
        "ozys_online",
        "ozys_uptime_s",
        "ozys_health",
        "ozys_mode",
        "ozys_custom_status",
        "payload_sdrm_online",
        "payload_sdrm_uptime_s",
        "payload_sdrm_health",
        "payload_sdrm_mode",
        "payload_sdrm_custom_status",
        "amp_out1_status",
        "amp_out2_status",
        "amp_out3_status",
        "amp_shared_battery_v",
        "payload_epm_batt_mv",
        "payload_sys_3v3_ma",
        "payload_sys_5v_ma",
        "payload_per_3v3_ma",
        "payload_per_5v_ma",
        "payload_per_9v_ma",
        "payload_per_12v_ma",
        "payload_actuator_1_steps",
        "payload_actuator_2_steps",
        "payload_actuator_3_steps",
    ])?;

    for r in records {
        let imu = r.imu.as_ref();
        let deployment = r.deployment.as_ref();
        let airbrakes = r.airbrakes.as_ref();
        // Two groups, two rates: the command and Icarus's report come off the
        // fast record and are on every row; the MPC's numbers and the servo
        // temperature come off the slow snapshot and repeat across ~42 rows.
        let air_brakes = &r.air_brakes;
        let air_brakes_mpc = r.air_brakes_mpc.as_ref();
        let amp = r.amp.as_ref();
        let payload = r.payload.as_ref();
        let pyro = r.pyro_flags;

        let mut row = vec![
            r.record_count.to_string(),
            r.source_block_crc_failed.to_string(),
            r.timestamp_us.to_string(),
            cell(r.slow_timestamp_us),
            cell(r.unix_time_us),
            cell(imu.map(|imu| imu.acc[0])),
            cell(imu.map(|imu| imu.acc[1])),
            cell(imu.map(|imu| imu.acc[2])),
            cell(imu.map(|imu| imu.gyro[0])),
            cell(imu.map(|imu| imu.gyro[1])),
            cell(imu.map(|imu| imu.gyro[2])),
            r.pressure.to_string(),
            cell(r.mag.map(|mag| mag[0])),
            cell(r.mag.map(|mag| mag[1])),
            cell(r.mag.map(|mag| mag[2])),
            cell(deployment.and_then(|d| d.kf_altitude_asl)),
            cell(deployment.and_then(|d| d.kf_vertical_velocity)),
            bit(deployment.map(|d| d.flags), DEPLOYMENT_BARO_GATE_REJECT),
            bit(deployment.map(|d| d.flags), DEPLOYMENT_BARO_RESYNC),
            cell(airbrakes.and_then(|a| a.kf_altitude_asl)),
            cell(airbrakes.and_then(|a| a.kf_vertical_velocity)),
            cell(airbrakes.and_then(|a| a.kf_tilt_deg)),
            bit(airbrakes.map(|a| a.flags), AIRBRAKES_PAD_CALIBRATED),
            bit(airbrakes.map(|a| a.flags), AIRBRAKES_SUBSONIC_DRAG),
            bit(airbrakes.map(|a| a.flags), AIRBRAKES_BURNOUT),
            // The name, like `flight_stage`, rather than the discriminant:
            // a CSV is read by people first.
            airbrakes
                .map(|a| format!("{:?}", AirbrakesState::from_flags(a.flags)))
                .unwrap_or_default(),
            cell(r.temperature),
            cell(r.battery_voltage),
            cell(r.lat_lon.map(|(lat, _)| lat)),
            cell(r.lat_lon.map(|(_, lon)| lon)),
            cell(r.gps_altitude_asl),
            cell(r.num_of_fix_satellites),
            cell(r.hdop),
            cell(r.vdop),
            cell(r.pdop),
            cell(r.launch_pad_altitude_asl),
            format!("{:?}", r.flight_stage),
            bit(pyro, PYRO_MAIN_CONTINUITY),
            bit(pyro, PYRO_MAIN_FIRE),
            bit(pyro, PYRO_DROGUE_CONTINUITY),
            bit(pyro, PYRO_DROGUE_FIRE),
            bit(pyro, PYRO_SHORT_CIRCUIT),
            cell(air_brakes.commanded_extension),
            cell(air_brakes.actual_extension),
            cell(air_brakes_mpc.and_then(|a| a.servo_temp)),
            cell(Some(air_brakes.validation_deploy as u8)),
            cell(air_brakes_mpc.and_then(|a| a.predicted_apogee_asl)),
            cell(air_brakes_mpc.and_then(|a| a.target_apogee_asl)),
        ];
        row.extend(node_cells(r.amp_node.as_ref()));
        row.extend(node_cells(r.icarus_node.as_ref()));
        row.extend(node_cells(r.ozys_node.as_ref()));
        row.extend(node_cells(r.payload_sdrm_node.as_ref()));
        row.extend([
            amp_out(amp.map(|a| a.out_status), 0),
            amp_out(amp.map(|a| a.out_status), 1),
            amp_out(amp.map(|a| a.out_status), 2),
            cell(amp.map(|a| a.shared_battery_v)),
            cell(payload.and_then(|p| p.epm_batt_mv)),
            cell(payload.and_then(|p| p.rail_ma[0])),
            cell(payload.and_then(|p| p.rail_ma[1])),
            cell(payload.and_then(|p| p.rail_ma[2])),
            cell(payload.and_then(|p| p.rail_ma[3])),
            cell(payload.and_then(|p| p.rail_ma[4])),
            cell(payload.and_then(|p| p.rail_ma[5])),
            cell(payload.and_then(|p| p.actuator_steps[0])),
            cell(payload.and_then(|p| p.actuator_steps[1])),
            cell(payload.and_then(|p| p.actuator_steps[2])),
        ]);
        w.write_record(&row)?;
    }

    w.flush()?;
    Ok(())
}

/// `list-files`: print a summary of what's stored on the VLF5.
pub fn list_files() -> Result<()> {
    let handle = find_and_open()?;
    drain_stale(&handle);
    send_request(&handle, CliRequest::List)?;
    let header = read_header(&handle)?;
    let (record_count, storage_version, block_count) = decode_response_header(&header)
        .ok_or_else(|| anyhow!("device sent an invalid response header"))?;

    println!("VLF5 flight log:");
    println!("  records      : {}", record_count);
    println!(
        "  data blocks  : {} ({} bytes on card)",
        block_count,
        block_count as usize * BLOCK_SIZE
    );
    if storage_version == STORAGE_VERSION {
        println!("  storage ver  : {}", storage_version);
    } else {
        println!(
            "  storage ver  : {} (this rocket-cli reads v{}; download unsupported)",
            storage_version, STORAGE_VERSION
        );
    }
    if record_count == 0 {
        println!("  (empty — nothing has been logged yet)");
    }
    Ok(())
}

/// `download-flight-log <out.csv>`: pull the whole log and write it as CSV.
pub fn download_file(output: &str) -> Result<()> {
    let handle = find_and_open()?;
    drain_stale(&handle);
    send_request(&handle, CliRequest::Download)?;
    let data = read_response(&handle)?;
    let (log_record_count, records) = parse_records(&data)?;
    write_csv(output, &records)?;
    println!(
        "Wrote {} fast row(s) from {} on-card record(s) to {}",
        records.len(),
        log_record_count,
        output
    );
    Ok(())
}

/// `clear-storage`: erase the log on the VLF5.
pub fn clear_storage() -> Result<()> {
    let handle = find_and_open()?;
    drain_stale(&handle);
    send_request(&handle, CliRequest::Clear)?;
    let _ack = read_header(&handle)?;
    println!("VLF5 storage cleared.");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use firmware_common_new::can_bus::messages::vl_status::FlightStage;
    use firmware_common_new::flight_data_record::{
        AirBrakesActuationRecord, DeploymentEstimatorRecord, FlightDataFastRecord, LogRecord,
        ParsedLogRecord, merge_log_records,
    };

    /// A CSV whose rows are narrower or wider than its header silently
    /// mis-labels every column to the right of the mistake, and nothing else in
    /// the pipeline would catch it — the writer is happy to emit ragged rows.
    /// This also pins the thing the `Option` refactor exists to guarantee: a
    /// Mach-lockout sample, where the deployment filter is frozen, must reach
    /// the spreadsheet as an EMPTY cell and never as `0`.
    #[test]
    fn csv_rows_match_the_header_and_absence_is_an_empty_cell() {
        let lockout = FlightDataFastRecord {
            sequence: 0,
            timestamp_us: 1000,
            unix_time_us: None,
            imu: None,
            pressure: 101325.0,
            mag: None,
            // The filter is frozen: the sample exists, the numbers do not.
            deployment: Some(DeploymentEstimatorRecord {
                kf_altitude_asl: None,
                kf_vertical_velocity: None,
                flags: 0,
            }),
            airbrakes: None,
            flight_stage: FlightStage::Ascent,
            pyro_flags: None,
            // Nothing commanded and nothing heard from Icarus, which is what
            // the pad looks like — and must reach the spreadsheet as two empty
            // cells, not as a pair of stowed zeroes.
            air_brakes: AirBrakesActuationRecord {
                commanded_extension: None,
                actual_extension: None,
                validation_deploy: false,
            },
        };
        // No slow record ahead of it, so every slow column is absent too. The
        // second row is the same sample read out of a block that failed its
        // CRC, which is exported all the same — but marked.
        let records = merge_log_records(&[
            ParsedLogRecord::good(LogRecord::Fast(lockout.clone())),
            ParsedLogRecord {
                record: LogRecord::Fast(FlightDataFastRecord {
                    sequence: 1,
                    ..lockout
                }),
                block_crc_ok: false,
            },
        ]);
        assert_eq!(records.len(), 2);

        let path = std::env::temp_dir().join("rocket_cli_csv_width_test.csv");
        let path = path.to_str().unwrap();
        write_csv(path, &records).expect("write_csv");
        let text = std::fs::read_to_string(path).expect("read back");
        std::fs::remove_file(path).ok();

        let mut lines = text.lines();
        let header: Vec<&str> = lines.next().expect("header").split(',').collect();
        let row: Vec<&str> = lines.next().expect("row").split(',').collect();
        let bad_row: Vec<&str> = lines.next().expect("second row").split(',').collect();
        assert_eq!(
            header.len(),
            row.len(),
            "header has {} columns but the row has {}",
            header.len(),
            row.len()
        );
        assert_eq!(header.len(), bad_row.len());

        let col_index = |name: &str| {
            header
                .iter()
                .position(|h| *h == name)
                .unwrap_or_else(|| panic!("no {} column", name))
        };
        let col = |name: &str| row[col_index(name)];
        // The whole point: frozen filter reads empty, not zero.
        assert_eq!(col("deployment_kf_altitude_asl"), "");
        assert_eq!(col("deployment_kf_vertical_velocity"), "");
        assert_eq!(col("acc_x"), "");
        assert_eq!(col("unix_time_us"), "");
        // A value that is genuinely present still prints.
        assert_eq!(col("pressure"), "101325");
        // The deleted `valid` bitmask must not have left a column behind.
        assert!(!header.contains(&"imu_valid"));

        // No slow snapshot precedes either row, so the snapshot clock is empty
        // rather than 0 — a 0 here would read as "sampled at T=0".
        assert_eq!(col("slow_timestamp_us"), "");
        // A row from a good block says so, and a row from a CRC-failed block
        // says so too. Both are present: the export does not drop the bad one.
        assert_eq!(col("source_block_crc_failed"), "false");
        assert_eq!(bad_row[col_index("source_block_crc_failed")], "true");
        assert_eq!(bad_row[col_index("pressure")], "101325");
    }
}
