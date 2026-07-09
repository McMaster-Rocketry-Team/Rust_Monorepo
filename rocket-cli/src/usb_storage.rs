//! Read flight-data records off a VLF5 over USB-C and write them as CSV.
//!
//! The VLF5 firmware logs tagged [`LogRecord`]s (IMU + SLOW) to its SD card.
//! This module speaks the small vendor protocol in
//! [`firmware_common_new::flight_storage`]: a vendor control transfer carries a
//! [`CliRequest`] in `wValue`, and the device replies on the bulk-IN endpoint
//! with a header followed (for downloads) by the raw SD data blocks.

use anyhow::Context as _;
use anyhow::{Result, anyhow, bail};
use rusb::{Context, DeviceHandle, Direction, Recipient, RequestType, UsbContext};
use std::time::{Duration, Instant};

use firmware_common_new::flight_data_record::{
    FlightDataRecord, PYRO_DROGUE_CONTINUITY, PYRO_DROGUE_FIRE, PYRO_MAIN_CONTINUITY,
    PYRO_MAIN_FIRE, PYRO_SHORT_CIRCUIT, VALID_AIRBRAKES_ACTUAL, VALID_AIRBRAKES_COMMANDED,
    VALID_BARO, VALID_BATTERY, VALID_GPS_ALT, VALID_GPS_FIX, VALID_IMU, VALID_MAG,
    merge_log_records,
};
use firmware_common_new::flight_storage::{
    BLOCK_SIZE, HEADER_LEN, RECORD_LEN_TAGGED, RECORD_LEN_V1, RECORDS_PER_BLOCK_V1,
    STORAGE_VERSION, STORAGE_VERSION_V2, decode_response_header, parse_log_records_tagged,
    parse_records_v1, verify_data_block,
};
use firmware_common_new::vlp::usb::CliRequest;

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
            Ok(n) => {
                data.extend_from_slice(&buf[..n]);
                idle_since = None;
            }
            Err(rusb::Error::Timeout) => {
                if expected.is_some_and(|e| data.len() >= e) {
                    break;
                }
                let since = *idle_since.get_or_insert_with(Instant::now);
                if since.elapsed() > Duration::from_secs(10) {
                    bail!(
                        "device stopped sending (got {} of {} expected bytes)",
                        data.len(),
                        expected.map_or("?".to_string(), |e| e.to_string())
                    );
                }
            }
            Err(e) => return Err(e).context("reading from the VLF5 bulk endpoint"),
        }

        if expected.is_none() && data.len() >= HEADER_LEN {
            let (_record_count, record_len, block_count) =
                decode_response_header(&data[..HEADER_LEN])
                    .ok_or_else(|| anyhow!("device sent an invalid response header"))?;
            if record_len != RECORD_LEN_TAGGED && record_len as usize != RECORD_LEN_V1 {
                bail!(
                    "unsupported record layout: device reports record_len={record_len}. \
                     Rebuild rocket-cli from the same source as the firmware."
                );
            }
            expected = Some(HEADER_LEN + block_count as usize * BLOCK_SIZE);
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
    let deadline = Instant::now() + Duration::from_secs(5);
    while data.len() < HEADER_LEN {
        match handle.read_bulk(EP_IN, &mut buf, Duration::from_secs(2)) {
            Ok(n) => data.extend_from_slice(&buf[..n]),
            Err(rusb::Error::Timeout) => {
                bail!("timed out waiting for a response from the VLF5")
            }
            Err(e) => return Err(e).context("reading from the VLF5 bulk endpoint"),
        }
        if Instant::now() > deadline {
            bail!("timed out waiting for a response from the VLF5");
        }
    }
    let mut header = [0u8; HEADER_LEN];
    header.copy_from_slice(&data[..HEADER_LEN]);
    Ok(header)
}

/// Split the raw block stream into merged CSV rows.
fn parse_records(data: &[u8]) -> Result<(u32, Vec<FlightDataRecord>)> {
    let (log_record_count, record_len, block_count) = decode_response_header(data)
        .ok_or_else(|| anyhow!("device sent an invalid response header"))?;
    let blocks = &data[HEADER_LEN..];

    let mut crc_errors = 0u32;
    for i in 0..block_count as usize {
        let start = i * BLOCK_SIZE;
        let block: &[u8; BLOCK_SIZE] = blocks
            .get(start..start + BLOCK_SIZE)
            .ok_or_else(|| anyhow!("response truncated at block {}", i))?
            .try_into()
            .unwrap();
        if !verify_data_block(block) {
            crc_errors += 1;
        }
    }
    if crc_errors > 0 {
        eprintln!(
            "warning: {} block(s) failed their CRC check — data may be corrupt",
            crc_errors
        );
    }

    let merged = if record_len == RECORD_LEN_TAGGED {
        let log = parse_tagged_log(log_record_count, blocks, block_count)
            .ok_or_else(|| anyhow!("failed to decode tagged log stream"))?;
        merge_log_records(&log)
    } else {
        parse_records_v1(blocks, log_record_count, block_count)
            .ok_or_else(|| anyhow!("failed to decode v1 log stream"))?
    };

    Ok((log_record_count, merged))
}

fn parse_tagged_log(
    record_count: u32,
    blocks: &[u8],
    block_count: u32,
) -> Option<Vec<firmware_common_new::flight_data_record::LogRecord>> {
    for version in [STORAGE_VERSION, STORAGE_VERSION_V2] {
        if let Some(log) = parse_log_records_tagged(record_count, blocks, block_count, version) {
            if log.len() == record_count as usize {
                return Some(log);
            }
        }
    }
    None
}

fn bit(mask: u8, flag: u8) -> String {
    ((mask & flag) != 0).to_string()
}

fn write_csv(path: &str, records: &[FlightDataRecord]) -> Result<()> {
    let mut w = csv::Writer::from_path(path).with_context(|| format!("creating {}", path))?;
    w.write_record([
        "record_count",
        "timestamp_us",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "temperature",
        "pressure",
        "mag_x",
        "mag_y",
        "mag_z",
        "battery_voltage",
        "lat",
        "lon",
        "altitude",
        "num_sats",
        "hdop",
        "vdop",
        "pdop",
        "flight_stage",
        "imu_valid",
        "baro_valid",
        "mag_valid",
        "gps_fix",
        "gps_alt_valid",
        "battery_valid",
        "pyro_main_continuity",
        "pyro_main_fire",
        "pyro_drogue_continuity",
        "pyro_drogue_fire",
        "pyro_short_circuit",
        "air_brakes_commanded_extension",
        "air_brakes_actual_extension",
        "air_brakes_commanded_valid",
        "air_brakes_actual_valid",
    ])?;

    for r in records {
        let v = r.valid;
        let p = r.pyro_flags;
        w.write_record([
            r.record_count.to_string(),
            r.timestamp_us.to_string(),
            r.acc[0].to_string(),
            r.acc[1].to_string(),
            r.acc[2].to_string(),
            r.gyro[0].to_string(),
            r.gyro[1].to_string(),
            r.gyro[2].to_string(),
            r.temperature.to_string(),
            r.pressure.to_string(),
            r.mag[0].to_string(),
            r.mag[1].to_string(),
            r.mag[2].to_string(),
            r.battery_voltage.to_string(),
            r.lat_lon.0.to_string(),
            r.lat_lon.1.to_string(),
            r.altitude.to_string(),
            r.num_of_fixed_satalites.to_string(),
            r.hdop.to_string(),
            r.vdop.to_string(),
            r.pdop.to_string(),
            format!("{:?}", r.flight_stage),
            bit(v, VALID_IMU),
            bit(v, VALID_BARO),
            bit(v, VALID_MAG),
            bit(v, VALID_GPS_FIX),
            bit(v, VALID_GPS_ALT),
            bit(v, VALID_BATTERY),
            bit(p, PYRO_MAIN_CONTINUITY),
            bit(p, PYRO_MAIN_FIRE),
            bit(p, PYRO_DROGUE_CONTINUITY),
            bit(p, PYRO_DROGUE_FIRE),
            bit(p, PYRO_SHORT_CIRCUIT),
            r.air_brakes_commanded_extension.to_string(),
            r.air_brakes_actual_extension.to_string(),
            bit(v, VALID_AIRBRAKES_COMMANDED),
            bit(v, VALID_AIRBRAKES_ACTUAL),
        ])?;
    }

    w.flush()?;
    Ok(())
}

/// `list-files`: print a summary of what's stored on the VLF5.
pub fn list_files() -> Result<()> {
    let handle = find_and_open()?;
    send_request(&handle, CliRequest::List)?;
    let header = read_header(&handle)?;
    let (record_count, record_len, block_count) = decode_response_header(&header)
        .ok_or_else(|| anyhow!("device sent an invalid response header"))?;

    println!("VLF5 flight log:");
    println!("  records      : {}", record_count);
    println!(
        "  data blocks  : {} ({} bytes on card)",
        block_count,
        block_count as usize * BLOCK_SIZE
    );
    if record_len == RECORD_LEN_TAGGED {
        println!("  format       : tagged v2 (IMU + SLOW stream)");
    } else {
        println!(
            "  format       : v1 fixed ({} bytes, {} records/block)",
            record_len, RECORDS_PER_BLOCK_V1
        );
    }
    if record_count == 0 {
        println!("  (empty — nothing has been logged yet)");
    }
    Ok(())
}

/// `download-file <out.csv>`: pull the whole log and write it as CSV.
pub fn download_file(output: &str) -> Result<()> {
    let handle = find_and_open()?;
    send_request(&handle, CliRequest::Download)?;
    let data = read_response(&handle)?;
    let (log_record_count, records) = parse_records(&data)?;
    write_csv(output, &records)?;
    println!(
        "Wrote {} IMU row(s) from {} on-card record(s) to {}",
        records.len(),
        log_record_count,
        output
    );
    Ok(())
}

/// `clear-storage`: erase the log on the VLF5.
pub fn clear_storage() -> Result<()> {
    let handle = find_and_open()?;
    send_request(&handle, CliRequest::Clear)?;
    let _ack = read_header(&handle)?;
    println!("VLF5 storage cleared.");
    Ok(())
}
