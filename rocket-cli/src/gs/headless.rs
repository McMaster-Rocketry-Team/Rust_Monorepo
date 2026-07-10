//! Non-interactive ground-station driver for scripted / HIL use.
//!
//! Two entry points, both reusing the same VLP ground-station plumbing as the TUI:
//!   * [`control_session`] — persistent session. Streams every downlink to stdout as
//!     one JSON line and reads operator commands (one per line) from stdin.
//!   * [`send_uplink_oneshot`] — connect, send a single uplink, wait for ack, print
//!     the JSON result, exit.
//!
//! All logs go to `.rocket-cli.log` only (stdout logging is disabled) so stdout is
//! pure JSON, one object per line, flushed immediately.

use std::io::{BufRead, Write};
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use base64::Engine as _;
use firmware_common_new::{
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::{VLPGroundStation, VLPTXError},
        lora_config::LoraConfig,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket,
            change_mode::{ChangeModePacket, Mode},
            fire_pyro::{FirePyroPacket, PyroSelect},
            reset::{DeviceToReset, ResetPacket},
            set_target_apogee::SetTargetApogeePacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;
use serde_json::{Value, json};
use tokio::sync::mpsc;

use crate::{
    enable_stdout_logging,
    gs::{MultiThreadRawMutex, config::GroundStationConfig, rpc_radio::RpcRadio, serial_wrapper::SerialWrapper},
};

/// Seconds to wait for a send-and-ack cycle. `send()` blocks until the rocket's next
/// downlink window (up to ~5 s) and then the uplink+ack round-trip, so keep this well
/// above the downlink period. A timeout means "no downlink / wrong frequency or key".
const SEND_TIMEOUT_SECS: u64 = 15;

// ---------------------------------------------------------------------------
// Link parameters
// ---------------------------------------------------------------------------

/// Resolved link parameters, from CLI flags falling back to `ground-station.toml`.
pub struct LinkParams {
    pub frequency: u32,
    pub power: i32,
    pub vlp_key: [u8; 32],
}

impl LinkParams {
    pub fn resolve(
        frequency: Option<u32>,
        power: Option<i32>,
        vlp_key: Option<String>,
    ) -> Result<Self> {
        let config = GroundStationConfig::load()?;
        let vlp_key = match vlp_key {
            Some(s) => decode_key(&s)?,
            None => config.vlp_key,
        };
        Ok(Self {
            frequency: frequency.unwrap_or(config.frequency),
            power: power.unwrap_or(config.power),
            vlp_key,
        })
    }
}

fn decode_key(s: &str) -> Result<[u8; 32]> {
    let bytes = base64::engine::general_purpose::STANDARD
        .decode(s.trim())
        .map_err(|e| anyhow!("--vlp-key is not valid base64: {e}"))?;
    bytes
        .try_into()
        .map_err(|v: Vec<u8>| anyhow!("--vlp-key must decode to 32 bytes, got {}", v.len()))
}

// ---------------------------------------------------------------------------
// Command model
// ---------------------------------------------------------------------------

/// A parsed operator command: either an uplink to sign+send, or a session control.
enum Command {
    Uplink { packet: VLPUplinkPacket, name: String },
    SetFrequency(u32),
    SetPower(i32),
    Quit,
}

/// Parse one command line (verb + args). Shared by `control` (stdin) and
/// `send-uplink` (argv). Errors carry a human-readable reason.
fn parse_command(line: &str) -> Result<Command> {
    let mut it = line.split_whitespace();
    let verb = it.next().ok_or_else(|| anyhow!("empty command"))?;
    let rest: Vec<&str> = it.collect();

    let uplink = |packet: VLPUplinkPacket, name: String| Ok(Command::Uplink { packet, name });

    match verb {
        "arm" => uplink(
            VLPUplinkPacket::ChangeMode(ChangeModePacket { mode: Mode::Armed }),
            "arm".into(),
        ),
        "mode" => {
            let m = *rest.first().ok_or_else(|| {
                anyhow!("mode requires an argument: low-power|self-test|armed|landed|demo")
            })?;
            let mode = match m {
                "low-power" | "lowpower" => Mode::LowPower,
                "self-test" | "selftest" => Mode::SelfTest,
                "armed" | "arm" => Mode::Armed,
                "landed" => Mode::Landed,
                "demo" => Mode::Demo,
                other => bail!("unknown mode '{other}'"),
            };
            uplink(
                VLPUplinkPacket::ChangeMode(ChangeModePacket { mode }),
                format!("mode {m}"),
            )
        }
        "target-apogee" | "target_apogee" => {
            let v = rest
                .first()
                .ok_or_else(|| anyhow!("target-apogee requires a value in meters"))?
                .parse::<f32>()
                .map_err(|_| anyhow!("target-apogee must be a number"))?;
            // Reject NaN/inf/negative before building the packet: NaN slips past the
            // fixed-point clamp and panics `num_traits::cast(...).unwrap()`, which in a
            // live `control` session would abort and tear down the daemon mid-flight.
            if !v.is_finite() || v < 0.0 {
                bail!("target-apogee must be a finite, non-negative number of meters");
            }
            uplink(
                VLPUplinkPacket::SetTargetApogee(SetTargetApogeePacket::new(v)),
                format!("target-apogee {v}"),
            )
        }
        "fire-pyro" | "fire_pyro" => {
            let p = *rest
                .first()
                .ok_or_else(|| anyhow!("fire-pyro requires: main|drogue"))?;
            let pyro = match p {
                "main" => PyroSelect::PyroMain,
                "drogue" => PyroSelect::PyroDrogue,
                other => bail!("unknown pyro '{other}' (expected main|drogue)"),
            };
            uplink(
                VLPUplinkPacket::FirePyro(FirePyroPacket { pyro }),
                format!("fire-pyro {p}"),
            )
        }
        "reset" => {
            let d = *rest.first().unwrap_or(&"all");
            let device = match d {
                "all" => DeviceToReset::All,
                "void-lake" | "voidlake" | "vl" => DeviceToReset::VoidLake,
                "amp" => DeviceToReset::AMP,
                "icarus" => DeviceToReset::Icarus,
                other => bail!("unknown reset device '{other}' (expected all|void-lake|amp|icarus)"),
            };
            uplink(
                VLPUplinkPacket::Reset(ResetPacket { device }),
                format!("reset {d}"),
            )
        }
        "set-frequency" | "set_frequency" | "freq" => {
            let f = rest
                .first()
                .ok_or_else(|| anyhow!("set-frequency requires a value in Hz"))?
                .parse::<u32>()
                .map_err(|_| anyhow!("set-frequency must be an integer in Hz"))?;
            Ok(Command::SetFrequency(f))
        }
        "set-power" | "set_power" | "power" => {
            let p = rest
                .first()
                .ok_or_else(|| anyhow!("set-power requires a value in dBm"))?
                .parse::<i32>()
                .map_err(|_| anyhow!("set-power must be an integer in dBm"))?;
            Ok(Command::SetPower(p))
        }
        "quit" | "exit" => Ok(Command::Quit),
        other => bail!("unknown command '{other}'"),
    }
}

// ---------------------------------------------------------------------------
// JSON output
// ---------------------------------------------------------------------------

fn emit(value: Value) {
    let mut out = std::io::stdout().lock();
    match writeln!(out, "{value}") {
        Ok(()) => {
            let _ = out.flush();
        }
        // stdout consumer went away: exit cleanly instead of panicking (`println!` would),
        // which would otherwise tear down the daemon mid-flight on the next telemetry emit.
        Err(e) if e.kind() == std::io::ErrorKind::BrokenPipe => std::process::exit(0),
        // Other transient write error: drop this line, keep the session alive.
        Err(_) => {}
    }
}

/// Serialize a downlink packet + link status to a JSON object (one line).
fn downlink_json(packet: &VLPDownlinkPacket, status: &PacketStatus) -> Value {
    match packet {
        VLPDownlinkPacket::Telemetry(p) => json!({
            "type": "telemetry",
            "rssi": status.rssi, "snr": status.snr,
            "flight_stage": format!("{:?}", p.flight_stage()),
            "altitude_agl": p.altitude_agl(),
            "max_altitude_agl": p.max_altitude_agl(),
            "air_speed": p.air_speed(),
            "max_air_speed": p.max_air_speed(),
            "tilt_deg": p.tilt_deg(),
            "airbrakes_cmd_pct": p.air_brakes_commanded_extension_percentage(),
            "airbrakes_actual_pct": p.air_brakes_actual_extension_percentage(),
            "pyro_main_continuity": p.pyro_main_continuity(),
            "pyro_drogue_continuity": p.pyro_drogue_continuity(),
            "vl_battery_v": p.vl_battery_v(),
            "shared_battery_v": p.shared_battery_v(),
            "air_temperature": p.air_temperature(),
            "satellites": p.num_of_fix_satellites(),
            "lat": p.lat(), "lon": p.lon(),
            "icarus_online": p.icarus_online(),
            "amp_online": p.amp_online(),
        }),
        VLPDownlinkPacket::LowPowerTelemetry(p) => json!({
            "type": "low_power_telemetry",
            "rssi": status.rssi, "snr": status.snr,
            "gps_fixed": p.gps_fixed,
            "satellites": p.num_of_fix_satellites(),
            "air_temperature": p.air_temperature(),
            "vl_battery_v": p.vl_battery_v(),
            "shared_battery_v": p.shared_battery_v(),
            "amp_online": p.amp_online,
        }),
        VLPDownlinkPacket::SelfTestResult(p) => json!({
            "type": "self_test_result",
            "rssi": status.rssi, "snr": status.snr,
            "imu_ok": p.imu_ok, "baro_ok": p.baro_ok, "mag_ok": p.mag_ok,
            "gps_ok": p.gps_ok, "sd_ok": p.sd_ok, "can_bus_ok": p.can_bus_ok,
            "main_continuity": p.main_continuity, "drogue_continuity": p.drogue_continuity,
        }),
        VLPDownlinkPacket::LandedTelemetry(p) => json!({
            "type": "landed_telemetry",
            "rssi": status.rssi, "snr": status.snr,
            "satellites": p.num_of_fix_satellites(),
            "lat": p.lat(), "lon": p.lon(),
            "vl_battery_v": p.battery_v(),
            "shared_battery_v": p.shared_battery_v(),
            "amp_online": p.amp_online(),
        }),
        VLPDownlinkPacket::GPSBeacon(p) => json!({
            "type": "gps_beacon",
            "rssi": status.rssi, "snr": status.snr,
            "satellites": p.num_of_fix_satellites(),
            "lat": p.lat(), "lon": p.lon(),
            "altitude_asl": p.altitude_asl(),
            "air_temperature": p.air_temperature(),
            "battery_v": p.battery_v(),
        }),
        VLPDownlinkPacket::Ack(_) => json!({
            "type": "ack_downlink", "rssi": status.rssi, "snr": status.snr,
        }),
    }
}

/// Result of a single uplink attempt.
enum SendOutcome {
    /// Transmitted and acknowledged.
    Ack(PacketStatus),
    /// Not acknowledged: the daemon attempted the uplink but got no valid ack (e.g. wrong
    /// key), or the GCM/radio reported a transmit failure. The packet may or may not have
    /// gone out over the air, but it was not confirmed.
    Nack(VLPTXError),
    /// No downlink arrived within the deadline; the queued packet was retracted and is
    /// guaranteed NOT to have been transmitted.
    TimedOut,
}

fn send_outcome_json(name: &str, outcome: &SendOutcome) -> Value {
    match outcome {
        SendOutcome::Ack(status) => json!({
            "type": "ack", "command": name, "rssi": status.rssi, "snr": status.snr,
        }),
        SendOutcome::Nack(e) => json!({
            "type": "nack", "command": name, "error": format!("{:?}", e),
        }),
        SendOutcome::TimedOut => json!({
            "type": "timeout", "command": name,
            "message": "no ack within timeout (no downlink? wrong frequency/key?)",
        }),
    }
}

/// Queue an uplink and wait for its result, up to `deadline`.
///
/// Cancel-safe by construction. `VLPGroundStation::send()` is not cancel-safe (it leaves
/// the packet queued if its future is dropped), so instead of `timeout(send())` we queue
/// with `send_nb` and poll. The retract-vs-committed decision is made SYNCHRONOUSLY — no
/// `.await` between observing "no result yet" and calling `take_pending_uplink()` — so in
/// the cooperative single-task `select!` the sibling daemon cannot take the packet in the
/// gap. Therefore:
///   * still queued at the deadline  -> retracted, guaranteed un-transmitted (`TimedOut`).
///   * already taken by the daemon   -> transmit committed; we always wait for the real
///     Ack/Nack (a sent packet is never misreported as a timeout). The daemon's own 500 ms
///     ack window bounds this wait, so a committed send always resolves shortly.
///
/// The daemon must be driven concurrently (a sibling branch of the same `select!`).
async fn send_uplink(
    vlp: &VLPGroundStation<MultiThreadRawMutex>,
    packet: VLPUplinkPacket,
    deadline: Duration,
) -> SendOutcome {
    let _ = vlp.try_get_send_result(); // drop any stale result from a prior send
    vlp.send_nb(packet);
    let start = std::time::Instant::now();
    let mut committed = false;
    loop {
        if let Some(result) = vlp.try_get_send_result() {
            return match result {
                Ok(status) => SendOutcome::Ack(status),
                Err(e) => SendOutcome::Nack(e),
            };
        }
        if !committed && start.elapsed() >= deadline {
            // Synchronous with the poll above (no await in between): a false return means
            // the daemon already took the packet, so the transmit is committed.
            if vlp.take_pending_uplink() {
                return SendOutcome::TimedOut;
            }
            committed = true;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

// ---------------------------------------------------------------------------
// Radio bring-up (shared)
// ---------------------------------------------------------------------------

/// Open the GCM serial port and configure the SX126x for VLP at the given params.
/// Returns the configured `RpcRadio` (and keeps the underlying serial alive via the
/// caller-owned `SerialWrapper` passed by mutable ref).
async fn open_radio<'a>(
    serial: &'a mut SerialWrapper,
    frequency: u32,
    power: i32,
) -> Result<RpcRadio<'a>> {
    let mut client = LoraRpcClient::new(serial);
    client
        .reset()
        .await
        .map_err(|e| anyhow!("GCM RPC reset failed: {e:?}"))?;
    client
        .configure(LoraConfig {
            frequency,
            sf: 12,
            bw: 250000,
            cr: 8,
            power,
        })
        .await
        .map_err(|e| anyhow!("GCM RPC configure failed: {e:?}"))?;
    Ok(RpcRadio::new(client, None))
}

fn open_serial(serial_path: &str) -> Result<SerialWrapper> {
    let port = serialport::new(serial_path, 115200)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| anyhow!("failed to open GCM serial {serial_path}: {e}"))?;
    Ok(SerialWrapper::new(port))
}

// ---------------------------------------------------------------------------
// control: persistent streaming session
// ---------------------------------------------------------------------------

enum SessionEnd {
    Reconfigure { frequency: u32, power: i32 },
    Quit,
}

/// Persistent bidirectional session: telemetry JSON to stdout, commands from stdin.
pub async fn control_session(serial_path: &str, params: LinkParams) -> Result<()> {
    enable_stdout_logging(false);

    // Blocking stdin reader → unbounded channel. Only ships raw lines; the VLP client
    // is only ever touched from the async session task below.
    let (tx, mut rx) = mpsc::unbounded_channel::<String>();
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        for line in stdin.lock().lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        // EOF or error → drop tx → rx.recv() yields None → session quits.
    });

    let key = params.vlp_key;
    let mut frequency = params.frequency;
    let mut power = params.power;

    loop {
        match run_control_session(serial_path, frequency, power, &key, &mut rx).await? {
            SessionEnd::Reconfigure {
                frequency: f,
                power: p,
            } => {
                frequency = f;
                power = p;
                emit(json!({"type": "link", "event": "reconfiguring",
                    "frequency": frequency, "power": power}));
            }
            SessionEnd::Quit => {
                emit(json!({"type": "link", "event": "closed"}));
                return Ok(());
            }
        }
    }
}

async fn run_control_session(
    serial_path: &str,
    frequency: u32,
    power: i32,
    key: &[u8; 32],
    rx: &mut mpsc::UnboundedReceiver<String>,
) -> Result<SessionEnd> {
    let mut serial = open_serial(serial_path)?;
    let mut rpc_radio = open_radio(&mut serial, frequency, power).await?;
    let vlp = VLPGroundStation::<MultiThreadRawMutex>::new();
    let mut daemon = vlp.daemon(&mut rpc_radio, key);

    emit(json!({"type": "link", "event": "configured",
        "frequency": frequency, "power": power}));

    let end = tokio::select! {
        _ = daemon.run() => SessionEnd::Quit,
        _ = drain_downlinks(&vlp) => SessionEnd::Quit,
        e = handle_commands(&vlp, rx, frequency, power) => e,
    };
    Ok(end)
}

/// Forever: forward each received downlink to stdout as JSON.
async fn drain_downlinks(vlp: &VLPGroundStation<MultiThreadRawMutex>) {
    loop {
        let (packet, status) = vlp.receive().await;
        emit(downlink_json(&packet, &status));
    }
}

/// Process operator commands until stdin closes, a `quit`, or a reconfigure request.
async fn handle_commands(
    vlp: &VLPGroundStation<MultiThreadRawMutex>,
    rx: &mut mpsc::UnboundedReceiver<String>,
    frequency: u32,
    power: i32,
) -> SessionEnd {
    loop {
        let line = match rx.recv().await {
            Some(l) => l,
            None => {
                // stdin closed. Stop accepting commands but keep the daemon and the
                // telemetry drain alive, so a piped one-shot (`echo cmd | control > log`)
                // still streams telemetry. The session ends on `quit`, SIGINT, or kill.
                std::future::pending::<()>().await;
                unreachable!();
            }
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        match parse_command(line) {
            Ok(Command::Quit) => return SessionEnd::Quit,
            Ok(Command::SetFrequency(f)) => {
                return SessionEnd::Reconfigure {
                    frequency: f,
                    power,
                };
            }
            Ok(Command::SetPower(p)) => {
                return SessionEnd::Reconfigure {
                    frequency,
                    power: p,
                };
            }
            Ok(Command::Uplink { packet, name }) => {
                emit(json!({"type": "sending", "command": name}));
                let outcome =
                    send_uplink(vlp, packet, Duration::from_secs(SEND_TIMEOUT_SECS)).await;
                emit(send_outcome_json(&name, &outcome));
            }
            Err(e) => emit(json!({"type": "error", "message": format!("{e}")})),
        }
    }
}

// ---------------------------------------------------------------------------
// send-uplink: one-shot
// ---------------------------------------------------------------------------

/// Connect, send a single uplink, wait for ack, print the JSON result, and exit.
pub async fn send_uplink_oneshot(
    serial_path: &str,
    params: LinkParams,
    command: &str,
) -> Result<()> {
    enable_stdout_logging(false);

    let (packet, name) = match parse_command(command)? {
        Command::Uplink { packet, name } => (packet, name),
        _ => bail!(
            "send-uplink only accepts uplink commands (arm, mode, target-apogee, fire-pyro, reset); \
             use `control` for set-frequency/set-power/quit"
        ),
    };

    let mut serial = open_serial(serial_path)?;
    let mut rpc_radio = open_radio(&mut serial, params.frequency, params.power).await?;
    let vlp = VLPGroundStation::<MultiThreadRawMutex>::new();
    let mut daemon = vlp.daemon(&mut rpc_radio, &params.vlp_key);

    // daemon.run() never returns; it drives the radio while send_uplink polls the result.
    let outcome = tokio::select! {
        _ = daemon.run() => SendOutcome::TimedOut,
        o = send_uplink(&vlp, packet, Duration::from_secs(SEND_TIMEOUT_SECS)) => o,
    };

    emit(send_outcome_json(&name, &outcome));
    match outcome {
        SendOutcome::Ack(_) => Ok(()),
        SendOutcome::Nack(e) => bail!("uplink '{name}' not acked: {e:?}"),
        SendOutcome::TimedOut => {
            bail!("timed out waiting for ack from '{name}' (no downlink? wrong frequency/key?)")
        }
    }
}
