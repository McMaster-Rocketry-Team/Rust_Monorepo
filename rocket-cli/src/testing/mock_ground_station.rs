use firmware_common_new::can_bus::custom_status::{
    NodeCustomStatusExt, payload_sdrm_custom_status::PayloadSDRMCustomStatus,
};
use std::{
    sync::{
        Arc, RwLock,
        atomic::{AtomicUsize, Ordering},
    },
    time::{Duration, Instant},
};

use crate::gs::{config::GroundStationConfig, tui_task, vlp_client::VLPClientTrait};
use anyhow::Result;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use firmware_common_new::{
    can_bus::messages::{
        amp_status::PowerOutputStatus,
        node_status::{NodeHealth, NodeMode},
        vl_status::FlightStage,
    },
    vlp::{
        client::VLPTXError,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket,
            landed_telemetry::LandedTelemetryPacket,
            low_power_telemetry::LowPowerTelemetryPacket,
            self_test_result::{NodeStatus, SelfTestResultPacketBuilder},
            telemetry::TelemetryPacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;
use tokio::task::spawn_blocking;

/// How long each mocked packet stays on screen before the next replaces it.
const PACKET_DWELL: Duration = Duration::from_secs(4);

struct MockVLPClient {
    /// One packet per downlink type, cycled so every panel is exercised.
    packets: Vec<(VLPDownlinkPacket, PacketStatus)>,
    next: AtomicUsize,
    last_emitted: RwLock<Option<Instant>>,
}

impl MockVLPClient {
    /// Every downlink packet type the display can render, cycled on a timer.
    ///
    /// Cycling rather than emitting one packet forever is what makes the mock
    /// worth running: each packet type has its own panel layout, and three of
    /// the four were previously unreachable here, so a mistake in them showed
    /// up for the first time on a live link. Rotating also exercises the
    /// field-cache reset in `DownlinkPacketDisplay::update`, which only
    /// happens when the packet type changes.
    pub fn new() -> Self {
        let status = PacketStatus { rssi: -40, snr: 6 };
        Self {
            packets: vec![
                (Self::telemetry(), status),
                (Self::self_test_result(), status),
                (Self::low_power_telemetry(), status),
                (Self::landed_telemetry(), status),
            ],
            next: AtomicUsize::new(0),
            last_emitted: RwLock::new(None),
        }
    }

    /// A mid-flight self test with the payload stack half up: EPM and SEM both
    /// answering, rails energized, one experiment running, SDRM logging but
    /// SEM not.
    ///
    /// The flags are carried in the SDRM's 11-bit node custom status, so they
    /// are built the way the payload builds them — through `to_u16` — rather
    /// than as a hand-written integer. That makes this a real check of the
    /// display's `from_u16` decode instead of a check that two literals match.
    fn self_test_result() -> VLPDownlinkPacket {
        let stack = PayloadSDRMCustomStatus {
            epm_alive: true,
            sem_alive: true,
            epm_rails_on: true,
            exp1_active: false,
            exp2_active: true,
            exp3_active: false,
            sdrm_sd_logging: true,
            sem_sd_logging: false,
            // Self test runs before the Armed transition, so the arm sequence
            // has not started: all three bits clear is the honest state here,
            // and it is also the one the ground has to tell apart from a
            // sequence that started and died.
            arm_seq_running: false,
            arm_seq_complete: false,
            arm_seq_fault: false,
        };

        let builder = SelfTestResultPacketBuilder::<NoopRawMutex>::new();
        builder.update(|state| {
            state.imu_ok = true;
            state.baro_ok = true;
            state.mag_ok = true;
            state.gps_ok = true;
            state.sd_ok = true;
            state.can_bus_ok = true;
            state.amp_out1_power_good = true;
            state.amp_out2_power_good = true;
            state.amp_out3_power_good = false;
            state.main_continuity = true;
            state.drogue_continuity = false;
            state.amp = NodeStatus {
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                rebooted_in_last_5s: false,
                custom_status: 0,
            };
            state.icarus = NodeStatus {
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                rebooted_in_last_5s: true,
                custom_status: 0,
            };
            state.ozys = NodeStatus {
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                rebooted_in_last_5s: false,
                custom_status: 0,
            };
            state.payload_sdrm = NodeStatus {
                health: NodeHealth::Healthy,
                mode: NodeMode::Operational,
                rebooted_in_last_5s: false,
                custom_status: stack.to_u16(),
            };
            // Uncomment to check the offline path: the eight stack flags must
            // go to `n/a`, not to a wall of red `F`s.
            // state.payload_sdrm = NodeStatus::offline();
        });
        builder.create_packet().into()
    }

    fn low_power_telemetry() -> VLPDownlinkPacket {
        LowPowerTelemetryPacket::new(
            0,
            9,
            true,
            Some((10.1, 20.2)),
            7.4,
            true,
            Some(8.4),
            21.0,
            Some(12600),
        )
        .into()
    }

    fn landed_telemetry() -> VLPDownlinkPacket {
        LandedTelemetryPacket::new(
            0,
            Some((10.1, 20.2)),
            9,
            7.2,
            true,
            false,
            Some(8.3),
            false,
            PowerOutputStatus::PowerGood,
            false,
            PowerOutputStatus::PowerGood,
            true,
            PowerOutputStatus::PowerBad,
        )
        .into()
    }

    /// Deliberately a *mid-flight Mach lockout* snapshot rather than a
    /// fully-populated one: the deployment KF is frozen
    /// (`deployment_kf: None`) while the stage still reads `Ascent`, which is
    /// the state a fully-populated mock could never show and the one the
    /// display has to get right — an operator seeing a blank altitude under an
    /// ascending rocket must read "filter locked out", not "on the ground at
    /// 0m".
    ///
    /// Icarus is online but has not sent an `IcarusStatusMessage` yet, and one
    /// EPM rail failed to read, so the other two flavours of absence are on
    /// screen too. Everything else is present, so the mock still exercises the
    /// value path for each field.
    fn telemetry() -> VLPDownlinkPacket {
        TelemetryPacket::new(
                    0,
                    true,
                    12,
                    Some((10.1, 20.2)),
                    7.4,
                    25.5,
                    false,
                    false,
                    None,
                    // Latched, so the apogee reached so far stays readable
                    // through the lockout that blanks the live altitude.
                    Some(1500.0),
                    Some(4.0),
                    FlightStage::Ascent,
                    true,
                    true,
                    Some(2800.0),
                    3000.0,
                    false,
                    false,
                    Some(8.4),
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    PowerOutputStatus::Disabled,
                    true,
                    false,
                    0.0,
                    None,
                    false,
                    false,
                    false,
                    false,
                    // A realistic mid-flight stack: both boards up, rails
                    // energized, one experiment running, SDRM logging but SEM
                    // not. A mix rather than all-false, so the panel shows both
                    // colours of flag and a wrong one is visible on sight.
                    PayloadSDRMCustomStatus {
                        epm_alive: true,
                        sem_alive: true,
                        epm_rails_on: true,
                        exp1_active: false,
                        exp2_active: true,
                        exp3_active: false,
                        sdrm_sd_logging: true,
                        sem_sd_logging: false,
                        arm_seq_running: false,
                        arm_seq_complete: true,
                        arm_seq_fault: false,
                    },
                    Some(12600),
                    [Some(120), Some(340), Some(55), Some(780), Some(1500), None],
                    [Some(0), Some(1200), Some(34567)],
                )
        .into()
    }
}

impl VLPClientTrait for MockVLPClient {
    fn send_nb(&self, _packet: VLPUplinkPacket) {
        unimplemented!()
    }

    fn try_get_send_result(&self) -> Option<std::result::Result<PacketStatus, VLPTXError>> {
        None
    }

    /// Hand out the next packet once `PACKET_DWELL` has passed, so each panel
    /// stays on screen long enough to read before the next replaces it. The
    /// TUI polls this far faster than that, hence the timer rather than a
    /// packet per call.
    fn try_receive(&self) -> Option<(VLPDownlinkPacket, PacketStatus)> {
        let mut last_emitted = self.last_emitted.write().unwrap();
        if let Some(last) = *last_emitted
            && last.elapsed() < PACKET_DWELL
        {
            return None;
        }
        *last_emitted = Some(Instant::now());

        let index = self.next.fetch_add(1, Ordering::Relaxed) % self.packets.len();
        Some(self.packets[index].clone())
    }
}

pub async fn mock_ground_station_tui() -> Result<()> {
    let config = Arc::new(RwLock::new(GroundStationConfig::load()?));

    let client = Box::leak(Box::new(MockVLPClient::new()));

    spawn_blocking(move || {
        tui_task(client, config)
    }).await??;

    Ok(())
}
