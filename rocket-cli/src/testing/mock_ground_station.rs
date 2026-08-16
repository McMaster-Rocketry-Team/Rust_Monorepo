use firmware_common_new::can_bus::custom_status::payload_sdrm_custom_status::PayloadSDRMCustomStatus;
use std::sync::{Arc, RwLock};

use crate::gs::{config::GroundStationConfig, tui_task, vlp_client::VLPClientTrait};
use anyhow::Result;
use firmware_common_new::{
    can_bus::messages::{amp_status::PowerOutputStatus, vl_status::FlightStage},
    vlp::{
        client::VLPTXError,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket, telemetry::TelemetryPacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;
use tokio::task::spawn_blocking;

struct MockVLPClient {
    mock_packet: RwLock<Option<(VLPDownlinkPacket, PacketStatus)>>,
}

impl MockVLPClient {
    /// The packet the mock TUI renders. Deliberately a *mid-flight Mach
    /// lockout* snapshot rather than a fully-populated one: the deployment KF
    /// is frozen (`deployment_kf: None`) while the stage still reads `Ascent`,
    /// which is the state a fully-populated mock could never show and the one
    /// the display has to get right — an operator seeing a blank altitude
    /// under an ascending rocket must read "filter locked out", not "on the
    /// ground at 0m".
    ///
    /// Icarus is online but has not sent an `IcarusStatusMessage` yet, and one
    /// EPM rail failed to read, so the other two flavours of absence are on
    /// screen too. Everything else is present, so the mock still exercises the
    /// value path for each field.
    pub fn new() -> Self {
        Self {
            mock_packet: RwLock::new(Some((
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
                    Some(2800.0),
                    3000.0,
                    false,
                    false,
                    8.4,
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
                    },
                    Some(12600),
                    [Some(120), Some(340), Some(55), Some(780), Some(1500), None],
                    [Some(0), Some(1200), Some(34567)],
                )
                .into(),
                PacketStatus { rssi: -40, snr: 6 },
            ))),
        }
    }
}

impl VLPClientTrait for MockVLPClient {
    fn send_nb(&self, _packet: VLPUplinkPacket) {
        unimplemented!()
    }

    fn try_get_send_result(&self) -> Option<std::result::Result<PacketStatus, VLPTXError>> {
        None
    }

    fn try_receive(&self) -> Option<(VLPDownlinkPacket, PacketStatus)> {
        self.mock_packet.write().unwrap().take()
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
