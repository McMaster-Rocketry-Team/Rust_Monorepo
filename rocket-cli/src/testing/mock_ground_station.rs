use std::sync::{Arc, RwLock};

use crate::gs::{config::GroundStationConfig, tui_task, vlp_client::VLPClientTrait};
use anyhow::Result;
use firmware_common_new::{
    can_bus::messages::{
        amp_status::PowerOutputStatus, vl_status::FlightStage, node_status::NodeHealth,
    },
    vlp::{
        client::VLPTXError,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket, gps_beacon::GPSBeaconPacket,
            telemetry::TelemetryPacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;

struct MockVLPClient {
    mock_packet: RwLock<Option<(VLPDownlinkPacket, PacketStatus)>>,
}

impl MockVLPClient {
    pub fn new() -> Self {
        Self {
            // mock_packet: RwLock::new(Some((
            //     GPSBeaconPacket::new(
            //         0,
            //         Some((10.1, 20.2)),
            //         12,
            //         7.4,
            //         20.5,
            //         100.3,
            //         true,
            //         false,
            //         false,
            //         false,
            //         false,
            //     )
            //     .into(),
            //     PacketStatus { rssi: -40, snr: 6 },
            // ))),
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
                    10.0,
                    20.0,
                    0.0,
                    0.0,
                    0.0,
                    FlightStage::Armed,
                    false,
                    false,
                    8.4,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    false,
                    100.0,
                    false,
                    false,
                    100.0,
                    false,
                    false,
                    0.0,
                    0.0,
                    50.0,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    NodeHealth::Healthy,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    3.7,
                    40.0,
                    3.8,
                    40.1,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
                    false,
                    false,
                    3.7,
                    40.0,
                    3.8,
                    40.1,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
                    1.0,
                    false,
                    PowerOutputStatus::Disabled,
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

    tui_task(client, config).await?;

    Ok(())
}
