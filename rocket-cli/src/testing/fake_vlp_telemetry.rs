use std::time::Duration;

use crate::{
    args::SendVLPTelemetryArgs,
    gs::{
        find_ground_station::find_ground_station, rpc_radio::RpcRadio,
        serial_wrapper::SerialWrapper,
    },
};
use anyhow::Result;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use firmware_common_new::{
    can_bus::messages::{amp_status::PowerOutputStatus, node_status::NodeHealth},
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPAvionics,
        lora_config::LoraConfig,
        packets::{VLPDownlinkPacket, gps_beacon::GPSBeaconPacket, telemetry::TelemetryPacket},
    },
};
use log::info;

pub async fn send_fake_vlp_telemetry(args: SendVLPTelemetryArgs) -> Result<()> {
    let serial_path = find_ground_station().await?;
    let serial = serialport::new(serial_path, 115200)
        .timeout(Duration::from_secs(5))
        .open()
        .unwrap();

    let mut serial = SerialWrapper::new(serial);

    let mut client = LoraRpcClient::new(&mut serial);
    client.reset().await.unwrap();
    client
        .configure(LoraConfig {
            frequency: args.frequency,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        })
        .await
        .unwrap();
    let mut rpc_radio = RpcRadio::new(client, Some(Box::new(|| {
        info!("successfully transmitted a VLP package");
        std::process::exit(0);
    })));

    let vlp_avionics_client = VLPAvionics::<ThreadModeRawMutex>::new();
    let vlp_key = [0u8;32];
    let mut daemon = vlp_avionics_client.daemon(&mut rpc_radio, &vlp_key);

    let packet: VLPDownlinkPacket = if let Some(altitude_agl) = args.altitude_agl {
        TelemetryPacket::new(
            0,
            true,
            12,
            Some((args.latitude, args.longitude)),
            7.4,
            25.5,
            25.6,
            false,
            false,
            altitude_agl,
            altitude_agl,
            altitude_agl,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            0,
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
            50.0,
            0.0,
            0.5,
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
        .into()
    } else {
        GPSBeaconPacket::new(
            0,
            Some((args.latitude, args.longitude)),
            12,
            7.4,
            0.0,
            0.0,
            false,
            false,
            false,
            false,
            false,
        )
        .into()
    };

    vlp_avionics_client.send(packet);

    daemon.run().await;

    Ok(())
}
