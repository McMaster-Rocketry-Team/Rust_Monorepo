use firmware_common_new::can_bus::custom_status::payload_sdrm_custom_status::PayloadSDRMCustomStatus;
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
    can_bus::messages::{
        amp_status::PowerOutputStatus, vl_status::FlightStage,
    },
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPAvionics,
        lora_config::LoraConfig,
        packets::{
            VLPDownlinkPacket,
            telemetry::{DeploymentKfState, IcarusAirBrakesState, TelemetryPacket},
        },
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
    let mut rpc_radio = RpcRadio::new(
        client,
        Some(Box::new(|success| {
            if success {
                info!("successfully transmitted a VLP package");
                std::process::exit(0);
            } else {
                info!("failed to transmit a VLP package");
                std::process::exit(1);
            }
        })),
    );

    let vlp_avionics_client = VLPAvionics::<ThreadModeRawMutex>::new();
    let vlp_key = [0u8; 32];
    let mut daemon = vlp_avionics_client.daemon(&mut rpc_radio, &vlp_key);

    let altitude_agl = args.altitude_agl.unwrap_or(0.0);
    // The opposite shape to the mock ground station's packet: everything the
    // operator asked for on the command line is present, because this one is
    // aimed at a real receiver during a range test and a blanked-out field
    // there would look like a link problem rather than a deliberate absence.
    let packet: VLPDownlinkPacket = {
        TelemetryPacket::new(
            0,
            true,
            12,
            Some((args.latitude, args.longitude)),
            7.4,
            25.5,
            false,
            false,
            Some(DeploymentKfState {
                altitude_agl,
                vertical_velocity: 0.0,
            }),
            Some(altitude_agl),
            Some(0.0),
            FlightStage::Armed,
            false,
            Some(altitude_agl),
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
            false,
            false,
            0.0,
            Some(IcarusAirBrakesState {
                actual_extension_percentage: 0.0,
                servo_temp: 50.0,
            }),
            false,
            false,
            false,
            false,
            PayloadSDRMCustomStatus::new(),
            Some(12600),
            [Some(120), Some(340), Some(55), Some(780), Some(1500), None],
            [Some(0), Some(1200), Some(34567)],
        )
        .into()
    };

    vlp_avionics_client.send(packet);

    daemon.run().await;

    Ok(())
}
