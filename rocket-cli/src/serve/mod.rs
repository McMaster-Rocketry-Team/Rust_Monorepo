use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use anyhow::Result;

use embassy_sync::blocking_mutex::raw::RawMutex;
use firmware_common_new::{
    can_bus::messages::amp_overwrite::PowerOutputOverwrite,
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::{VLPGroundStation, VLPTXError},
        lora_config::LoraConfig,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket,
            amp_output_overwrite::AMPOutputOverwritePacket,
            change_mode::{ChangeModePacket, Mode},
            fire_pyro::{FirePyroPacket, PyroSelect},
            reset::{DeviceToReset, ResetPacket},
            set_target_apogee::SetTargetApogeePacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;
use tokio::task::spawn_blocking;

use crate::gs::{
    config::GroundStationConfig,
    rpc_radio::RpcRadio, serial_wrapper::SerialWrapper, vlp_client::VLPClientTrait,
    MultiThreadRawMutex
};

pub mod server;

#[cfg(feature = "server")]
pub async fn run_live_server(serial_path: &str) -> Result<()>{
    use crate::serve::server::start_server;

    let serial = serialport::new(serial_path, 115200)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| {
            anyhow::anyhow!("failed to open the ground station serial port {serial_path}: {e} (is exactly one GCM plugged in?)")
        })?;
    let mut serial = SerialWrapper::new(serial);

    let config = Arc::new(RwLock::new(GroundStationConfig::load()?));
    let mut client = LoraRpcClient::new(&mut serial);
    client.reset().await.map_err(|e| {
        anyhow::anyhow!("ground station did not respond to the reset handshake: {e:?} (check the GCM connection and try again)")
    })?;
    client
        .configure(LoraConfig {
            frequency: config.read().unwrap().frequency,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: config.read().unwrap().power,
        })
        .await
        .map_err(|e| anyhow::anyhow!("failed to configure the ground station radio: {e:?}"))?;
    let mut rpc_radio = RpcRadio::new(client, None);
    let vlp_gcm_client = Box::leak(Box::new(VLPGroundStation::<MultiThreadRawMutex>::new()));
    let vlp_key = config.read().unwrap().vlp_key.clone();
    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &vlp_key);

    struct VLPClientWrapper(&'static VLPGroundStation<MultiThreadRawMutex>);

    impl VLPClientTrait for VLPClientWrapper {
        fn send_nb(&self, packet: VLPUplinkPacket) {
            self.0.send_nb(packet);
        }

        fn try_get_send_result(&self) -> Option<std::result::Result<PacketStatus, VLPTXError>> {
            self.0.try_get_send_result()
        }

        fn try_receive(&self) -> Option<(VLPDownlinkPacket, PacketStatus)> {
            self.0.try_receive()
        }
    }

    let vlp_client = Box::leak(Box::new(VLPClientWrapper(vlp_gcm_client)));
    let config = config.clone();

    tokio::select! {
        _ = daemon.run() => {}
        _ = spawn_blocking(move || {
            start_server(vlp_client, config)
        }) => {}
    }

    Ok(())
}