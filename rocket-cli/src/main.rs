mod args;
mod bluetooth;
mod connection_method;
mod elf_locator;
mod gen_key;
mod gs;
mod monitor;
mod probe;
mod testing;
mod usb;

use anyhow::bail;
use anyhow::{Result, anyhow};
use args::Cli;
use args::ModeSelect;
use args::TestingModeSelect;
use clap::Parser;
use connection_method::ConnectionMethod;
use connection_method::get_connection_method;
use gen_key::gen_ota_key;
use log::LevelFilter;
use monitor::monitor_tui;
use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo, available_ports};
use testing::decode_bluetooth_chunk::test_decode_bluetooth_chunk;
use testing::mock_connection_method::MockConnectionMethod;

use crate::gen_key::gen_vlp_key;
use crate::gs::ground_station_tui;

#[tokio::main]
async fn main() -> Result<()> {
    let args = Cli::parse();
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .filter_module("rocket_cli", LevelFilter::Trace)
        .filter_module("firmware_common_new", LevelFilter::Trace)
        .try_init()
        .ok();

    match args.mode {
        ModeSelect::Download(args) => {
            let mut connection_method = get_connection_method(
                Some(args.chip),
                Some(args.firmware_elf_path.clone()),
                Some(args.node_type),
                Some(args.secret_path),
            )
            .await?;

            connection_method.download().await?;
            monitor_tui(&mut connection_method, Some(&args.firmware_elf_path)).await?;

            connection_method.dispose().await
        }
        ModeSelect::Attach(args) => {
            let mut connection_method =
                get_connection_method(args.chip, args.elf, None, None).await?;

            monitor_tui(&mut connection_method, None).await?;

            connection_method.dispose().await
        }
        ModeSelect::GroundStation => {
            let ground_station_serial_ports = available_ports()
                .unwrap()
                .into_iter()
                .filter(|port| {
                    matches!(
                        port.port_type,
                        SerialPortType::UsbPort(UsbPortInfo {
                            vid: 0x120a,
                            pid: 0x0005,
                            ..
                        })
                    )
                })
                .collect::<Vec<SerialPortInfo>>();

            if ground_station_serial_ports.len() == 0 {
                bail!("No ground station connected")
            } else if ground_station_serial_ports.len() > 1 {
                bail!("More than one ground stations connected")
            }

            ground_station_tui(&ground_station_serial_ports[0].port_name).await
        }
        ModeSelect::GenVlpKey => gen_vlp_key(),
        ModeSelect::GenOtaKey(args) => gen_ota_key(args),
        ModeSelect::Testing(TestingModeSelect::DecodeBluetoothChunk(args)) => {
            test_decode_bluetooth_chunk(args).map_err(|e| anyhow!("{:?}", e))
        }
        ModeSelect::Testing(TestingModeSelect::MockConnection) => {
            let mut connection_method: Box<dyn ConnectionMethod> = Box::new(MockConnectionMethod);

            monitor_tui(&mut connection_method, None).await
        }
    }
}
