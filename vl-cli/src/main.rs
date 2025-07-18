#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::cmp::Ordering;
use std::time::Duration;

use anyhow::anyhow;
use anyhow::Result;
use clap::{Parser, Subcommand};
use clap_num::maybe_hex;
use embedded_hal_async::delay::DelayNs;
use firmware_common::common::console::vl_rpc::GCMPollDownlinkPacketResponse;
use firmware_common::common::vlp::packet::DeleteLogsPacket;
use firmware_common::common::vlp::packet::LowPowerModePacket;
use firmware_common::common::vlp::packet::ManualTriggerDeplotmentPacket;
use firmware_common::common::vlp::packet::ResetPacket;
use firmware_common::common::vlp::packet::SoftArmPacket;
use firmware_common::common::vlp::packet::VLPDownlinkPacket;
use firmware_common::common::vlp::packet::VLPUplinkPacket;
use firmware_common::common::vlp::packet::VerticalCalibrationPacket;
use firmware_common::common::vlp::telemetry_packet::TelemetryPacket;
use firmware_common::sg_rpc;
use firmware_common::vl_rpc;
use firmware_common::vl_rpc::RpcPacketStatus;
use log::LevelFilter;
use tokio::fs::read_to_string;
use tokio::time::sleep;
use tokio_serial::available_ports;
use vl_host_lib::common::list_files;
use vl_host_lib::common::probe_device_type;
use vl_host_lib::common::pull_file;
use vl_host_lib::create_serial;
use vl_host_lib::ozys::pull_ozys_data;
use vl_host_lib::vl::format_lora_key;
use vl_host_lib::vl::gen_lora_key;
use vl_host_lib::vl::json_to_device_config;
use vl_host_lib::vl::json_to_flight_profile;
use vl_host_lib::vl::pull_flight_data;
use vl_host_lib::vl::pull_vacuum_test;
use vlfs::FileID;
use vlfs::FileType;

struct Delay;

impl DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        sleep(Duration::from_nanos(ns as u64)).await;
    }
}

#[derive(Parser)]
#[command(name = "VL CLI")]
#[command(bin_name = "vl-cli")]
struct Cli {
    #[clap(subcommand)]
    mode: ModeSelect,
}

#[derive(Subcommand)]
enum ModeSelect {
    #[command(about = "List all the devices connected to the host")]
    Detect,

    #[command(about = "Void Lake specific commands")]
    VL(VLCli),

    #[command(about = "OZYS specific commands")]
    OZYS(OZYSCli),

    #[command(about = "Generate a new Lora key")]
    GenLoraKey,
}

#[derive(Parser)]
struct VLCli {
    serial: String,

    #[clap(subcommand)]
    command: VLCommands,
}

#[derive(Parser)]
struct OZYSCli {
    serial: String,

    #[clap(subcommand)]
    command: SGCommands,
}

#[derive(Subcommand)]
enum SGCommands {
    #[command(about = "Pull all the strain gauges readings from device")]
    PullData(PullDataArgs),

    #[command(about = "Clear all the data on the device")]
    ClearData,

    #[command(about = "Output realtime readings from the device")]
    RealTime(RealTimeArgs),

    LS(LSArgs),
    PullFile(PullArgs),

    #[command(about = "Reset device")]
    Reset,
}

#[derive(Subcommand)]
enum VLCommands {
    #[clap(subcommand)]
    GCMSendUplink(GCMUplinkPacket),
    GCMListen(GCMArgs),
    SetFlightProfile(FlightProfileArgs),
    SetDeviceConfig(DeviceConfigArgs),

    #[command(about = "Pull flight data from device")]
    PullFlight(PullDataArgs),

    #[command(about = "Pull vacuum test data from device")]
    PullVacuumTest(PullDataArgs),

    #[command(about = "Pull ground test data from device")]
    PullGroundTest(PullDataArgs),

    LS(LSArgs),
    PullFile(PullArgs),

    #[command(about = "Reset device")]
    Reset,
}

#[derive(Subcommand)]
enum GCMUplinkPacket {
    VerticalCalibration,
    SoftArm,
    SoftDisarm,
    LowPowerModeOn,
    LowPowerModeOff,
    Reset,
    DeleteLogs,
    ManualTriggerDeployment,
}

fn file_type_parser(s: &str) -> Result<FileType, String> {
    maybe_hex(s).map(FileType)
}

#[derive(clap::Args)]
#[command(about = "List files on the device")]
struct LSArgs {
    #[arg(value_parser=file_type_parser)]
    file_type: Option<FileType>,
}

fn file_id_parser(s: &str) -> Result<FileID, String> {
    maybe_hex(s).map(FileID)
}

#[derive(clap::Args)]
#[command(about = "Pull a file from the device")]
struct PullArgs {
    #[arg(value_parser=file_id_parser)]
    file_id: FileID,
    host_path: std::path::PathBuf,
}

#[derive(clap::Args)]
#[command(about = "Listen on VLP Downlink packet")]
struct GCMArgs {}

#[derive(clap::Args)]
#[command(about = "Set flight profile")]
struct FlightProfileArgs {
    profile_path: std::path::PathBuf,
}

#[derive(clap::Args)]
#[command(about = "Set device config")]
struct DeviceConfigArgs {
    config_path: std::path::PathBuf,
}

#[derive(clap::Args)]
struct PullDataArgs {
    save_folder: std::path::PathBuf,
}

#[derive(clap::Args)]
struct RealTimeArgs {
    channel: Option<usize>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Info)
        .try_init();

    let args = Cli::parse();

    match args.mode {
        ModeSelect::Detect => {
            let mut results = vec![];
            for port in available_ports().unwrap() {
                let probe_result = probe_device_type(port.port_name.clone()).await;
                results.push((port.port_name, probe_result));
            }
            results.sort_by(|a, b| match (&a.1, &b.1) {
                (Ok(_), Ok(_)) => Ordering::Equal,
                (Ok(_), _) => Ordering::Less,
                (_, Ok(_)) => Ordering::Greater,
                (_, _) => Ordering::Equal,
            });
            let mut count = 0;
            for (port_name, result) in &results {
                if let Ok(device_type) = result {
                    println!("{}: {}", port_name, device_type);
                    count += 1;
                }
            }
            println!("{} devices found", count);
        }
        ModeSelect::VL(VLCli { serial, command }) => {
            let mut serial = create_serial(serial)?;
            let mut client = vl_rpc::RpcClient::new(&mut serial, Delay);
            client.reset().await.map_err(|_| anyhow!("reset Error"))?;

            let timestamp = chrono::Utc::now().timestamp_micros() as f64 / 1000.0;
            match command {
                VLCommands::GCMSendUplink(uplink_packet) => {
                    let packet: VLPUplinkPacket = match uplink_packet {
                        GCMUplinkPacket::VerticalCalibration => {
                            VerticalCalibrationPacket { timestamp }.into()
                        }
                        GCMUplinkPacket::SoftArm => SoftArmPacket {
                            timestamp,
                            armed: true,
                        }
                        .into(),
                        GCMUplinkPacket::SoftDisarm => SoftArmPacket {
                            timestamp,
                            armed: false,
                        }
                        .into(),
                        GCMUplinkPacket::LowPowerModeOn => LowPowerModePacket {
                            timestamp,
                            enabled: true,
                        }
                        .into(),
                        GCMUplinkPacket::LowPowerModeOff => LowPowerModePacket {
                            timestamp,
                            enabled: false,
                        }
                        .into(),
                        GCMUplinkPacket::Reset => ResetPacket { timestamp }.into(),
                        GCMUplinkPacket::DeleteLogs => DeleteLogsPacket { timestamp }.into(),
                        GCMUplinkPacket::ManualTriggerDeployment => {
                            ManualTriggerDeplotmentPacket { timestamp }.into()
                        }
                    };

                    let result = client.g_c_m_send_uplink_packet(packet).await.unwrap();
                    println!("{:?}", result);
                    loop {
                        match client.g_c_m_poll_downlink_packet().await {
                            Ok(GCMPollDownlinkPacketResponse {
                                packet: Some((packet, status)),
                            }) => {
                                if let VLPDownlinkPacket::TelemetryPacket(packet) = packet {
                                    print_telemetry_packet(&packet, &status);
                                }
                            }
                            Err(e) => {
                                println!("{:?}", e);
                            }
                            _ => {}
                        }
                        sleep(Duration::from_millis(100)).await;
                    }
                }
                VLCommands::GCMListen(_) => loop {
                    match client.g_c_m_poll_downlink_packet().await {
                        Ok(GCMPollDownlinkPacketResponse {
                            packet: Some((packet, status)),
                        }) => {
                            if let VLPDownlinkPacket::TelemetryPacket(packet) = packet {
                                print_telemetry_packet(&packet, &status);
                            }
                        }
                        Err(e) => {
                            println!("{:?}", e);
                        }
                        _ => {}
                    }
                    sleep(Duration::from_millis(100)).await;
                },
                VLCommands::SetFlightProfile(args) => {
                    let json = read_to_string(args.profile_path).await?;
                    let profile = json_to_flight_profile(json)?;
                    client.set_flight_profile(profile).await.unwrap();
                }
                VLCommands::SetDeviceConfig(args) => {
                    let json = read_to_string(args.config_path).await?;
                    let device_config = json_to_device_config(json)?;
                    client.set_device_config(device_config).await.unwrap();
                }
                VLCommands::PullFlight(args) => {
                    pull_flight_data(&mut client, &args.save_folder).await.unwrap();
                },
                VLCommands::PullVacuumTest(args) => {
                    pull_vacuum_test(&mut client, &args.save_folder)
                        .await
                        .unwrap();
                }
                VLCommands::PullGroundTest(_) => todo!(),
                VLCommands::LS(args) => {
                    let files = list_files(&mut client, args.file_type).await.unwrap();
                    for file in files {
                        println!("{:?}", file);
                    }
                }
                VLCommands::PullFile(args) => {
                    pull_file(&mut client, args.file_id, args.host_path)
                        .await
                        .unwrap();
                }
                VLCommands::Reset => {
                    client.reset_device().await.unwrap();
                }
            }
        }
        ModeSelect::OZYS(OZYSCli { serial, command }) => {
            let mut serial = create_serial(serial)?;
            let mut client = sg_rpc::RpcClient::new(&mut serial, Delay);
            client.reset().await.map_err(|_| anyhow!("reset Error"))?;

            match command {
                SGCommands::PullData(args) => {
                    pull_ozys_data(&mut client, &args.save_folder)
                        .await
                        .unwrap();
                }
                SGCommands::ClearData => {
                    client.clear_data().await.unwrap();
                }
                SGCommands::RealTime(args) => {
                    client.set_adc_enable(true).await.unwrap();
                    loop {
                        let reading = client.get_real_time().await.unwrap();
                        if let Some(sample) = reading.sample {
                            if let Some(channel) = args.channel {
                                println!("{}", sample[channel - 1]);
                            } else {
                                println!("{} {} {} {}", sample[0], sample[1], sample[2], sample[3]);
                            }
                        }
                    }
                }
                SGCommands::LS(args) => {
                    let files = list_files(&mut client, args.file_type).await.unwrap();
                    for file in files {
                        println!("{:?}", file);
                    }
                }
                SGCommands::PullFile(args) => {
                    pull_file(&mut client, args.file_id, args.host_path)
                        .await
                        .unwrap();
                }
                SGCommands::Reset => {
                    client.reset_device().await.unwrap();
                }
            }
        }
        ModeSelect::GenLoraKey => {
            let key = gen_lora_key();
            println!("{}", format_lora_key(&key));
        }
    }

    println!("Done");
    Ok(())
}

fn print_telemetry_packet(packet: &TelemetryPacket, status: &RpcPacketStatus) {
    if let Some((lat, lon)) = packet.lat_lon() {
        println!("GPS: {}, {}", lat, lon);
    }
    println!(
        "{} ({:?}) Altitude: {}/{}, Speed: {}/{}, Temp: {}, Main Cont: {}, Drogue Cont: {}, H Armed: {}, S Armed: {}, Free space: {}MiB, RSSI: {}, SNR: {}{}{}",
        packet.timestamp() / 1000.0,
        packet.backup_flight_core_state(),
        packet.altitude(),
        packet.max_altitude(),
        packet.air_speed(),
        packet.max_air_speed(),
        packet.temperature(),
        packet.pyro_main_continuity(),
        packet.pyro_drogue_continuity(),
        packet.hardware_armed(),
        packet.software_armed(),
        packet.free_space() / 1024.0 / 1024.0,
        status.rssi,
        status.snr,
        if packet.drogue_deployed() { " Drogue Deployed" } else { "" },
        if packet.main_deployed() { " Main Deployed" } else { "" },
    );
}
