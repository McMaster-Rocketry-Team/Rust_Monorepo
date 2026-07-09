use std::fmt::Display;

use clap::Parser;
use clap::Subcommand;
use serde::Deserialize;
use serde::Serialize;

use crate::testing::decode_bluetooth_chunk::DecodeBluetoothChunkArgs;
use firmware_common_new::can_bus::node_types::*;

#[derive(Parser, Debug)]
#[command(name = "Rocket CLI")]
#[command(bin_name = "rocket-cli")]
pub struct Cli {
    #[clap(subcommand)]
    pub mode: ModeSelect,
}

#[derive(Subcommand, Debug)]
pub enum ModeSelect {
    #[command(about = "download firmware to stm32 via probe or ota")]
    Download(DownloadCli),

    #[command(about = "download firmware to esp32 via probe or ota")]
    DownloadEsp(DownloadEspCli),

    #[command(about = "attach to target via probe or ota")]
    Attach(AttachCli),

    #[command(about = "connect to ground station")]
    GroundStation,

    #[command(
        about = "non-interactive ground station: stream downlink JSON to stdout, read commands from stdin"
    )]
    Control(ControlArgs),

    #[command(about = "non-interactive ground station: send a single uplink and exit")]
    SendUplink(SendUplinkArgs),

    #[command(about = "generate vlp key")]
    GenVlpKey(GenVlpKeyCli),

    #[command(about = "generate private and public keys for ota")]
    GenOtaKey(GenOtaKeyCli),

    #[command(about = "show SD flight log summary from a connected VLF5")]
    ListFlightLog,

    #[command(about = "download SD flight log from a connected VLF5 to CSV")]
    DownloadFlightLog(DownloadFlightLogArgs),

    #[command(about = "erase the SD flight log on a connected VLF5")]
    ClearFlightLog,

    #[clap(subcommand)]
    #[command(about = "functions used for testing")]
    Testing(TestingModeSelect),
}

#[derive(Parser, Debug)]
pub struct DownloadFlightLogArgs {
    #[arg(default_value = "flight_log.csv")]
    pub output: String,
}

#[derive(Parser, Debug)]
pub struct ControlArgs {
    #[arg(
        long,
        help = "talk to a HIL VLF5 directly over USB instead of the GCM/LoRa (no ack)"
    )]
    pub usb: bool,
    #[arg(long, help = "LoRa frequency in Hz (default: ground-station.toml)")]
    pub frequency: Option<u32>,
    #[arg(long, help = "LoRa TX power in dBm (default: ground-station.toml)")]
    pub power: Option<i32>,
    #[arg(long, help = "base64 32-byte VLP key (default: ground-station.toml)")]
    pub vlp_key: Option<String>,
}

#[derive(Parser, Debug)]
pub struct SendUplinkArgs {
    #[arg(
        long,
        help = "talk to a HIL VLF5 directly over USB instead of the GCM/LoRa (no ack)"
    )]
    pub usb: bool,
    #[arg(long, help = "LoRa frequency in Hz (default: ground-station.toml)")]
    pub frequency: Option<u32>,
    #[arg(long, help = "LoRa TX power in dBm (default: ground-station.toml)")]
    pub power: Option<i32>,
    #[arg(long, help = "base64 32-byte VLP key (default: ground-station.toml)")]
    pub vlp_key: Option<String>,
    #[arg(
        trailing_var_arg = true,
        allow_hyphen_values = true,
        required = true,
        help = "uplink command, e.g. `arm`, `mode armed`, `target-apogee 3000`, `fire-pyro drogue`, `reset all`"
    )]
    pub command: Vec<String>,
}

#[derive(Parser, Debug)]
pub struct DownloadCli {
    pub chip: String,
    pub secret_path: std::path::PathBuf,
    pub node_type: NodeTypeEnum,
    pub firmware_elf_path: std::path::PathBuf,
}

#[derive(Parser, Debug)]
pub struct DownloadEspCli {
    pub secret_path: std::path::PathBuf,
    pub node_type: NodeTypeEnum,
    pub firmware_bin_path: std::path::PathBuf,
}

#[derive(Parser, Debug)]
pub struct AttachCli {
    #[arg(long)]
    pub chip: Option<String>,
    #[arg(long, help = "firmware elf path")]
    pub elf: Option<std::path::PathBuf>,
}

#[derive(Parser, Debug)]
pub struct GenVlpKeyCli {
    pub key_path: std::path::PathBuf,
}

#[derive(Parser, Debug)]
pub struct GenOtaKeyCli {
    pub key_directory: std::path::PathBuf,
}

#[derive(Subcommand, Debug)]
pub enum TestingModeSelect {
    DecodeBluetoothChunk(DecodeBluetoothChunkArgs),
    MockConnection,
    MockGroundStation,
    SendVLPTelemetry(SendVLPTelemetryArgs),
}

#[derive(Parser, Debug)]
pub struct SendVLPTelemetryArgs {
    pub frequency: u32,
    pub longitude: f64,
    pub latitude: f64,
    pub altitude_agl: Option<f32>,
}

#[derive(
    clap::ValueEnum,
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    Serialize,
    Deserialize,
    PartialOrd,
    Ord,
)]
pub enum NodeTypeEnum {
    VoidLake,
    AMP,
    ICARUS,
    PayloadActivation,
    RocketWifi,
    OZYS,
    Bulkhead,
    EPS1,
    EPS2,
    AeroRust,
    Other,
}

impl Display for NodeTypeEnum {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NodeTypeEnum::VoidLake => write!(f, "Void Lake"),
            NodeTypeEnum::AMP => write!(f, "AMP"),
            NodeTypeEnum::ICARUS => write!(f, "ICARUS"),
            NodeTypeEnum::PayloadActivation => write!(f, "Payload Activation"),
            NodeTypeEnum::RocketWifi => write!(f, "Rocket WiFi"),
            NodeTypeEnum::OZYS => write!(f, "OZYS"),
            NodeTypeEnum::Bulkhead => write!(f, "Bulkhead"),
            NodeTypeEnum::EPS1 => write!(f, "EPS1"),
            NodeTypeEnum::EPS2 => write!(f, "EPS2"),
            NodeTypeEnum::AeroRust => write!(f, "AeroRust"),
            NodeTypeEnum::Other => write!(f, "Other"),
        }
    }
}

impl From<u8> for NodeTypeEnum {
    fn from(value: u8) -> Self {
        match value {
            VOID_LAKE_NODE_TYPE => Self::VoidLake,
            AMP_NODE_TYPE => Self::AMP,
            ICARUS_NODE_TYPE => Self::ICARUS,
            PAYLOAD_ACTIVATION_NODE_TYPE => Self::PayloadActivation,
            PAYLOAD_ROCKET_WIFI_NODE_TYPE => Self::RocketWifi,
            OZYS_NODE_TYPE => Self::OZYS,
            BULKHEAD_NODE_TYPE => Self::Bulkhead,
            PAYLOAD_EPS1_NODE_TYPE => Self::EPS1,
            PAYLOAD_EPS2_NODE_TYPE => Self::EPS2,
            AERO_RUST_NODE_TYPE => Self::AeroRust,
            _ => Self::Other,
        }
    }
}

impl Into<u8> for NodeTypeEnum {
    fn into(self) -> u8 {
        match self {
            NodeTypeEnum::VoidLake => VOID_LAKE_NODE_TYPE,
            NodeTypeEnum::AMP => AMP_NODE_TYPE,
            NodeTypeEnum::ICARUS => ICARUS_NODE_TYPE,
            NodeTypeEnum::PayloadActivation => PAYLOAD_ACTIVATION_NODE_TYPE,
            NodeTypeEnum::RocketWifi => PAYLOAD_ROCKET_WIFI_NODE_TYPE,
            NodeTypeEnum::OZYS => OZYS_NODE_TYPE,
            NodeTypeEnum::Bulkhead => BULKHEAD_NODE_TYPE,
            NodeTypeEnum::EPS1 => PAYLOAD_EPS1_NODE_TYPE,
            NodeTypeEnum::EPS2 => PAYLOAD_EPS2_NODE_TYPE,
            NodeTypeEnum::AeroRust => AERO_RUST_NODE_TYPE,
            NodeTypeEnum::Other => unimplemented!(),
        }
    }
}
