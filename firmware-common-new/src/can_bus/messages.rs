use crate::utils::FixedLenSerializable;
use ack::AckMessage;
#[cfg(not(feature = "bootloader"))]
use amp_control::AmpControlMessage;
#[cfg(not(feature = "bootloader"))]
use amp_reset_output::AmpResetOutputMessage;
#[cfg(not(feature = "bootloader"))]
use amp_overwrite::AmpOverwriteMessage;
#[cfg(not(feature = "bootloader"))]
use amp_status::AmpStatusMessage;
#[cfg(not(feature = "bootloader"))]
use avionics_status::AvionicsStatusMessage;
#[cfg(not(feature = "bootloader"))]
use baro_measurement::BaroMeasurementMessage;
#[cfg(not(feature = "bootloader"))]
use brightness_measurement::BrightnessMeasurementMessage;
use core::fmt::Debug;
use data_transfer::DataTransferMessage;
#[cfg(not(feature = "bootloader"))]
use icarus_status::IcarusStatusMessage;
#[cfg(not(feature = "bootloader"))]
use imu_measurement::IMUMeasurementMessage;
use node_status::NodeStatusMessage;
#[cfg(not(feature = "bootloader"))]
use payload_eps_output_overwrite::PayloadEPSOutputOverwriteMessage;
#[cfg(not(feature = "bootloader"))]
use payload_eps_status::PayloadEPSStatusMessage;
use reset::ResetMessage;
#[cfg(not(feature = "bootloader"))]
use rocket_state::RocketStateMessage;
#[cfg(not(feature = "bootloader"))]
use unix_time::UnixTimeMessage;

use super::id::{CanBusExtendedId, CanBusMessageTypeFlag, create_can_bus_message_type};

pub mod ack;
#[cfg(not(feature = "bootloader"))]
pub mod amp_control;
#[cfg(not(feature = "bootloader"))]
pub mod amp_reset_output;
#[cfg(not(feature = "bootloader"))]
pub mod amp_overwrite;
#[cfg(not(feature = "bootloader"))]
pub mod amp_status;
#[cfg(not(feature = "bootloader"))]
pub mod avionics_status;
#[cfg(not(feature = "bootloader"))]
pub mod baro_measurement;
#[cfg(not(feature = "bootloader"))]
pub mod brightness_measurement;
pub mod data_transfer;
#[cfg(not(feature = "bootloader"))]
pub mod icarus_status;
#[cfg(not(feature = "bootloader"))]
pub mod imu_measurement;
pub mod node_status;
#[cfg(not(feature = "bootloader"))]
pub mod payload_eps_output_overwrite;
#[cfg(not(feature = "bootloader"))]
pub mod payload_eps_status;
pub mod reset;
#[cfg(not(feature = "bootloader"))]
pub mod rocket_state;
#[cfg(not(feature = "bootloader"))]
pub mod unix_time;

pub const RESET_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    0,
);
pub const PRE_UNIX_TIME_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: true,
    },
    0,
);
pub const UNIX_TIME_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    7,
);
pub const NODE_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    0,
);
pub const BARO_MEASUREMENT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    0,
);
pub const IMU_MEASUREMENT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    1,
);
pub const BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    2,
);
pub const AMP_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    1,
);
pub const AMP_OVERWRITE_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    3,
);
pub const AMP_CONTROL_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    0,
);
pub const AMP_RESET_OUTPUT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    4,
);
pub const PAYLOAD_EPS_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    2,
);
pub const PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    1,
);
pub const AVIONICS_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    4,
);
pub const ROCKET_STATE_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    3,
);
pub const ICARUS_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    0,
);
pub const DATA_TRANSFER_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: false,
        is_data: true,
        is_misc: false,
    },
    0,
);
pub const ACK_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    2,
);
pub const LOG_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: true,
    },
    0,
);

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, Ord, PartialOrd, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub enum CanBusMessageEnum {
    Reset(ResetMessage),
    // the usize does nothing here, it just makes firmware-common-ffi not complain about unsafe zero size type
    #[cfg(not(feature = "bootloader"))]
    PreUnixTime(usize),
    #[cfg(not(feature = "bootloader"))]
    UnixTime(UnixTimeMessage),
    NodeStatus(NodeStatusMessage),

    #[cfg(not(feature = "bootloader"))]
    BaroMeasurement(BaroMeasurementMessage),
    #[cfg(not(feature = "bootloader"))]
    IMUMeasurement(IMUMeasurementMessage),
    #[cfg(not(feature = "bootloader"))]
    BrightnessMeasurement(BrightnessMeasurementMessage),

    #[cfg(not(feature = "bootloader"))]
    AmpStatus(AmpStatusMessage),
    #[cfg(not(feature = "bootloader"))]
    AmpOverwrite(AmpOverwriteMessage),
    #[cfg(not(feature = "bootloader"))]
    AmpControl(AmpControlMessage),
    #[cfg(not(feature = "bootloader"))]
    AmpResetOutput(AmpResetOutputMessage),

    #[cfg(not(feature = "bootloader"))]
    PayloadEPSStatus(PayloadEPSStatusMessage),
    #[cfg(not(feature = "bootloader"))]
    PayloadEPSOutputOverwrite(PayloadEPSOutputOverwriteMessage),

    #[cfg(not(feature = "bootloader"))]
    AvionicsStatus(AvionicsStatusMessage),
    #[cfg(not(feature = "bootloader"))]
    RocketState(RocketStateMessage),
    #[cfg(not(feature = "bootloader"))]
    IcarusStatus(IcarusStatusMessage),

    DataTransfer(DataTransferMessage),
    Ack(AckMessage),
}

impl CanBusMessageEnum {
    pub fn priority(&self) -> u8 {
        match self {
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::UnixTime(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PreUnixTime(_) => 1,
            CanBusMessageEnum::NodeStatus(m) => m.priority(),
            CanBusMessageEnum::Reset(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BaroMeasurement(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IMUMeasurement(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BrightnessMeasurement(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpStatus(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpOverwrite(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpControl(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpResetOutput(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSStatus(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSOutputOverwrite(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AvionicsStatus(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::RocketState(m) => m.priority(),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IcarusStatus(m) => m.priority(),
            CanBusMessageEnum::DataTransfer(m) => m.priority(),
            CanBusMessageEnum::Ack(m) => m.priority(),
        }
    }

    pub fn get_message_type(&self) -> u8 {
        match self {
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::UnixTime(_) => UNIX_TIME_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PreUnixTime(_) => PRE_UNIX_TIME_MESSAGE_TYPE,
            CanBusMessageEnum::NodeStatus(_) => NODE_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::Reset(_) => RESET_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BaroMeasurement(_) => BARO_MEASUREMENT_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IMUMeasurement(_) => IMU_MEASUREMENT_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BrightnessMeasurement(_) => BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpStatus(_) => AMP_STATUS_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpOverwrite(_) => AMP_OVERWRITE_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpControl(_) => AMP_CONTROL_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpResetOutput(_) => AMP_RESET_OUTPUT_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSStatus(_) => PAYLOAD_EPS_STATUS_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSOutputOverwrite(_) => {
                PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE
            }
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AvionicsStatus(_) => AVIONICS_STATUS_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::RocketState(_) => ROCKET_STATE_MESSAGE_TYPE,
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IcarusStatus(_) => ICARUS_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::DataTransfer(_) => DATA_TRANSFER_MESSAGE_TYPE,
            CanBusMessageEnum::Ack(_) => ACK_MESSAGE_TYPE,
        }
    }

    pub fn get_id(&self, node_type: u8, node_id: u16) -> CanBusExtendedId {
        CanBusExtendedId::new(self.priority(), self.get_message_type(), node_type, node_id)
    }

    pub fn serialized_len(message_type: u8) -> Option<usize> {
        match message_type {
            #[cfg(not(feature = "bootloader"))]
            UNIX_TIME_MESSAGE_TYPE => Some(UnixTimeMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            PRE_UNIX_TIME_MESSAGE_TYPE => Some(0),
            NODE_STATUS_MESSAGE_TYPE => Some(NodeStatusMessage::serialized_len()),
            RESET_MESSAGE_TYPE => Some(ResetMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            BARO_MEASUREMENT_MESSAGE_TYPE => Some(BaroMeasurementMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            IMU_MEASUREMENT_MESSAGE_TYPE => Some(IMUMeasurementMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE => {
                Some(BrightnessMeasurementMessage::serialized_len())
            }
            #[cfg(not(feature = "bootloader"))]
            AMP_STATUS_MESSAGE_TYPE => Some(AmpStatusMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            AMP_OVERWRITE_MESSAGE_TYPE => Some(AmpOverwriteMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            AMP_CONTROL_MESSAGE_TYPE => Some(AmpControlMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            AMP_RESET_OUTPUT_MESSAGE_TYPE => Some(AmpResetOutputMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            PAYLOAD_EPS_STATUS_MESSAGE_TYPE => Some(PayloadEPSStatusMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE => {
                Some(PayloadEPSOutputOverwriteMessage::serialized_len())
            }
            #[cfg(not(feature = "bootloader"))]
            AVIONICS_STATUS_MESSAGE_TYPE => Some(AvionicsStatusMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            ROCKET_STATE_MESSAGE_TYPE => Some(RocketStateMessage::serialized_len()),
            #[cfg(not(feature = "bootloader"))]
            ICARUS_STATUS_MESSAGE_TYPE => Some(IcarusStatusMessage::serialized_len()),
            DATA_TRANSFER_MESSAGE_TYPE => Some(DataTransferMessage::serialized_len()),
            ACK_MESSAGE_TYPE => Some(AckMessage::serialized_len()),
            _ => None,
        }
    }

    pub fn serialize(&self, buffer: &mut [u8]) -> usize {
        match self {
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::UnixTime(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PreUnixTime(_) => 0,
            CanBusMessageEnum::NodeStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::Reset(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BaroMeasurement(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IMUMeasurement(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::BrightnessMeasurement(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpStatus(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpOverwrite(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpControl(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AmpResetOutput(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSStatus(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::PayloadEPSOutputOverwrite(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::AvionicsStatus(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::RocketState(m) => m.serialize(buffer),
            #[cfg(not(feature = "bootloader"))]
            CanBusMessageEnum::IcarusStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::DataTransfer(m) => m.serialize(buffer),
            CanBusMessageEnum::Ack(m) => m.serialize(buffer),
        }
    }

    pub fn deserialize(message_type: u8, data: &[u8]) -> Option<Self> {
        match message_type {
            RESET_MESSAGE_TYPE => ResetMessage::deserialize(data).map(CanBusMessageEnum::Reset),
            #[cfg(not(feature = "bootloader"))]
            PRE_UNIX_TIME_MESSAGE_TYPE => Some(CanBusMessageEnum::PreUnixTime(0)),
            #[cfg(not(feature = "bootloader"))]
            UNIX_TIME_MESSAGE_TYPE => {
                UnixTimeMessage::deserialize(data).map(CanBusMessageEnum::UnixTime)
            }
            NODE_STATUS_MESSAGE_TYPE => {
                NodeStatusMessage::deserialize(data).map(CanBusMessageEnum::NodeStatus)
            }

            #[cfg(not(feature = "bootloader"))]
            BARO_MEASUREMENT_MESSAGE_TYPE => {
                BaroMeasurementMessage::deserialize(data).map(CanBusMessageEnum::BaroMeasurement)
            }
            #[cfg(not(feature = "bootloader"))]
            IMU_MEASUREMENT_MESSAGE_TYPE => {
                IMUMeasurementMessage::deserialize(data).map(CanBusMessageEnum::IMUMeasurement)
            }
            #[cfg(not(feature = "bootloader"))]
            BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE => BrightnessMeasurementMessage::deserialize(data)
                .map(CanBusMessageEnum::BrightnessMeasurement),

            #[cfg(not(feature = "bootloader"))]
            AMP_STATUS_MESSAGE_TYPE => {
                AmpStatusMessage::deserialize(data).map(CanBusMessageEnum::AmpStatus)
            }
            #[cfg(not(feature = "bootloader"))]
            AMP_OVERWRITE_MESSAGE_TYPE => {
                AmpOverwriteMessage::deserialize(data).map(CanBusMessageEnum::AmpOverwrite)
            }
            #[cfg(not(feature = "bootloader"))]
            AMP_CONTROL_MESSAGE_TYPE => {
                AmpControlMessage::deserialize(data).map(CanBusMessageEnum::AmpControl)
            }
            #[cfg(not(feature = "bootloader"))]
            AMP_RESET_OUTPUT_MESSAGE_TYPE => {
                AmpResetOutputMessage::deserialize(data).map(CanBusMessageEnum::AmpResetOutput)
            }

            #[cfg(not(feature = "bootloader"))]
            PAYLOAD_EPS_STATUS_MESSAGE_TYPE => {
                PayloadEPSStatusMessage::deserialize(data).map(CanBusMessageEnum::PayloadEPSStatus)
            }
            #[cfg(not(feature = "bootloader"))]
            PAYLOAD_EPS_OUTPUT_OVERWRITE_MESSAGE_TYPE => {
                PayloadEPSOutputOverwriteMessage::deserialize(data)
                    .map(CanBusMessageEnum::PayloadEPSOutputOverwrite)
            }

            #[cfg(not(feature = "bootloader"))]
            AVIONICS_STATUS_MESSAGE_TYPE => {
                AvionicsStatusMessage::deserialize(data).map(CanBusMessageEnum::AvionicsStatus)
            }
            #[cfg(not(feature = "bootloader"))]
            ICARUS_STATUS_MESSAGE_TYPE => {
                IcarusStatusMessage::deserialize(data).map(CanBusMessageEnum::IcarusStatus)
            }

            DATA_TRANSFER_MESSAGE_TYPE => {
                DataTransferMessage::deserialize(data).map(CanBusMessageEnum::DataTransfer)
            }
            ACK_MESSAGE_TYPE => AckMessage::deserialize(data).map(CanBusMessageEnum::Ack),
            _ => None,
        }
    }
}

pub trait CanBusMessage {
    /// 0-7, highest priority is 0
    fn priority(&self) -> u8;
}
