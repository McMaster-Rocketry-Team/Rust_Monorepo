use crate::utils::FixedLenSerializable;
use ack::AckMessage;
use airbrakes_control::AirBrakesControlMessage;
use amp_control::AmpControlMessage;
use amp_overwrite::AmpOverwriteMessage;
use amp_reset_output::AmpResetOutputMessage;
use amp_status::AmpStatusMessage;
use baro_measurement::BaroMeasurementMessage;
use brightness_measurement::BrightnessMeasurementMessage;
use core::fmt::Debug;
use custom_payload_status::CustomPayloadStatusMessage;
use data_transfer::DataTransferMessage;
use icarus_status::IcarusStatusMessage;
use imu_measurement::IMUMeasurementMessage;
use mag_measurement::MagMeasurementMessage;
use node_status::NodeStatusMessage;
use ozys_measurement::OzysMeasurementMessage;
use reset::ResetMessage;
use rocket_state::RocketStateMessage;
use static_assertions::const_assert;
use unix_time::UnixTimeMessage;
use vl_status::VLStatusMessage;

use super::id::{CanBusExtendedId, CanBusMessageTypeFlag, create_can_bus_message_type};

pub mod ack;
pub mod airbrakes_control;
pub mod amp_control;
pub mod amp_overwrite;
pub mod amp_reset_output;
pub mod amp_status;
pub mod baro_measurement;
pub mod brightness_measurement;
pub mod custom_payload_status;
pub mod data_transfer;
pub mod icarus_status;
pub mod imu_measurement;
pub mod mag_measurement;
pub mod node_status;
pub mod ozys_measurement;
pub mod reset;
pub mod rocket_state;
pub mod unix_time;
pub mod vl_status;

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
pub const MAG_MEASUREMENT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    4,
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
pub const OZYS_MEASUREMENT_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: true,
        is_control: false,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    5,
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
pub const CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: false,
        is_status: true,
        is_data: false,
        is_misc: false,
    },
    3,
);
pub const VL_STATUS_MESSAGE_TYPE: u8 = create_can_bus_message_type(
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
pub const AIRBRAKES_CONTROL_MESSAGE_TYPE: u8 = create_can_bus_message_type(
    CanBusMessageTypeFlag {
        is_measurement: false,
        is_control: true,
        is_status: false,
        is_data: false,
        is_misc: false,
    },
    5,
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

pub const MAX_CAN_MESSAGE_SIZE: usize = 64;

const_assert!(size_of::<CanBusMessageEnum>() <= MAX_CAN_MESSAGE_SIZE);

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, Ord, PartialOrd, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub enum CanBusMessageEnum {
    Reset(ResetMessage),
    // the usize does nothing here, it just makes firmware-common-ffi not complain about unsafe zero size type
    PreUnixTime(usize),
    UnixTime(UnixTimeMessage),
    NodeStatus(NodeStatusMessage),

    BaroMeasurement(BaroMeasurementMessage),
    IMUMeasurement(IMUMeasurementMessage),
    MagMeasurement(MagMeasurementMessage),
    BrightnessMeasurement(BrightnessMeasurementMessage),
    OzysMeasurement(OzysMeasurementMessage),

    AmpStatus(AmpStatusMessage),
    AmpOverwrite(AmpOverwriteMessage),
    AmpControl(AmpControlMessage),
    AmpResetOutput(AmpResetOutputMessage),

    CustomPayloadStatus(CustomPayloadStatusMessage),

    VLStatus(VLStatusMessage),
    RocketState(RocketStateMessage),
    IcarusStatus(IcarusStatusMessage),
    AirBrakesControl(AirBrakesControlMessage),

    DataTransfer(DataTransferMessage),
    Ack(AckMessage),
}

impl CanBusMessageEnum {
    pub fn priority(&self) -> u8 {
        match self {
            CanBusMessageEnum::UnixTime(m) => m.priority(),
            CanBusMessageEnum::PreUnixTime(_) => 1,
            CanBusMessageEnum::NodeStatus(m) => m.priority(),
            CanBusMessageEnum::Reset(m) => m.priority(),
            CanBusMessageEnum::BaroMeasurement(m) => m.priority(),
            CanBusMessageEnum::IMUMeasurement(m) => m.priority(),
            CanBusMessageEnum::MagMeasurement(m) => m.priority(),
            CanBusMessageEnum::BrightnessMeasurement(m) => m.priority(),
            CanBusMessageEnum::OzysMeasurement(m) => m.priority(),
            CanBusMessageEnum::AmpStatus(m) => m.priority(),
            CanBusMessageEnum::AmpOverwrite(m) => m.priority(),
            CanBusMessageEnum::AmpControl(m) => m.priority(),
            CanBusMessageEnum::AmpResetOutput(m) => m.priority(),
            CanBusMessageEnum::CustomPayloadStatus(m) => m.priority(),
            CanBusMessageEnum::VLStatus(m) => m.priority(),
            CanBusMessageEnum::RocketState(m) => m.priority(),
            CanBusMessageEnum::IcarusStatus(m) => m.priority(),
            CanBusMessageEnum::AirBrakesControl(m) => m.priority(),
            CanBusMessageEnum::DataTransfer(m) => m.priority(),
            CanBusMessageEnum::Ack(m) => m.priority(),
        }
    }

    pub fn get_message_type(&self) -> u8 {
        match self {
            CanBusMessageEnum::UnixTime(_) => UNIX_TIME_MESSAGE_TYPE,
            CanBusMessageEnum::PreUnixTime(_) => PRE_UNIX_TIME_MESSAGE_TYPE,
            CanBusMessageEnum::NodeStatus(_) => NODE_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::Reset(_) => RESET_MESSAGE_TYPE,
            CanBusMessageEnum::BaroMeasurement(_) => BARO_MEASUREMENT_MESSAGE_TYPE,
            CanBusMessageEnum::IMUMeasurement(_) => IMU_MEASUREMENT_MESSAGE_TYPE,
            CanBusMessageEnum::MagMeasurement(_) => MAG_MEASUREMENT_MESSAGE_TYPE,
            CanBusMessageEnum::BrightnessMeasurement(_) => BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE,
            CanBusMessageEnum::OzysMeasurement(_) => OZYS_MEASUREMENT_MESSAGE_TYPE,

            CanBusMessageEnum::AmpStatus(_) => AMP_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::AmpOverwrite(_) => AMP_OVERWRITE_MESSAGE_TYPE,
            CanBusMessageEnum::AmpControl(_) => AMP_CONTROL_MESSAGE_TYPE,
            CanBusMessageEnum::AmpResetOutput(_) => AMP_RESET_OUTPUT_MESSAGE_TYPE,
            CanBusMessageEnum::CustomPayloadStatus(_) => CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::VLStatus(_) => VL_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::RocketState(_) => ROCKET_STATE_MESSAGE_TYPE,
            CanBusMessageEnum::IcarusStatus(_) => ICARUS_STATUS_MESSAGE_TYPE,
            CanBusMessageEnum::AirBrakesControl(_) => AIRBRAKES_CONTROL_MESSAGE_TYPE,
            CanBusMessageEnum::DataTransfer(_) => DATA_TRANSFER_MESSAGE_TYPE,
            CanBusMessageEnum::Ack(_) => ACK_MESSAGE_TYPE,
        }
    }

    pub fn get_id(&self, node_type: u8, node_id: u16) -> CanBusExtendedId {
        CanBusExtendedId::new(self.priority(), self.get_message_type(), node_type, node_id)
    }

    pub fn serialized_len(message_type: u8) -> Option<usize> {
        match message_type {
            UNIX_TIME_MESSAGE_TYPE => Some(UnixTimeMessage::serialized_len()),
            PRE_UNIX_TIME_MESSAGE_TYPE => Some(0),
            NODE_STATUS_MESSAGE_TYPE => Some(NodeStatusMessage::serialized_len()),
            RESET_MESSAGE_TYPE => Some(ResetMessage::serialized_len()),
            BARO_MEASUREMENT_MESSAGE_TYPE => Some(BaroMeasurementMessage::serialized_len()),
            IMU_MEASUREMENT_MESSAGE_TYPE => Some(IMUMeasurementMessage::serialized_len()),
            MAG_MEASUREMENT_MESSAGE_TYPE => Some(MagMeasurementMessage::serialized_len()),
            BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE => {
                Some(BrightnessMeasurementMessage::serialized_len())
            }
            OZYS_MEASUREMENT_MESSAGE_TYPE => Some(OzysMeasurementMessage::serialized_len()),
            AMP_STATUS_MESSAGE_TYPE => Some(AmpStatusMessage::serialized_len()),
            AMP_OVERWRITE_MESSAGE_TYPE => Some(AmpOverwriteMessage::serialized_len()),
            AMP_CONTROL_MESSAGE_TYPE => Some(AmpControlMessage::serialized_len()),
            AMP_RESET_OUTPUT_MESSAGE_TYPE => Some(AmpResetOutputMessage::serialized_len()),
            CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE => {
                Some(CustomPayloadStatusMessage::serialized_len())
            }
            VL_STATUS_MESSAGE_TYPE => Some(VLStatusMessage::serialized_len()),
            ROCKET_STATE_MESSAGE_TYPE => Some(RocketStateMessage::serialized_len()),
            ICARUS_STATUS_MESSAGE_TYPE => Some(IcarusStatusMessage::serialized_len()),
            AIRBRAKES_CONTROL_MESSAGE_TYPE => Some(AirBrakesControlMessage::serialized_len()),
            DATA_TRANSFER_MESSAGE_TYPE => Some(DataTransferMessage::serialized_len()),
            ACK_MESSAGE_TYPE => Some(AckMessage::serialized_len()),
            _ => None,
        }
    }

    pub fn serialize(&self, buffer: &mut [u8]) -> usize {
        match self {
            CanBusMessageEnum::UnixTime(m) => m.serialize(buffer),
            CanBusMessageEnum::PreUnixTime(_) => 0,
            CanBusMessageEnum::NodeStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::Reset(m) => m.serialize(buffer),
            CanBusMessageEnum::BaroMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::IMUMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::MagMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::BrightnessMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::OzysMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpOverwrite(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpControl(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpResetOutput(m) => m.serialize(buffer),
            CanBusMessageEnum::CustomPayloadStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::VLStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::RocketState(m) => m.serialize(buffer),
            CanBusMessageEnum::IcarusStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::AirBrakesControl(m) => m.serialize(buffer),
            CanBusMessageEnum::DataTransfer(m) => m.serialize(buffer),
            CanBusMessageEnum::Ack(m) => m.serialize(buffer),
        }
    }

    pub fn deserialize(message_type: u8, data: &[u8]) -> Option<Self> {
        match message_type {
            RESET_MESSAGE_TYPE => ResetMessage::deserialize(data).map(CanBusMessageEnum::Reset),
            PRE_UNIX_TIME_MESSAGE_TYPE => Some(CanBusMessageEnum::PreUnixTime(0)),
            UNIX_TIME_MESSAGE_TYPE => {
                UnixTimeMessage::deserialize(data).map(CanBusMessageEnum::UnixTime)
            }
            NODE_STATUS_MESSAGE_TYPE => {
                NodeStatusMessage::deserialize(data).map(CanBusMessageEnum::NodeStatus)
            }

            BARO_MEASUREMENT_MESSAGE_TYPE => {
                BaroMeasurementMessage::deserialize(data).map(CanBusMessageEnum::BaroMeasurement)
            }
            IMU_MEASUREMENT_MESSAGE_TYPE => {
                IMUMeasurementMessage::deserialize(data).map(CanBusMessageEnum::IMUMeasurement)
            }
            MAG_MEASUREMENT_MESSAGE_TYPE => {
                MagMeasurementMessage::deserialize(data).map(CanBusMessageEnum::MagMeasurement)
            }
            BRIGHTNESS_MEASUREMENT_MESSAGE_TYPE => BrightnessMeasurementMessage::deserialize(data)
                .map(CanBusMessageEnum::BrightnessMeasurement),
            OZYS_MEASUREMENT_MESSAGE_TYPE => {
                OzysMeasurementMessage::deserialize(data).map(CanBusMessageEnum::OzysMeasurement)
            }

            AMP_STATUS_MESSAGE_TYPE => {
                AmpStatusMessage::deserialize(data).map(CanBusMessageEnum::AmpStatus)
            }
            AMP_OVERWRITE_MESSAGE_TYPE => {
                AmpOverwriteMessage::deserialize(data).map(CanBusMessageEnum::AmpOverwrite)
            }
            AMP_CONTROL_MESSAGE_TYPE => {
                AmpControlMessage::deserialize(data).map(CanBusMessageEnum::AmpControl)
            }
            AMP_RESET_OUTPUT_MESSAGE_TYPE => {
                AmpResetOutputMessage::deserialize(data).map(CanBusMessageEnum::AmpResetOutput)
            }

            CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE => CustomPayloadStatusMessage::deserialize(data)
                .map(CanBusMessageEnum::CustomPayloadStatus),

            VL_STATUS_MESSAGE_TYPE => {
                VLStatusMessage::deserialize(data).map(CanBusMessageEnum::VLStatus)
            }
            ICARUS_STATUS_MESSAGE_TYPE => {
                IcarusStatusMessage::deserialize(data).map(CanBusMessageEnum::IcarusStatus)
            }
            AIRBRAKES_CONTROL_MESSAGE_TYPE => {
                AirBrakesControlMessage::deserialize(data).map(CanBusMessageEnum::AirBrakesControl)
            }
            ROCKET_STATE_MESSAGE_TYPE => {
                RocketStateMessage::deserialize(data).map(CanBusMessageEnum::RocketState)
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

#[cfg(test)]
mod tests {
    use std::{fs, path::Path};

    use serde::{Deserialize, Serialize};

    use crate::can_bus::sender::CanBusMultiFrameEncoder;

    use super::*;

    pub fn test_serialize_deserialize(messages: Vec<CanBusMessageEnum>) {
        for message in messages {
            let mut buffer = [0u8; MAX_CAN_MESSAGE_SIZE];
            let message_type = message.get_message_type();
            let len = message.serialize(&mut buffer);

            let deserialized =
                CanBusMessageEnum::deserialize(message_type, &buffer[..len]).unwrap();

            assert_eq!(deserialized, message);
        }
    }

    #[derive(Serialize, Deserialize)]
    struct ReferenceData {
        message: CanBusMessageEnum,
        message_type: u8,
        serialized_data: Vec<u8>,
        frame_id: u32,
        encoded_data: Vec<Vec<u8>>,
    }

    pub fn create_reference_data(messages: Vec<CanBusMessageEnum>, name: &str) {
        let mut results = Vec::new();

        for message in messages {
            let mut buffer = [0u8; MAX_CAN_MESSAGE_SIZE];
            let message_type = message.get_message_type();
            let len = message.serialize(&mut buffer);
            let serialized_data = Vec::from(&buffer[..len]);

            let encoder = CanBusMultiFrameEncoder::new(&message);
            let encoded_data = encoder.map(|x| x.to_vec()).collect::<Vec<_>>();

            let frame_id = message.get_id(10, 20).into();

            results.push(ReferenceData {
                message,
                message_type,
                serialized_data,
                frame_id,
                encoded_data,
            });
        }

        let reference_data_string = serde_json::to_string_pretty(&results).unwrap();

        let path_str = format!("./can_bus_reference_data/{}.json", name);
        let file_path = Path::new(&path_str);
        if let Some(parent) = file_path.parent() {
            if !parent.exists() {
                std::fs::create_dir_all(parent).unwrap();
            }
        }
        fs::write(&file_path, reference_data_string).unwrap();
    }
}
