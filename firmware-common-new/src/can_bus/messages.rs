use crate::utils::FixedLenSerializable;
use ack::AckMessage;
use brightness_measurement::BrightnessMeasurementMessage;
use core::fmt::Debug;
use data_transfer::DataTransferMessage;
use payload_eps_control::PayloadEPSControlMessage;
use payload_self_test::PayloadEPSSelfTestMessage;

pub use amp_control::AmpControlMessage;
pub use amp_status::AmpStatusMessage;
pub use avionics_status::AvionicsStatusMessage;
pub use baro_measurement::BaroMeasurementMessage;
pub use icarus_status::IcarusStatusMessage;
pub use imu_measurement::IMUMeasurementMessage;
pub use node_status::NodeStatusMessage;
pub use reset::ResetMessage;
pub use tempurature_measurement::TempuratureMeasurementMessage;
pub use unix_time::UnixTimeMessage;
pub use payload_eps_status::PayloadEPSStatusMessage;

use super::id::CanBusExtendedId;

pub mod ack;
pub mod amp_control;
pub mod amp_status;
pub mod avionics_status;
pub mod baro_measurement;
pub mod brightness_measurement;
pub mod data_transfer;
pub mod icarus_status;
pub mod imu_measurement;
pub mod node_status;
pub mod payload_eps_control;
pub mod payload_self_test;
pub mod reset;
pub mod tempurature_measurement;
pub mod unix_time;
pub mod payload_eps_status;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[repr(C)]
pub enum CanBusMessageEnum {
    UnixTime(UnixTimeMessage),
    NodeStatus(NodeStatusMessage),
    Reset(ResetMessage),

    BaroMeasurement(BaroMeasurementMessage),
    IMUMeasurement(IMUMeasurementMessage),
    TempuratureMeasurement(TempuratureMeasurementMessage),
    BrightnessMeasurement(BrightnessMeasurementMessage),

    AmpStatus(AmpStatusMessage),
    AmpControl(AmpControlMessage),

    PayloadEPSStatus(PayloadEPSStatusMessage),
    PayloadEPSControl(PayloadEPSControlMessage),
    PayloadEPSSelfTest(PayloadEPSSelfTestMessage),

    AvionicsStatus(AvionicsStatusMessage),
    IcarusStatus(IcarusStatusMessage),

    DataTransfer(DataTransferMessage),
    Ack(AckMessage),
}

impl CanBusMessageEnum {
    pub fn priority(&self) -> u8 {
        match self {
            CanBusMessageEnum::UnixTime(m) => m.priority(),
            CanBusMessageEnum::NodeStatus(m) => m.priority(),
            CanBusMessageEnum::Reset(m) => m.priority(),
            CanBusMessageEnum::BaroMeasurement(m) => m.priority(),
            CanBusMessageEnum::IMUMeasurement(m) => m.priority(),
            CanBusMessageEnum::TempuratureMeasurement(m) => m.priority(),
            CanBusMessageEnum::BrightnessMeasurement(m) => m.priority(),
            CanBusMessageEnum::AmpStatus(m) => m.priority(),
            CanBusMessageEnum::AmpControl(m) => m.priority(),
            CanBusMessageEnum::PayloadEPSStatus(m) => m.priority(),
            CanBusMessageEnum::PayloadEPSControl(m) => m.priority(),
            CanBusMessageEnum::PayloadEPSSelfTest(m) => m.priority(),
            CanBusMessageEnum::AvionicsStatus(m) => m.priority(),
            CanBusMessageEnum::IcarusStatus(m) => m.priority(),
            CanBusMessageEnum::DataTransfer(m) => m.priority(),
            CanBusMessageEnum::Ack(m) => m.priority(),
        }
    }

    pub fn get_message_type(&self) -> u8 {
        match self {
            CanBusMessageEnum::UnixTime(_) => 0,
            CanBusMessageEnum::NodeStatus(_) => 1,
            CanBusMessageEnum::Reset(_) => 2,
            CanBusMessageEnum::BaroMeasurement(_) => 3,
            CanBusMessageEnum::IMUMeasurement(_) => 4,
            CanBusMessageEnum::TempuratureMeasurement(_) => 5,
            CanBusMessageEnum::BrightnessMeasurement(_) => 13,
            CanBusMessageEnum::AmpStatus(_) => 6,
            CanBusMessageEnum::AmpControl(_) => 7,
            CanBusMessageEnum::PayloadEPSStatus(_) => 8,
            CanBusMessageEnum::PayloadEPSControl(_) => 9,
            CanBusMessageEnum::PayloadEPSSelfTest(_) => 10,
            CanBusMessageEnum::AvionicsStatus(_) => 11,
            CanBusMessageEnum::IcarusStatus(_) => 12,
            CanBusMessageEnum::DataTransfer(_) => 14,
            CanBusMessageEnum::Ack(_) => 15,
        }
    }

    pub fn get_id(&self, node_type: u8, node_id: u16) -> CanBusExtendedId {
        CanBusExtendedId::new(self.priority(), self.get_message_type(), node_type, node_id)
    }

    pub fn serialize(self, buffer: &mut [u8]) -> usize {
        match self {
            CanBusMessageEnum::UnixTime(m) => m.serialize(buffer),
            CanBusMessageEnum::NodeStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::Reset(m) => m.serialize(buffer),
            CanBusMessageEnum::BaroMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::IMUMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::TempuratureMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::BrightnessMeasurement(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::AmpControl(m) => m.serialize(buffer),
            CanBusMessageEnum::PayloadEPSStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::PayloadEPSControl(m) => m.serialize(buffer),
            CanBusMessageEnum::PayloadEPSSelfTest(m) => m.serialize(buffer),
            CanBusMessageEnum::AvionicsStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::IcarusStatus(m) => m.serialize(buffer),
            CanBusMessageEnum::DataTransfer(m) => m.serialize(buffer),
            CanBusMessageEnum::Ack(m) => m.serialize(buffer),
        }
    }

    pub fn deserialize(message_type: u8, data: &[u8]) -> Option<Self> {
        match message_type {
            0 => UnixTimeMessage::deserialize(data).map(CanBusMessageEnum::UnixTime),
            1 => NodeStatusMessage::deserialize(data).map(CanBusMessageEnum::NodeStatus),
            2 => ResetMessage::deserialize(data).map(CanBusMessageEnum::Reset),
            3 => BaroMeasurementMessage::deserialize(data).map(CanBusMessageEnum::BaroMeasurement),
            4 => IMUMeasurementMessage::deserialize(data).map(CanBusMessageEnum::IMUMeasurement),
            5 => TempuratureMeasurementMessage::deserialize(data)
                .map(CanBusMessageEnum::TempuratureMeasurement),
            6 => AmpStatusMessage::deserialize(data).map(CanBusMessageEnum::AmpStatus),
            7 => AmpControlMessage::deserialize(data).map(CanBusMessageEnum::AmpControl),
            8 => PayloadEPSStatusMessage::deserialize(data).map(CanBusMessageEnum::PayloadEPSStatus),
            9 => PayloadEPSControlMessage::deserialize(data).map(CanBusMessageEnum::PayloadEPSControl),
            10 => PayloadEPSSelfTestMessage::deserialize(data).map(CanBusMessageEnum::PayloadEPSSelfTest),
            11 => AvionicsStatusMessage::deserialize(data).map(CanBusMessageEnum::AvionicsStatus),
            12 => IcarusStatusMessage::deserialize(data).map(CanBusMessageEnum::IcarusStatus),
            13 => BrightnessMeasurementMessage::deserialize(data)
                .map(CanBusMessageEnum::BrightnessMeasurement),
            14 => DataTransferMessage::deserialize(data).map(CanBusMessageEnum::DataTransfer),
            15 => AckMessage::deserialize(data).map(CanBusMessageEnum::Ack),
            _ => None,
        }
    }
}

pub trait CanBusMessage {
    /// 0-7, highest priority is 0
    fn priority(&self) -> u8;
}
