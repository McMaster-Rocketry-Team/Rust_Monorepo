use ack::AckMessage;
use brightness_measurement::BrightnessMeasurementMessage;
use core::{
    any::{Any, TypeId},
    fmt::Debug,
};
use data_transfer::DataTransferMessage;
use payload_control::PayloadControlMessage;
use payload_self_test::PayloadSelfTestMessage;
use serde::{Deserialize, Serialize};

pub use amp_control::AmpControlMessage;
pub use amp_status::AmpStatusMessage;
pub use avionics_status::AvionicsStatusMessage;
pub use baro_measurement::BaroMeasurementMessage;
pub use icarus_status::IcarusStatusMessage;
pub use imu_measurement::IMUMeasurementMessage;
pub use node_status::NodeStatusMessage;
pub use payload_status::PayloadStatusMessage;
pub use reset::ResetMessage;
pub use tempurature_measurement::TempuratureMeasurementMessage;
pub use unix_time::UnixTimeMessage;

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
pub mod payload_control;
pub mod payload_self_test;
pub mod payload_status;
pub mod reset;
pub mod tempurature_measurement;
pub mod unix_time;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
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

    PayloadStatus(PayloadStatusMessage),
    PayloadControl(PayloadControlMessage),
    PayloadSelfTest(PayloadSelfTestMessage),

    AvionicsStatus(AvionicsStatusMessage),
    IcarusStatus(IcarusStatusMessage),

    DataTransfer(DataTransferMessage),
    Ack(AckMessage),
}

impl CanBusMessageEnum {
    pub(super) fn get_message_type<T: CanBusMessage>() -> Option<u8> {
        let t_id = TypeId::of::<T>();
        if t_id == TypeId::of::<UnixTimeMessage>() {
            Some(0)
        } else if t_id == TypeId::of::<NodeStatusMessage>() {
            Some(1)
        } else if t_id == TypeId::of::<ResetMessage>() {
            Some(2)
        } else if t_id == TypeId::of::<BaroMeasurementMessage>() {
            Some(3)
        } else if t_id == TypeId::of::<IMUMeasurementMessage>() {
            Some(4)
        } else if t_id == TypeId::of::<TempuratureMeasurementMessage>() {
            Some(5)
        } else if t_id == TypeId::of::<AmpStatusMessage>() {
            Some(6)
        } else if t_id == TypeId::of::<AmpControlMessage>() {
            Some(7)
        } else if t_id == TypeId::of::<PayloadStatusMessage>() {
            Some(8)
        } else if t_id == TypeId::of::<PayloadControlMessage>() {
            Some(9)
        } else if t_id == TypeId::of::<PayloadSelfTestMessage>() {
            Some(10)
        } else if t_id == TypeId::of::<AvionicsStatusMessage>() {
            Some(11)
        } else if t_id == TypeId::of::<IcarusStatusMessage>() {
            Some(12)
        } else if t_id == TypeId::of::<BrightnessMeasurementMessage>() {
            Some(13)
        } else if t_id == TypeId::of::<DataTransferMessage>() {
            Some(14)
        } else if t_id == TypeId::of::<AckMessage>() {
            Some(15)
        } else {
            None
        }
    }

    pub(super) fn deserialize(message_type: u8, data: &[u8]) -> Option<Self> {
        match message_type {
            0 => <UnixTimeMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::UnixTime),
            1 => <NodeStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::NodeStatus),
            2 => <ResetMessage as CanBusMessage>::deserialize(data).map(CanBusMessageEnum::Reset),
            3 => <BaroMeasurementMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::BaroMeasurement),
            4 => <IMUMeasurementMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::IMUMeasurement),
            5 => <TempuratureMeasurementMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::TempuratureMeasurement),
            6 => <AmpStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::AmpStatus),
            7 => <AmpControlMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::AmpControl),
            8 => <PayloadStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::PayloadStatus),
            9 => <PayloadControlMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::PayloadControl),
            10 => <PayloadSelfTestMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::PayloadSelfTest),
            11 => <AvionicsStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::AvionicsStatus),
            12 => <IcarusStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::IcarusStatus),
            13 => <BrightnessMeasurementMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::BrightnessMeasurement),
            14 => <DataTransferMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::DataTransfer),
            15 => <AckMessage as CanBusMessage>::deserialize(data).map(CanBusMessageEnum::Ack),
            _ => None,
        }
    }
}

pub trait CanBusMessage: Clone + Debug + Any {
    fn len() -> usize;

    /// 0-7, highest priority is 0
    fn priority(&self) -> u8;

    fn serialize(self, buffer: &mut [u8]);

    fn deserialize(data: &[u8]) -> Option<Self>;
}
