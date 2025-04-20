use amp_control::AmpControlMessage;
use amp_status::AmpStatusMessage;
use avionics_status::AvionicsStatusMessage;
use baro_measurement::BaroMeasurementMessage;
use bulkhead_status::BulkheadStatusMessage;
use core::{
    any::{Any, TypeId},
    fmt::Debug,
};
use icarus_status::IcarusStatusMessage;
use imu_measurement::IMUMeasurementMessage;
use node_status::NodeStatusMessage;
use payload_activation_status::PayloadActivationStatusMessage;
use reset::ResetMessage;
use serde::{Deserialize, Serialize};
use tempurature_measurement::TempuratureMeasurementMessage;
use unix_time::UnixTimeMessage;

mod amp_control;
mod amp_status;
mod avionics_status;
mod baro_measurement;
mod bulkhead_status;
mod icarus_status;
mod imu_measurement;
mod node_status;
mod payload_activation_status;
mod reset;
mod tempurature_measurement;
mod unix_time;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CanBusMessageEnum {
    UnixTime(UnixTimeMessage),
    NodeStatus(NodeStatusMessage),
    Reset(ResetMessage),

    BaroMeasurement(BaroMeasurementMessage),
    IMUMeasurement(IMUMeasurementMessage),
    TempuratureMeasurement(TempuratureMeasurementMessage),

    AmpStatus(AmpStatusMessage),
    AmpControl(AmpControlMessage),

    AvionicsStatus(AvionicsStatusMessage),
    IcarusStatus(IcarusStatusMessage),
    BulkheadStatus(BulkheadStatusMessage),
    PayloadActivationStatus(PayloadActivationStatusMessage),
}

impl CanBusMessageEnum {
    pub(super) fn get_message_len(message_type: u8) -> Option<usize> {
        match message_type {
            0 => Some(UnixTimeMessage::len()),
            1 => Some(NodeStatusMessage::len()),
            2 => Some(ResetMessage::len()),

            3 => Some(BaroMeasurementMessage::len()),
            4 => Some(IMUMeasurementMessage::len()),
            5 => Some(TempuratureMeasurementMessage::len()),

            6 => Some(AmpStatusMessage::len()),
            7 => Some(AmpControlMessage::len()),

            8 => Some(AvionicsStatusMessage::len()),
            9 => Some(IcarusStatusMessage::len()),
            10 => Some(BulkheadStatusMessage::len()),
            11 => Some(PayloadActivationStatusMessage::len()),
            _ => None,
        }
    }

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
        } else if t_id == TypeId::of::<AvionicsStatusMessage>() {
            Some(8)
        } else if t_id == TypeId::of::<IcarusStatusMessage>() {
            Some(9)
        } else if t_id == TypeId::of::<BulkheadStatusMessage>() {
            Some(10)
        } else if t_id == TypeId::of::<PayloadActivationStatusMessage>() {
            Some(11)
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
            8 => <AvionicsStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::AvionicsStatus),
            9 => <IcarusStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::IcarusStatus),
            10 => <BulkheadStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::BulkheadStatus),
            11 => <PayloadActivationStatusMessage as CanBusMessage>::deserialize(data)
                .map(CanBusMessageEnum::PayloadActivationStatus),
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
