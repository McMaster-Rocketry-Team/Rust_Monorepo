use amp_control::AmpControlMessage;
use core::{
    any::{Any, TypeId},
    fmt::Debug,
};
use packed_struct::PackedStructSlice;
use serde::{Deserialize, Serialize};
use unix_time::UnixTimeMessage;

mod amp_control;
mod unix_time;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CanBusMessageEnum {
    UnixTime(UnixTimeMessage),
    AmpControl(AmpControlMessage),
}

impl CanBusMessageEnum {
    pub(super) fn get_message_len(message_type: u8) -> Option<usize> {
        match message_type {
            0 => Some(UnixTimeMessage::len()),
            1 => Some(AmpControlMessage::len()),
            _ => None,
        }
    }

    pub(super) fn get_message_type<T: CanBusMessage>() -> Option<u8> {
        let t_id = TypeId::of::<T>();
        if t_id == TypeId::of::<UnixTimeMessage>() {
            Some(0)
        } else if t_id == TypeId::of::<AmpControlMessage>() {
            Some(1)
        } else {
            None
        }
    }

    pub(super) fn deserialize(message_type: u8, data: &[u8]) -> Option<Self> {
        match message_type {
            0 => UnixTimeMessage::unpack_from_slice(data)
                .ok()
                .map(CanBusMessageEnum::UnixTime),
            1 => AmpControlMessage::unpack_from_slice(data)
                .ok()
                .map(CanBusMessageEnum::AmpControl),
            _ => None,
        }
    }
}

pub trait CanBusMessage: Clone + Debug + Any {
    fn len() -> usize;

    /// 0-7, highest priority is 0
    fn priority() -> u8;

    fn serialize(self, buffer: &mut [u8]);

    fn deserialize(data: &[u8]) -> Option<Self>;
}
