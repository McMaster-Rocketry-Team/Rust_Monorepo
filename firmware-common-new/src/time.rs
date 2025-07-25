use serde::{Serialize, Deserialize};

pub trait Clock {
    fn now_us(&self) -> u64;
}

pub trait TimestampType: Clone + Sized + Serialize + Deserialize<'static> {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UnixTimestamp;

impl TimestampType for UnixTimestamp {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BootTimestamp;

impl TimestampType for BootTimestamp {}
