use serde::{Serialize, Deserialize};

pub trait Clock: Clone {
    fn now_ms(&self) -> f64;
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
