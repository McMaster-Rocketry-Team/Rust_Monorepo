use core::marker::PhantomData;

use crate::time::TimestampType;
use serde::{Serialize, Deserialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorReading<T: TimestampType, D> {
    _phantom_timestamp: PhantomData<T>,
    pub timestamp_us: u64,
    pub data: D,
}

impl<T: TimestampType, D> SensorReading<T, D> {
    pub fn new(timestamp_us: u64, data: D) -> Self {
        SensorReading {
            _phantom_timestamp: PhantomData,
            timestamp_us,
            data,
        }
    }

    pub fn timestamp_s(&self) -> f64 {
        self.timestamp_us as f64 / 1_000_000.0
    }
}