// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "defmt")]
use defmt::info;

mod fmt;
mod gps;
mod sensor_reading;
mod time;

pub use gps::*;
pub use sensor_reading::*;
pub use time::*;
