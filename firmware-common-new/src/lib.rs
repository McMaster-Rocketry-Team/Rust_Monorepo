// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "defmt", feature = "log"))]
compile_error!("Feature 'defmt' and 'log' are mutually exclusive and cannot be enabled together");

mod fmt;
mod gps;
mod sensor_reading;
mod time;

pub use gps::*;
pub use sensor_reading::*;
pub use time::*;
