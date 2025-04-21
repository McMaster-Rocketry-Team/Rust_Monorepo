// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![feature(let_chains)]

#[cfg(all(feature = "defmt", feature = "log"))]
compile_error!("Feature 'defmt' and 'log' are mutually exclusive and cannot be enabled together");

mod fmt;

pub(crate) mod fixed_point;
pub mod can_bus;
pub mod gps;
pub mod sensor_reading;
pub mod time;
pub mod vlp;
