// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![recursion_limit = "256"]

mod fmt;
pub(crate) mod utils;

#[cfg(test)]
mod tests;

pub mod flight_data_record;
pub mod flight_storage;

pub mod can_bus;
pub(crate) mod fixed_point;
pub mod gps;
pub mod readings;
pub mod sensor_reading;
pub mod signal_with_ack;
pub mod time;
pub mod variance;
pub mod vlp;
pub mod heatshrink;
pub mod rpc;