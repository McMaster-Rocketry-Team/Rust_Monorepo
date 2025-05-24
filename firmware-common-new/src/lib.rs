// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "wasm")), no_std)]
#![feature(let_chains)]
#![feature(assert_matches)]
#![feature(slice_as_array)]

mod fmt;
pub(crate) mod utils;

#[cfg(test)]
mod tests;

pub mod bootloader;
pub mod can_bus;
pub(crate) mod fixed_point;
pub mod gps;
pub mod readings;
pub mod sensor_reading;
pub mod signal_with_ack;
pub mod time;
pub mod variance;
#[cfg(not(feature = "bootloader"))]
pub mod vlp;
pub mod heatshrink;