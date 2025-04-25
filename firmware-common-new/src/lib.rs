// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![feature(let_chains)]
#![feature(assert_matches)]

mod fmt;
pub(crate) mod utils;

pub(crate) mod fixed_point;
pub mod can_bus;
pub mod gps;
pub mod sensor_reading;
pub mod time;
pub mod vlp;
