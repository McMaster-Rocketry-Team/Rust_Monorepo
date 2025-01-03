// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![feature(const_trait_impl)]
#![feature(generic_const_exprs)]
#![feature(let_chains)]
#![feature(try_blocks)]
#![feature(async_closure)]
#![feature(assert_matches)]
#![feature(never_type)]
#![feature(core_intrinsics)]
#![allow(async_fn_in_trait)]

mod fmt;
pub mod utils;
pub mod avionics;
pub mod common;
pub mod driver;
mod gcm;
mod ground_test_avionics;
pub mod strain_gauges;
pub mod vacuum_test;
mod vl_main;

pub use common::console::vl_rpc;
pub use common::console::sg_rpc;
pub use common::console::common_rpc_trait::CommonRPCTrait;
pub use common::vl_device_manager::VLDeviceManager;
pub use vl_main::vl_main;
pub use strain_gauges::high_prio::sg_high_prio_main;
pub use strain_gauges::mid_prio::sg_mid_prio_main;
pub use strain_gauges::low_prio::sg_low_prio_main;

