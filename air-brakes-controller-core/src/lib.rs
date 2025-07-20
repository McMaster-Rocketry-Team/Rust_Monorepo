// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(any(test, feature = "std")), no_std)]

use java_bindgen::prelude::*;

mod fmt;
mod state_propagation;

#[cfg(test)]
mod tests;

static mut global_var: f32 = 0.0;

#[java_bindgen]
fn openrocket_post_step(a: f32) -> JResult<f32> {
    unsafe {
        global_var += a *2.0;
        Ok(global_var)
    }
}
