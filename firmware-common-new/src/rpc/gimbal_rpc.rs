use crate::create_rpc;
use core::mem::MaybeUninit;

create_rpc! {
    gimbal
    0 arm_led | enabled: bool | -> ()
    1 status_led | enabled: bool | -> ()
    2 move_deg | tilt: f32, pan: f32 | -> ()
    3 measure_deg | | -> (tilt: f32, pan: f32)
}
