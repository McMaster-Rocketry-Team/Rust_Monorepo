use crate::create_rpc;

create_rpc! {
    gimbal
    0 gimbal_info | | -> (tilt_range_deg: (f32, f32), pan_range_deg: (f32, f32), focal_length_range_mm: (f32, f32))
    1 set_arm_led | enabled: bool | -> ()
    2 set_status_led | enabled: bool | -> ()
    3 set_deg | tilt_deg: f32, pan_deg: f32 | -> ()
    4 get_deg | | -> (tilt_deg: f32, pan_deg: f32)
    5 set_focal_length_mm | focal_length_mm: f32 | -> ()
    6 get_focal_length_mm | | -> (focal_length_mm: f32)
    7 get_gps_data | | -> (coordinates: Option<(f64, f64)>, timestamp_ms: Option<u64>)
}
