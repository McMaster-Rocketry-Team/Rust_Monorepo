use std::net::{IpAddr, Ipv4Addr};

#[tarpc::service]
pub trait AirBrakesControllerService {
    async fn update(reset: bool, acc: [f32; 3], gyro: [f32; 3], servo_angle: f32) -> f32;
}

pub const SERVER_ADDR: (IpAddr, u16) = (IpAddr::V4(Ipv4Addr::LOCALHOST), 15899);
