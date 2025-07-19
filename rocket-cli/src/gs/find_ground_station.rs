use anyhow::{Result, bail};
use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo, available_ports};

pub async fn find_ground_station() -> Result<String> {
    let ground_station_serial_ports = available_ports()
        .unwrap()
        .into_iter()
        .filter(|port| {
            matches!(
                port.port_type,
                SerialPortType::UsbPort(UsbPortInfo {
                    vid: 0x120a,
                    pid: 0x0005,
                    ..
                })
            )
        })
        .collect::<Vec<SerialPortInfo>>();

    if ground_station_serial_ports.len() == 0 {
        bail!("No ground station connected")
    } else if ground_station_serial_ports.len() > 1 {
        bail!("More than one ground stations connected")
    }
    Ok(ground_station_serial_ports[0].port_name.clone())
}
