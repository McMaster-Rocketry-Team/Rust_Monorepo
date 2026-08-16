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

    // macOS exposes each USB CDC device twice (/dev/cu.* and /dev/tty.*); keep only the callout node.
    #[cfg(target_os = "macos")]
    let ground_station_serial_ports: Vec<SerialPortInfo> = ground_station_serial_ports
        .into_iter()
        .filter(|port| !port.port_name.contains("tty"))
        .collect();

    if ground_station_serial_ports.len() == 0 {
        bail!("No ground station connected")
    } else if ground_station_serial_ports.len() > 1 {
        println!(
            "More than one ground stations connected (current detected ground stations : {})",
            ground_station_serial_ports.len()
        );

        for port in ground_station_serial_ports {
            println!("port name {}", port.port_name);
        }
        bail!("");
    }
    Ok(ground_station_serial_ports[0].port_name.clone())
}
