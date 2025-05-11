use crate::{
    sensor_reading::SensorReading,
    time::{BootTimestamp, Clock},
};
use chrono::{TimeZone as _, Utc};
use embassy_futures::yield_now;
use embedded_io_async::Read;
use heapless::String;
use nmea::Nmea;
use serde::{Deserialize, Serialize};
use crate::fmt::Debug2DefmtWrapper;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GPSData {
    pub timestamp: Option<i64>, // in seconds
    pub lat_lon: Option<(f64, f64)>,
    pub altitude: Option<f32>,
    pub num_of_fix_satellites: u8,
    pub hdop: Option<f32>,
    pub vdop: Option<f32>,
    pub pdop: Option<f32>,
}

impl From<&Nmea> for GPSData {
    fn from(nmea: &Nmea) -> Self {
        let lat_lon: Option<(f64, f64)> =
            if let (Some(lat), Some(lon)) = (nmea.latitude, nmea.longitude) {
                Some((lat, lon))
            } else {
                None
            };

        let timestamp = if let (Some(date), Some(time)) = (nmea.fix_date, nmea.fix_time) {
            let datetime = date.and_time(time);
            let datetime = Utc.from_utc_datetime(&datetime);
            Some(datetime.timestamp())
        } else {
            None
        };

        Self {
            timestamp,
            lat_lon,
            altitude: nmea.altitude,
            num_of_fix_satellites: nmea.num_of_fix_satellites.unwrap_or(0) as u8,
            hdop: nmea.hdop,
            vdop: nmea.vdop,
            pdop: nmea.pdop,
        }
    }
}

pub async fn run_gps_uart_receiver(
    rx: &mut impl Read,
    clock: impl Clock,
    mut on_receive: impl FnMut(SensorReading<BootTimestamp, GPSData>),
) {
    let mut buffer = [0; 84];
    let mut sentence = String::<84>::new();
    let mut nmea = Nmea::default();
    loop {
        match rx.read(&mut buffer).await {
            Ok(length) => {
                for i in 0..length {
                    if buffer[i] == b'$' {
                        sentence.clear();
                    }
                    sentence.push(buffer[i] as char).ok();

                    if buffer[i] == 10u8 || sentence.len() == 84 {
                        // log_info!("NMEA sentence: {}", sentence);
                        nmea.parse(sentence.as_str()).ok();

                        on_receive(SensorReading::new(clock.now_us(), (&nmea).into()));

                        sentence.clear();
                        for j in (i + 1)..length {
                            sentence.push(buffer[j] as char).ok();
                        }
                    }
                }
            }
            Err(e) => {
                log_error!("Error reading from UART: {:?}", Debug2DefmtWrapper(e));
                yield_now().await;
            }
        }
    }
}
