// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "defmt")]
use defmt::info;

use embassy_futures::select::{Either, select};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorType, Read, ReadExactError, Write};
use packed_struct::prelude::*;
mod fmt;

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct ServoStatus {
    #[packed_field(bits = "0")]
    pub is_under_over_voltage: bool,
    #[packed_field(bits = "1")]
    pub is_over_current: bool,
    #[packed_field(bits = "2")]
    pub is_over_temperature: bool,
    #[packed_field(bits = "3")]
    pub is_over_loaded: bool,
    #[packed_field(bits = "4")]
    pub is_hardware_failure: bool,
    #[packed_field(bits = "5")]
    pub last_command_corrupted: bool,
    #[packed_field(bits = "6")]
    pub last_command_failed: bool,
    #[packed_field(bits = "7")]
    pub is_turning: bool,
}

impl ServoStatus {
    pub fn is_ok(&self) -> bool {
        !self.is_under_over_voltage
            && !self.is_over_current
            && !self.is_over_temperature
            && !self.is_over_loaded
            && !self.is_hardware_failure
            && !self.last_command_corrupted
            && !self.last_command_failed
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Measurements {
    /// in degrees
    pub angle: f32,

    /// in degrees per second
    pub angular_velocity: i16,

    /// in A
    pub current: f32,

    /// in 0-1
    pub pwm_duty_cycle: f32,

    /// in C
    pub temperature: i16,
}

impl From<MeasurementsRaw> for Measurements {
    fn from(value: MeasurementsRaw) -> Self {
        Self {
            angle: value.angle as f32 / 10.0,
            angular_velocity: value.angular_velocity,
            current: value.current as f32 / 1000.0,
            pwm_duty_cycle: value.pwm_duty_cycle as f32 / 1000.0,
            temperature: value.temperature,
        }
    }
}

#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(endian = "lsb")]
struct MeasurementsRaw {
    angle: i32,
    angular_velocity: i16,
    current: i16,
    pwm_duty_cycle: i16,
    temperature: i16,
}

pub struct DSPowerServo<S, D>
where
    S: Read + Write,
    D: DelayNs,
{
    serial: S,
    // Maximum amount of buffer needed for a single response
    buffer: [u8; 19],
    delay: D,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum DSPowerServoError<S: ErrorType> {
    SerialError(S::Error),
    UnexpectedEof,
    ChecksumError { expected: u8, actual: u8 },
    ReadTimeout,
}

impl<S: ErrorType> DSPowerServoError<S> {
    fn from_read_exact_error(error: ReadExactError<S::Error>) -> Self {
        match error {
            ReadExactError::Other(e) => DSPowerServoError::SerialError(e),
            ReadExactError::UnexpectedEof => DSPowerServoError::UnexpectedEof,
        }
    }
}

#[repr(u8)]
pub enum Command {
    GetStatus = 1,
    ReadRegister = 2,
    WriteRegister = 3,
}

impl<S, D> DSPowerServo<S, D>
where
    S: Read + Write,
    D: DelayNs,
{
    pub fn new(serial: S, delay: D) -> Self {
        Self {
            serial,
            buffer: [0u8; 19],
            delay,
        }
    }

    // FIXME doesn't seem to work
    pub async fn reset(&mut self) -> Result<(), DSPowerServoError<S>> {
        log_info!("Resetting servo");

        self.write_register(0x02, &[0xE1, 0xE2, 0xE3, 0xE4]).await?;
        self.delay.delay_ms(100).await;

        let read_all_fut = async {
            let result: Result<(), DSPowerServoError<S>> = loop {
                let len = self
                    .serial
                    .read(&mut self.buffer)
                    .await
                    .map_err(DSPowerServoError::SerialError)?;

                log_info!("read {} bytes", len);
                if len == 0 {
                    break Ok(());
                }
            };
            result
        };

        let timeout_fut = self.delay.delay_ms(500);

        match select(timeout_fut, read_all_fut).await {
            Either::First(_) => {}
            Either::Second(result) => result?,
        }

        Ok(())
    }

    pub async fn init(&mut self, enable_protections: bool) -> Result<(), DSPowerServoError<S>> {
        log_info!("Initializing servo");
        // Save settings to flash
        self.overwrite_register_if_different(0x04, &[0]).await?;

        // break on by default
        self.overwrite_register_if_different(0x11, &[0b0000_0011])
            .await?;

        // Wait 200us before sending responses
        self.overwrite_register_if_different(0x12, &(0u16.to_le_bytes()))
            .await?;

        // Max duty cycle: 100%
        self.overwrite_register_if_different(0x13, &(1000u16.to_le_bytes()))
            .await?;

        // Max current: 5A
        self.overwrite_register_if_different(0x14, &(5000u16.to_le_bytes()))
            .await?;

        // Max speed: 10000 deg/s
        self.overwrite_register_if_different(0x15, &(10000u16.to_le_bytes()))
            .await?;

        // Max duty cycle while not moving: 100%
        self.overwrite_register_if_different(0x1C, &(1000u16.to_le_bytes()))
            .await?;

        if enable_protections {
            self.overwrite_register_if_different(0x32, &[0b0000_1111])
                .await?;
        } else {
            self.overwrite_register_if_different(0x32, &[0]).await?;
        }

        Ok(())
    }

    pub async fn reduce_torque(&mut self) -> Result<(), DSPowerServoError<S>> {
        // Don't save settings to flash
        self.overwrite_register_if_different(0x04, &[1]).await?;

        // Max duty cycle: 20%
        self.overwrite_register_if_different(0x13, &(200u16.to_le_bytes()))
            .await?;

        // Max duty cycle while not moving: 20%
        self.overwrite_register_if_different(0x1C, &(200u16.to_le_bytes()))
            .await?;

        Ok(())
    }

    pub async fn restore_torque(&mut self) -> Result<(), DSPowerServoError<S>> {
        // Max duty cycle: 100%
        self.overwrite_register_if_different(0x13, &(1000u16.to_le_bytes()))
            .await?;

        // Max duty cycle while not moving: 100%
        self.overwrite_register_if_different(0x1C, &(1000u16.to_le_bytes()))
            .await?;

        // Save settings to flash
        self.overwrite_register_if_different(0x04, &[0]).await?;

        Ok(())
    }

    /// angle is in degrees
    pub async fn move_to(&mut self, angle: f32) -> Result<(), DSPowerServoError<S>> {
        let angle = (angle * 10.0) as i16;
        log_info!("Moving to angle {}", angle);
        self.write_register(0x65, &angle.to_le_bytes()).await
    }

    fn craft_request(&mut self, command: Command, address: Option<u8>, parameters: &[u8]) -> usize {
        let len: usize = 6 + if address.is_some() { 1 } else { 0 } + parameters.len();

        // Request header
        self.buffer[0] = 0xF9;
        self.buffer[1] = 0xFF;
        // Super ID, assumes only one servo connected
        self.buffer[2] = 253;
        // Data length
        self.buffer[3] = len as u8 - 4;
        // Command ID
        self.buffer[4] = command as u8;
        // Address
        if let Some(address) = address {
            self.buffer[5] = address;

            // Parameters
            self.buffer[6..(6 + parameters.len())].copy_from_slice(parameters);
        }

        let mut sum = 0u8;
        for byte in self.buffer.iter().take(len - 1).skip(2) {
            sum = sum.wrapping_add(*byte);
        }
        self.buffer[len - 1] = !sum;

        log_info!("request crafted: {:?}", &self.buffer[..len]);
        len
    }

    pub async fn get_status(&mut self) -> Result<ServoStatus, DSPowerServoError<S>> {
        let request_len = self.craft_request(Command::GetStatus, None, &[]);

        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;

        let response_buffer = &mut self.buffer[..6];
        Self::read_exact_with_timeout(response_buffer, &mut self.serial, &mut self.delay).await?;
        Self::verify_checksum(response_buffer)?;

        Ok(ServoStatus::unpack(&[response_buffer[4]]).unwrap())
    }

    pub async fn batch_read_measurements(&mut self) -> Result<Measurements, DSPowerServoError<S>> {
        let request_len = self.craft_request(
            Command::ReadRegister,
            Some(0x59),
            &[0x46, 0x47, 0x48, 0x49, 0x4A],
        );

        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;

        let response_buffer = &mut self.buffer[..19];
        Self::read_exact_with_timeout(response_buffer, &mut self.serial, &mut self.delay).await?;
        Self::verify_checksum(response_buffer)?;

        let measurements_raw = MeasurementsRaw::unpack_from_slice(&response_buffer[6..18]).unwrap();

        Ok(measurements_raw.into())
    }

    async fn read_register(
        &mut self,
        address: u8,
        buf: &mut [u8],
    ) -> Result<(), DSPowerServoError<S>> {
        let request_len = self.craft_request(Command::ReadRegister, Some(address), &[]);

        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;

        let response_buffer = &mut self.buffer[..(7 + buf.len())];
        Self::read_exact_with_timeout(response_buffer, &mut self.serial, &mut self.delay).await?;
        Self::verify_checksum(response_buffer)?;

        buf.copy_from_slice(&response_buffer[6..(6 + buf.len())]);

        Ok(())
    }

    async fn write_register(
        &mut self,
        address: u8,
        value: &[u8],
    ) -> Result<(), DSPowerServoError<S>> {
        let request_len = self.craft_request(Command::WriteRegister, Some(address), value);

        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;
        log_info!("Wrote register 0x{:X} with value {:?}", address, value);

        // The servo won't send any response because we are using the Super ID

        Ok(())
    }

    async fn overwrite_register_if_different(
        &mut self,
        address: u8,
        value: &[u8],
    ) -> Result<(), DSPowerServoError<S>> {
        let request_len = self.craft_request(Command::ReadRegister, Some(address), &[]);

        log_info!("writing");
        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;
        log_info!("write done");

        let response_buffer = &mut self.buffer[..(7 + value.len())];
        Self::read_exact_with_timeout(response_buffer, &mut self.serial, &mut self.delay).await?;
        log_info!("read done");
        Self::verify_checksum(response_buffer)?;

        let existing_value = &response_buffer[6..(6 + value.len())];

        if existing_value == value {
            log_debug!("Register 0x{:X} already has value {:?}", address, value);
            return Ok(());
        }

        log_debug!(
            "Register 0x{:X} has value {:?}, overwriting with {:?}",
            address,
            existing_value,
            value
        );
        self.write_register(address, value).await
    }

    fn verify_checksum(buf: &[u8]) -> Result<(), DSPowerServoError<S>> {
        let mut sum = 0u8;
        for byte in buf.iter().take(buf.len() - 1).skip(2) {
            sum = sum.wrapping_add(*byte);
        }
        if !sum != buf[buf.len() - 1] {
            log_warn!("Received a response with invalid checksum: {:?}", buf);
            Err(DSPowerServoError::ChecksumError {
                expected: !sum,
                actual: buf[buf.len() - 1],
            })
        } else {
            Ok(())
        }
    }

    async fn read_exact_with_timeout(
        buffer: &mut [u8],
        serial: &mut S,
        delay: &mut D,
    ) -> Result<(), DSPowerServoError<S>> {
        let timeout_fut = delay.delay_ms(10);

        let read_fut = serial.read_exact(buffer);

        match select(timeout_fut, read_fut).await {
            Either::First(_) => Err(DSPowerServoError::ReadTimeout),
            Either::Second(result) => result.map_err(DSPowerServoError::from_read_exact_error),
        }
    }
}

#[cfg(test)]
mod tests {
    use log::LevelFilter;

    use super::*;

    pub(crate) fn init_logger() {
        #[cfg(feature = "log")]
        let _ = env_logger::builder()
            .filter_level(LevelFilter::Trace)
            .filter(Some("dspower_servo"), LevelFilter::Trace)
            .is_test(true)
            .try_init();
    }

    // mod unit_tests {
    //     use super::*;

    //     struct MockSerial;

    //     impl ErrorType for MockSerial {
    //         type Error = std::io::Error;
    //     }

    //     impl Read for MockSerial {
    //         async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
    //             unimplemented!()
    //         }
    //     }

    //     impl Write for MockSerial {
    //         async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
    //             unimplemented!()
    //         }
    //     }

    //     #[test]
    //     fn test_craft_result() {
    //         init_logger();

    //         let mut servo = DSPowerServo::new(MockSerial);

    //         let len = servo.craft_request(Command::GetStatus, None, &[]);
    //         assert_eq!(len, 6);
    //         assert_eq!(&servo.buffer[0..len], &[0xF9, 0xFF, 253, 2, 1, 255]);
    //     }

    //     #[test]
    //     fn test_craft_result2() {
    //         init_logger();

    //         let mut servo = DSPowerServo::new(MockSerial);

    //         let len =
    //             servo.craft_request(Command::WriteRegister, Some(0x12), &(200u16.to_le_bytes()));
    //         assert_eq!(len, 6);
    //         assert_eq!(&servo.buffer[0..len], &[0xF9, 0xFF, 253, 2, 1, 255]);
    //     }
    // }

    // mod hardware_tests {
    //     use tokio::io::{AsyncReadExt, AsyncWriteExt};
    //     use tokio_serial::SerialPortBuilderExt;

    //     use super::*;

    //     #[derive(Debug)]
    //     struct SerialWrapper(tokio_serial::SerialStream);

    //     impl ErrorType for SerialWrapper {
    //         type Error = std::io::Error;
    //     }

    //     impl Read for SerialWrapper {
    //         async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
    //             self.0.read(buf).await
    //         }
    //     }

    //     impl Write for SerialWrapper {
    //         async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
    //             self.0.write(buf).await
    //         }
    //     }

    //     #[tokio::test]
    //     async fn test_init() {
    //         init_logger();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         servo.init(true).await.unwrap();
    //     }

    //     #[tokio::test]
    //     async fn test_move() {
    //         init_logger();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         servo.init(true).await.unwrap();

    //         servo.move_to(-90.0).await.unwrap();
    //         let status = servo.get_status().await.unwrap();
    //         println!("{:?}", status);
    //     }

    //     #[tokio::test]
    //     async fn test_get_status() {
    //         init_logger();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         let status = servo.get_status().await.unwrap();
    //         println!("{:?}", status);
    //     }

    //     #[tokio::test]
    //     async fn test_read_register() {
    //         init_logger();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         // read baud rate
    //         let mut buffer = [0u8; 2];
    //         servo.read_register(0x10, &mut buffer).await.unwrap();
    //         let value = u16::from_le_bytes(buffer);
    //         assert_eq!(value, 1152);
    //     }

    //     #[tokio::test]
    //     async fn test_batch_read_measurements() {
    //         init_logger();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         let measurements = servo.batch_read_measurements().await.unwrap();

    //         println!("{:?}", measurements);
    //     }

    //     #[tokio::test]
    //     async fn run_benchmark() {
    //         use core::f32::consts::PI;
    //         use csv::Writer;
    //         use itertools::izip;
    //         use tokio::time::{self, Duration};

    //         init_logger();

    //         let mut csv_writer = Writer::from_path("output.csv").unwrap();

    //         let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
    //             .open_native_async()
    //             .unwrap();
    //         let mut servo = DSPowerServo::new(SerialWrapper(serial));

    //         servo.init(true).await.unwrap();

    //         let mut interval = time::interval(Duration::from_millis(10));

    //         let mut angles: Vec<f32> = Vec::new();

    //         // reset to zero
    //         angles.append(&mut vec![0.0; 100]);

    //         // step inputs
    //         for angle in [10, 30, 50, 70, 90, 110, 130].iter() {
    //             angles.append(&mut vec![0.0; 100]);
    //             angles.append(&mut vec![*angle as f32; 100]);
    //         }

    //         // frequency sweeps
    //         for amplitude in [10, 30, 50, 70].iter() {
    //             angles.append(&mut vec![0.0; 100]);

    //             let mut t = 0.0f32;
    //             while t < 40.0 {
    //                 let angle =
    //                     (t * PI * (1.1f32.powf(t)) / 10.0).sin() * (*amplitude as f32 / 2.0);
    //                 angles.push(angle);

    //                 t += 0.01;
    //             }
    //         }

    //         angles.append(&mut vec![0.0; 100]);

    //         let mut timestamps: Vec<f32> = Vec::new();
    //         let mut commanded_angles: Vec<f32> = Vec::new();
    //         let mut measurements_list: Vec<Measurements> = Vec::new();
    //         for (i, angle) in angles.iter().enumerate() {
    //             servo.move_to(*angle).await.unwrap();
    //             let measurements = servo.batch_read_measurements().await.unwrap();
    //             let status = servo.get_status().await.unwrap();
    //             if !status.is_ok() {
    //                 log_warn!("Servo status: {:?}", status);
    //                 log_warn!("Measurements: {:?}", measurements);
    //             }

    //             let t = i as f32 * 0.01 - 1.0;
    //             if t > 0.0 {
    //                 timestamps.push(t);
    //                 commanded_angles.push(*angle);
    //                 measurements_list.push(measurements);
    //             }

    //             interval.tick().await;
    //         }

    //         csv_writer
    //             .write_record(&[
    //                 "timestamp",
    //                 "commanded_angle",
    //                 "actual_angle",
    //                 "angular_velocity",
    //                 "current",
    //                 "pwm_duty_cycle",
    //                 "temperature",
    //             ])
    //             .unwrap();
    //         for (timestamp, commanded_angle, measurements) in
    //             izip!(timestamps, commanded_angles, measurements_list)
    //         {
    //             csv_writer
    //                 .write_record(&[
    //                     timestamp.to_string(),
    //                     commanded_angle.to_string(),
    //                     measurements.angle.to_string(),
    //                     measurements.angular_velocity.to_string(),
    //                     measurements.current.to_string(),
    //                     measurements.pwm_duty_cycle.to_string(),
    //                     measurements.temperature.to_string(),
    //                 ])
    //                 .unwrap();
    //         }
    //     }
    // }
}
