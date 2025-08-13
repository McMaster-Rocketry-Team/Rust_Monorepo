// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]

use embassy_futures::select::{Either, select};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Error as _, ErrorKind, ErrorType, Read, ReadExactError, Write};
use packed_struct::prelude::*;

mod fmt;
mod sliding_mode_controller;

pub use sliding_mode_controller::ServoSlidingModeController;

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

    /// -1 to 1
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

pub struct DSPowerServo<'a, S>
where
    S: Read + Write,
{
    serial: &'a mut S,
    // Maximum amount of buffer needed for a single response
    buffer: [u8; 19],
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

impl<'a, S> DSPowerServo<'a, S>
where
    S: Read + Write,
{
    pub fn new(serial: &'a mut S) -> Self {
        Self {
            serial,
            buffer: [0u8; 19],
        }
    }

    pub async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), DSPowerServoError<S>> {
        log_info!("Resetting servo");

        self.write_register(0x02, &[0xE1, 0xE2, 0xE3, 0xE4]).await?;
        delay.delay_ms(100).await;

        let read_all_fut = async {
            loop {
                match self.serial.read(&mut self.buffer).await {
                    Ok(0) => break Ok(()),
                    Ok(_) => continue,
                    Err(e) if e.kind() == ErrorKind::TimedOut => break Ok(()),
                    Err(e) => break Err(DSPowerServoError::SerialError(e)),
                }
            }
        };

        let timeout_fut = delay.delay_ms(300);

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

        // Don't save settings to flash
        self.overwrite_register_if_different(0x04, &[1]).await?;

        Ok(())
    }

    /// if duty cycle is outside the range of 0.0 and 1.0,
    /// it wil be clamped to the range
    pub async fn set_max_duty_cycle(
        &mut self,
        duty_cycle: f32,
    ) -> Result<(), DSPowerServoError<S>> {
        let duty_cycle = ((duty_cycle * 1000.0) as u16).min(1000).to_le_bytes();

        // Max duty cycle
        self.overwrite_register_if_different(0x13, &duty_cycle).await?;

        // Max duty cycle while not moving
        self.overwrite_register_if_different(0x1C, &duty_cycle).await?;

        Ok(())
    }

    /// angle is in degrees
    pub async fn move_to(&mut self, angle: f32) -> Result<(), DSPowerServoError<S>> {
        let angle = (angle * 10.0) as i16;
        log_trace!("Moving to angle {}", angle);
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

        log_trace!("request crafted: {:?}", &self.buffer[..len]);
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
        self.serial
            .read_exact(response_buffer)
            .await
            .map_err(DSPowerServoError::from_read_exact_error)?;
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
        self.serial
            .read_exact(response_buffer)
            .await
            .map_err(DSPowerServoError::from_read_exact_error)?;
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
        self.serial
            .read_exact(response_buffer)
            .await
            .map_err(DSPowerServoError::from_read_exact_error)?;
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
        log_trace!("Wrote register 0x{:X} with value {:?}", address, value);

        // The servo won't send any response because we are using the Super ID

        Ok(())
    }

    async fn overwrite_register_if_different(
        &mut self,
        address: u8,
        value: &[u8],
    ) -> Result<(), DSPowerServoError<S>> {
        let request_len = self.craft_request(Command::ReadRegister, Some(address), &[]);

        log_trace!("writing");
        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(DSPowerServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(DSPowerServoError::SerialError)?;
        log_trace!("write done");

        let response_buffer = &mut self.buffer[..(7 + value.len())];
        self.serial
            .read_exact(response_buffer)
            .await
            .map_err(DSPowerServoError::from_read_exact_error)?;
        log_trace!("read done");
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

    mod unit_tests {
        use super::*;

        struct MockSerial;

        impl ErrorType for MockSerial {
            type Error = std::io::Error;
        }

        impl Read for MockSerial {
            async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
                unimplemented!()
            }
        }

        impl Write for MockSerial {
            async fn write(&mut self, _buf: &[u8]) -> Result<usize, Self::Error> {
                unimplemented!()
            }
        }

        #[test]
        fn test_craft_result() {
            init_logger();

            let mut serial = MockSerial;
            let mut servo = DSPowerServo::new(&mut serial);

            let len = servo.craft_request(Command::GetStatus, None, &[]);
            assert_eq!(len, 6);
            assert_eq!(&servo.buffer[0..len], &[0xF9, 0xFF, 253, 2, 1, 255]);
        }

        #[test]
        fn test_craft_result2() {
            init_logger();

            let mut serial = MockSerial;
            let mut servo = DSPowerServo::new(&mut serial);

            let len =
                servo.craft_request(Command::WriteRegister, Some(0x12), &(200u16.to_le_bytes()));
            assert_eq!(len, 9);
            assert_eq!(
                &servo.buffer[0..len],
                &[0xF9, 0xFF, 253, 5, 3, 0x12, 200, 0, 32]
            );
        }
    }

    mod hardware_tests {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        use tokio_serial::SerialPortBuilderExt;

        use super::*;

        #[derive(Debug)]
        struct SerialWrapper(tokio_serial::SerialStream);

        impl ErrorType for SerialWrapper {
            type Error = std::io::Error;
        }

        impl Read for SerialWrapper {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                self.0.read(buf).await
            }
        }

        impl Write for SerialWrapper {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                self.0.write(buf).await
            }
        }

        #[tokio::test]
        async fn test_init() {
            init_logger();

            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            let mut serial = SerialWrapper(serial);
            let mut servo = DSPowerServo::new(&mut serial);

            servo.init(true).await.unwrap();
        }

        #[tokio::test]
        async fn test_move() {
            init_logger();

            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            let mut serial = SerialWrapper(serial);
            let mut servo = DSPowerServo::new(&mut serial);

            servo.init(true).await.unwrap();

            servo.move_to(-90.0).await.unwrap();
            let status = servo.get_status().await.unwrap();
            println!("{:?}", status);
        }

        #[tokio::test]
        async fn test_get_status() {
            init_logger();

            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            let mut serial = SerialWrapper(serial);
            let mut servo = DSPowerServo::new(&mut serial);

            let status = servo.get_status().await.unwrap();
            println!("{:?}", status);
        }

        #[tokio::test]
        async fn test_read_register() {
            init_logger();

            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            let mut serial = SerialWrapper(serial);
            let mut servo = DSPowerServo::new(&mut serial);

            // read baud rate
            let mut buffer = [0u8; 2];
            servo.read_register(0x10, &mut buffer).await.unwrap();
            let value = u16::from_le_bytes(buffer);
            assert_eq!(value, 1152);
        }

        #[tokio::test]
        async fn test_batch_read_measurements() {
            init_logger();

            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            let mut serial = SerialWrapper(serial);
            let mut servo = DSPowerServo::new(&mut serial);

            let measurements = servo.batch_read_measurements().await.unwrap();

            println!("{:?}", measurements);
        }
    }
}
