// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]

//! Driver for Hiwonder serial bus servos (LX-16A family).
//!
//! The servos speak a half-duplex UART protocol at 115200 baud. Multiple
//! servos share the same bus; each is addressed by a 1-byte ID. This driver
//! targets a single servo at a known ID (stored in [`HiwonderServo::id`]),
//! matching the "one servo per handle" style of the sibling `dspower-servo`
//! crate.
//!
//! Frame layout (request and response are identical):
//!
//! ```text
//! 0x55 0x55 | ID | Length | Cmd | Prm 1 ... Prm N | Checksum
//! ```
//!
//! - `Length` = number of parameters + 3
//! - `Checksum` = `!(ID + Length + Cmd + Prm1 + ... + PrmN)` (lowest byte)

use embedded_io_async::{ErrorType, Read, ReadExactError, Write};
use packed_struct::prelude::*;

mod fmt;
mod sliding_mode_controller;

pub use sliding_mode_controller::ServoSlidingModeController;

/// ID that every servo on the bus listens to. Servos do not reply to
/// broadcast requests, except for [`Command::IdRead`].
pub const BROADCAST_ID: u8 = 0xFE;

/// Position units (0..=1000) span 0..=240 degrees, i.e. 0.24 deg per unit.
const DEGREES_PER_UNIT: f32 = 0.24;

/// Faults that make the servo flash its LED and cut torque.
///
/// This maps the 0..=7 value used by [`Command::LedErrorRead`] /
/// [`Command::LedErrorWrite`] onto its individual bits.
#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct ServoFault {
    #[packed_field(bits = "0")]
    pub over_temperature: bool,
    #[packed_field(bits = "1")]
    pub over_voltage: bool,
    #[packed_field(bits = "2")]
    pub stalled: bool,
}

impl ServoFault {
    pub const NONE: ServoFault = ServoFault {
        over_temperature: false,
        over_voltage: false,
        stalled: false,
    };

    pub fn is_ok(&self) -> bool {
        !self.over_temperature && !self.over_voltage && !self.stalled
    }
}

pub struct HiwonderServo<'a, S>
where
    S: Read + Write,
{
    serial: &'a mut S,
    /// The bus ID this handle talks to.
    pub id: u8,
    // Maximum amount of buffer needed for a single request or response
    buffer: [u8; 10],
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum HiwonderServoError<S: ErrorType> {
    SerialError(S::Error),
    UnexpectedEof,
    ChecksumError { expected: u8, actual: u8 },
    InvalidHeader { actual: [u8; 2] },
    UnexpectedCommand { expected: u8, actual: u8 },
    ReadTimeout,
}

impl<S: ErrorType> HiwonderServoError<S> {
    fn from_read_exact_error(error: ReadExactError<S::Error>) -> Self {
        match error {
            ReadExactError::Other(e) => HiwonderServoError::SerialError(e),
            ReadExactError::UnexpectedEof => HiwonderServoError::UnexpectedEof,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Command {
    MoveTimeWrite = 1,
    MoveTimeRead = 2,
    MoveStart = 11,
    MoveStop = 12,
    IdWrite = 13,
    IdRead = 14,
    AngleLimitWrite = 20,
    AngleLimitRead = 21,
    VinLimitWrite = 22,
    VinLimitRead = 23,
    TempMaxLimitWrite = 24,
    TempMaxLimitRead = 25,
    TempRead = 26,
    VinRead = 27,
    PosRead = 28,
    MotorModeWrite = 29,
    MotorModeRead = 30,
    LoadOrUnloadWrite = 31,
    LoadOrUnloadRead = 32,
    LedCtrlWrite = 33,
    LedCtrlRead = 34,
    LedErrorWrite = 35,
    LedErrorRead = 36,
}

fn degrees_to_position(angle: f32) -> u16 {
    let units = (angle / DEGREES_PER_UNIT) as i32;
    units.clamp(0, 1000) as u16
}

fn position_to_degrees(position: i16) -> f32 {
    position as f32 * DEGREES_PER_UNIT
}

impl<'a, S> HiwonderServo<'a, S>
where
    S: Read + Write,
{
    /// Create a handle to the servo at bus `id`.
    pub fn new(serial: &'a mut S, id: u8) -> Self {
        Self {
            serial,
            id,
            buffer: [0u8; 10],
        }
    }

    /// Configure the servo for maximum performance.
    ///
    /// This pins the internal limits wide open — full 0..=240 deg travel, the
    /// widest input-voltage window, and the highest temperature cutoff — so the
    /// servo never derates or unloads under normal operation, then selects
    /// position control with torque enabled. These knobs are deliberately not
    /// exposed individually; `init` is the one place they are set.
    ///
    /// `enable_protections` decides whether over-temperature / over-voltage /
    /// stall faults flash the LED and unload the motor (`true`) or are ignored
    /// for absolute maximum availability (`false`).
    ///
    /// The persisted (flash-backed) settings are read first and only rewritten
    /// when they differ, to avoid needless flash wear.
    pub async fn init(&mut self, enable_protections: bool) -> Result<(), HiwonderServoError<S>> {
        log_info!("Initializing servo {}", self.id);

        // Full 0..=1000 (0..=240 deg) travel: no software angle clamp.
        let mut angle_limits = [0u8; 4];
        angle_limits[0..2].copy_from_slice(&0u16.to_le_bytes());
        angle_limits[2..4].copy_from_slice(&1000u16.to_le_bytes());
        self.ensure_setting(Command::AngleLimitRead, Command::AngleLimitWrite, &angle_limits)
            .await?;

        // Widest allowed input-voltage window (4.5..=12.0 V) so brownouts do
        // not unload the motor.
        let mut voltage_limits = [0u8; 4];
        voltage_limits[0..2].copy_from_slice(&4500u16.to_le_bytes());
        voltage_limits[2..4].copy_from_slice(&12000u16.to_le_bytes());
        self.ensure_setting(Command::VinLimitRead, Command::VinLimitWrite, &voltage_limits)
            .await?;

        // Highest allowed temperature cutoff (100 C).
        self.ensure_setting(Command::TempMaxLimitRead, Command::TempMaxLimitWrite, &[100])
            .await?;

        // Which faults flash the LED and unload the motor.
        let alarm = if enable_protections { 0b0000_0111 } else { 0 };
        self.ensure_setting(Command::LedErrorRead, Command::LedErrorWrite, &[alarm])
            .await?;

        // LED on (0 = always on).
        self.ensure_setting(Command::LedCtrlRead, Command::LedCtrlWrite, &[0])
            .await?;

        // Position control mode and torque enabled: neither survives a power
        // cycle, so they are always written fresh. (1 = torque loaded/on.)
        self.set_position_mode().await?;
        self.write_command(Command::LoadOrUnloadWrite, &[1]).await?;

        Ok(())
    }

    /// Move to `angle` (degrees, 0..=240) as fast as the servo can. The angle
    /// is clamped to the valid range.
    pub async fn move_to(&mut self, angle: f32) -> Result<(), HiwonderServoError<S>> {
        let position = degrees_to_position(angle);
        log_trace!("Moving to {} deg (pos {})", angle, position);

        // params[0..2] = position, params[2..4] = time; time 0 = maximum speed.
        let mut params = [0u8; 4];
        params[0..2].copy_from_slice(&position.to_le_bytes());
        self.write_command(Command::MoveTimeWrite, &params).await
    }

    /// Immediately stop an in-progress move and hold the current angle.
    pub async fn stop(&mut self) -> Result<(), HiwonderServoError<S>> {
        self.write_command(Command::MoveStop, &[]).await
    }

    /// Read the current angle in degrees. May be slightly negative or above
    /// 240 because of the angle-offset trim.
    pub async fn read_position(&mut self) -> Result<f32, HiwonderServoError<S>> {
        let params = self.query(Command::PosRead, 2).await?;
        let raw = i16::from_le_bytes([params[0], params[1]]);
        Ok(position_to_degrees(raw))
    }

    /// Read the internal temperature in degrees Celsius.
    pub async fn read_temperature(&mut self) -> Result<i16, HiwonderServoError<S>> {
        let params = self.query(Command::TempRead, 1).await?;
        Ok(params[0] as i16)
    }

    /// Read the input voltage in Volts.
    pub async fn read_voltage(&mut self) -> Result<f32, HiwonderServoError<S>> {
        let params = self.query(Command::VinRead, 2).await?;
        let millivolts = u16::from_le_bytes([params[0], params[1]]);
        Ok(millivolts as f32 / 1000.0)
    }

    /// Read the active fault flags (parity with `dspower-servo`'s `get_status`).
    pub async fn read_fault(&mut self) -> Result<ServoFault, HiwonderServoError<S>> {
        let params = self.query(Command::LedErrorRead, 1).await?;
        Ok(ServoFault::unpack(&[params[0] & 0b0000_0111]).unwrap())
    }

    /// Read the servo's ID. Uses the broadcast address so a single servo of
    /// unknown ID can be discovered; only valid when exactly one servo is on
    /// the bus.
    pub async fn read_id(&mut self) -> Result<u8, HiwonderServoError<S>> {
        let request_len = self.craft_request(BROADCAST_ID, Command::IdRead, &[]);
        self.send(request_len).await?;

        let response = self.read_response(Command::IdRead, 1).await?;
        Ok(response[0])
    }

    /// Persistently change the servo's bus ID and retarget this handle to it.
    pub async fn set_id(&mut self, new_id: u8) -> Result<(), HiwonderServoError<S>> {
        self.write_command(Command::IdWrite, &[new_id]).await?;
        self.id = new_id;
        Ok(())
    }

    /// Put the servo in position control mode (0..=240 degrees).
    async fn set_position_mode(&mut self) -> Result<(), HiwonderServoError<S>> {
        self.write_command(Command::MotorModeWrite, &[0, 0, 0, 0])
            .await
    }

    /// Turn the servo's status LED on or off.
    pub async fn set_led(&mut self, on: bool) -> Result<(), HiwonderServoError<S>> {
        // 0 = always on, 1 = off
        self.write_command(Command::LedCtrlWrite, &[if on { 0 } else { 1 }])
            .await
    }

    /// Write a flash-persisted setting only if it currently differs, to avoid
    /// needless flash wear (these settings survive power-off).
    async fn ensure_setting(
        &mut self,
        read: Command,
        write: Command,
        desired: &[u8],
    ) -> Result<(), HiwonderServoError<S>> {
        let current = self.query(read, desired.len()).await?;
        if current == desired {
            log_debug!("Setting {} already at desired value", write as u8);
            return Ok(());
        }
        log_debug!("Writing setting {} = {:?}", write as u8, desired);
        self.write_command(write, desired).await
    }

    /// Fill `self.buffer` with a request frame, returning its total length.
    fn craft_request(&mut self, id: u8, command: Command, parameters: &[u8]) -> usize {
        // header(2) + id(1) + length(1) + command(1) + params(N) + checksum(1)
        let len = 6 + parameters.len();

        self.buffer[0] = 0x55;
        self.buffer[1] = 0x55;
        self.buffer[2] = id;
        // Length counts everything from itself up to and including the checksum.
        self.buffer[3] = parameters.len() as u8 + 3;
        self.buffer[4] = command as u8;
        self.buffer[5..(5 + parameters.len())].copy_from_slice(parameters);

        let mut sum = 0u8;
        for byte in self.buffer.iter().take(len - 1).skip(2) {
            sum = sum.wrapping_add(*byte);
        }
        self.buffer[len - 1] = !sum;

        log_trace!("request crafted: {:?}", &self.buffer[..len]);
        len
    }

    /// Write the first `request_len` bytes of `self.buffer` and flush.
    async fn send(&mut self, request_len: usize) -> Result<(), HiwonderServoError<S>> {
        self.serial
            .write_all(&self.buffer[..request_len])
            .await
            .map_err(HiwonderServoError::SerialError)?;
        self.serial
            .flush()
            .await
            .map_err(HiwonderServoError::SerialError)?;
        Ok(())
    }

    /// Send a request that expects no reply (write commands, broadcast-free).
    async fn write_command(
        &mut self,
        command: Command,
        parameters: &[u8],
    ) -> Result<(), HiwonderServoError<S>> {
        let request_len = self.craft_request(self.id, command, parameters);
        self.send(request_len).await?;
        log_trace!("Sent command {}", command as u8);
        Ok(())
    }

    /// Read a `response_params`-byte reply, validate it, and return the
    /// parameter bytes.
    async fn read_response(
        &mut self,
        command: Command,
        response_params: usize,
    ) -> Result<&[u8], HiwonderServoError<S>> {
        let response_len = 6 + response_params;
        self.serial
            .read_exact(&mut self.buffer[..response_len])
            .await
            .map_err(HiwonderServoError::from_read_exact_error)?;

        let response = &self.buffer[..response_len];
        if response[0] != 0x55 || response[1] != 0x55 {
            log_warn!("Response with invalid header: {:?}", response);
            return Err(HiwonderServoError::InvalidHeader {
                actual: [response[0], response[1]],
            });
        }
        Self::verify_checksum(response)?;
        if response[4] != command as u8 {
            log_warn!(
                "Response for command {} but expected {}",
                response[4],
                command as u8
            );
            return Err(HiwonderServoError::UnexpectedCommand {
                expected: command as u8,
                actual: response[4],
            });
        }

        Ok(&self.buffer[5..(5 + response_params)])
    }

    /// Send a read command to `self.id` and return its `response_params` bytes.
    async fn query(
        &mut self,
        command: Command,
        response_params: usize,
    ) -> Result<&[u8], HiwonderServoError<S>> {
        let request_len = self.craft_request(self.id, command, &[]);
        self.send(request_len).await?;
        self.read_response(command, response_params).await
    }

    fn verify_checksum(buf: &[u8]) -> Result<(), HiwonderServoError<S>> {
        let mut sum = 0u8;
        for byte in buf.iter().take(buf.len() - 1).skip(2) {
            sum = sum.wrapping_add(*byte);
        }
        if !sum != buf[buf.len() - 1] {
            log_warn!("Received a response with invalid checksum: {:?}", buf);
            Err(HiwonderServoError::ChecksumError {
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
            .filter(Some("hiwonder_servo"), LevelFilter::Trace)
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
        fn test_craft_move() {
            init_logger();

            let mut serial = MockSerial;
            let mut servo = HiwonderServo::new(&mut serial, 1);

            // Move servo 1 to position 500 (120 deg) in 1000 ms.
            let mut params = [0u8; 4];
            params[0..2].copy_from_slice(&500u16.to_le_bytes());
            params[2..4].copy_from_slice(&1000u16.to_le_bytes());
            let len = servo.craft_request(1, Command::MoveTimeWrite, &params);

            assert_eq!(len, 10);
            assert_eq!(
                &servo.buffer[0..len],
                &[0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03, 0x16]
            );
        }

        #[test]
        fn test_craft_pos_read() {
            init_logger();

            let mut serial = MockSerial;
            let mut servo = HiwonderServo::new(&mut serial, 1);

            let len = servo.craft_request(1, Command::PosRead, &[]);

            assert_eq!(len, 6);
            assert_eq!(&servo.buffer[0..len], &[0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF]);
        }

        #[test]
        fn test_degree_position_roundtrip() {
            assert_eq!(degrees_to_position(0.0), 0);
            assert_eq!(degrees_to_position(120.0), 500);
            assert_eq!(degrees_to_position(240.0), 1000);
            // clamped
            assert_eq!(degrees_to_position(300.0), 1000);
            assert_eq!(degrees_to_position(-10.0), 0);

            assert_eq!(position_to_degrees(500), 120.0);
        }

        #[test]
        fn test_fault_bits() {
            let fault = ServoFault::unpack(&[0b101]).unwrap();
            assert!(fault.over_temperature);
            assert!(!fault.over_voltage);
            assert!(fault.stalled);
            assert!(!fault.is_ok());

            assert!(ServoFault::NONE.is_ok());
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

        fn open() -> SerialWrapper {
            let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
                .open_native_async()
                .unwrap();
            SerialWrapper(serial)
        }

        #[tokio::test]
        async fn test_init() {
            init_logger();

            let mut serial = open();
            let mut servo = HiwonderServo::new(&mut serial, 1);

            servo.init(true).await.unwrap();
        }

        #[tokio::test]
        async fn test_move() {
            init_logger();

            let mut serial = open();
            let mut servo = HiwonderServo::new(&mut serial, 1);

            servo.init(true).await.unwrap();
            servo.move_to(120.0).await.unwrap();
        }

        #[tokio::test]
        async fn test_read_id() {
            init_logger();

            let mut serial = open();
            let mut servo = HiwonderServo::new(&mut serial, 1);

            let id = servo.read_id().await.unwrap();
            println!("servo id: {}", id);
        }

        #[tokio::test]
        async fn test_read_position() {
            init_logger();

            let mut serial = open();
            let mut servo = HiwonderServo::new(&mut serial, 1);

            let angle = servo.read_position().await.unwrap();
            println!("angle: {}", angle);
        }
    }
}
