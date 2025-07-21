use std::sync::{Arc, Mutex};

use firmware_common_new::rpc::half_duplex_serial::HalfDuplexSerial;
use serialport::{ClearBuffer, SerialPort};

use anyhow::Result;
use tokio::task::spawn_blocking;

#[derive(Debug)]
pub struct SerialWrapper(Arc<Mutex<Box<dyn SerialPort>>>);

impl SerialWrapper {
    pub fn new(serial: Box<dyn SerialPort>) -> Self {
        Self(Arc::new(Mutex::new(serial)))
    }
}


impl HalfDuplexSerial for SerialWrapper {
    type Error = serialport::Error;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let serial = self.0.clone();
        let buffer_len = buf.len();
        let (buf2, result) = spawn_blocking(move || {
            let mut serial = serial.lock().unwrap();
            let mut buf = vec![0u8; buffer_len];

            let result = serial.read(&mut buf).map_err(serialport::Error::from);

            (buf, result)
        })
        .await
        .unwrap();

        buf.copy_from_slice(&buf2);
        result
    }

    async fn clear_read_buffer(&mut self) -> Result<(), Self::Error> {
        let serial = self.0.clone();

        spawn_blocking(move || {
            let serial = serial.lock().unwrap();
            serial.clear(ClearBuffer::Input)
        })
        .await
        .unwrap()
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let serial = self.0.clone();
        let buffer = Vec::from(buf);

        spawn_blocking(move || {
            let mut serial = serial.lock().unwrap();
            serial.write(&buffer).map_err(serialport::Error::from)
        })
        .await
        .unwrap()
    }
}

impl SerialWrapper {
    pub fn set_dtr(&mut self, dtr: bool) -> Result<(), serialport::Error> {
        let mut serial =self.0.lock().unwrap();
        serial.write_data_terminal_ready(dtr)
    }
}
