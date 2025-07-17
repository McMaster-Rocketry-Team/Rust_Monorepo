use std::time::Duration;

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorType, Read, Write};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time::sleep,
};

use anyhow::Result;

#[derive(Debug)]
pub struct SerialWrapper(pub tokio_serial::SerialStream);

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

pub struct Delay;

impl DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        sleep(Duration::from_nanos(ns as u64)).await;
    }
}
