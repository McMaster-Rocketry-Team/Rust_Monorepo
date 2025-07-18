use embedded_io_async::{ErrorType, Read, Write};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

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
