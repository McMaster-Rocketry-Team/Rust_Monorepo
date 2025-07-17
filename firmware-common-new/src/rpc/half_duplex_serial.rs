use core::fmt::Debug;

use embedded_io_async::ReadExactError;

pub trait HalfDuplexSerial {
    #[cfg(not(feature = "defmt"))]
    type Error: Debug;
    #[cfg(feature = "defmt")]
    type Error: Debug + defmt::Format;

    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, Self::Error>>;
    fn clear_read_buffer(&mut self) -> impl Future<Output = Result<(), Self::Error>>;
    fn write(&mut self, buf: &[u8]) -> impl Future<Output = Result<usize, Self::Error>>;

    fn read_exact(
        &mut self,
        mut buf: &mut [u8],
    ) -> impl Future<Output = Result<(), ReadExactError<Self::Error>>> {
        async move {
            while !buf.is_empty() {
                match self.read(buf).await {
                    Ok(0) => break,
                    Ok(n) => buf = &mut buf[n..],
                    Err(e) => return Err(ReadExactError::Other(e)),
                }
            }
            if buf.is_empty() {
                Ok(())
            } else {
                Err(ReadExactError::UnexpectedEof)
            }
        }
    }
}
