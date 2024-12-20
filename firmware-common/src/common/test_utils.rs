use core::convert::Infallible;

pub(crate) struct BufferWriter<'a, const N: usize> {
    pub buffer: &'a mut [u8; N],
    pub offset: usize,
}

impl<'a, const N: usize> BufferWriter<'a, N> {
    pub(crate) fn new(buffer: &'a mut [u8; N]) -> Self {
        Self { buffer, offset: 0 }
    }

    pub(crate) fn into_reader(self) -> BufferReader<'a, N> {
        BufferReader::new(self.buffer, self.offset)
    }
}

impl<'a, const N: usize> embedded_io_async::ErrorType for BufferWriter<'a, N> {
    type Error = Infallible;
}

impl<'a, const N: usize> embedded_io_async::Write for BufferWriter<'a, N> {
    async fn write(&mut self, slice: &[u8]) -> Result<usize, Self::Error> {
        self.buffer[self.offset..self.offset + slice.len()].copy_from_slice(slice);
        self.offset += slice.len();
        Ok(slice.len())
    }
}

pub(crate) struct BufferReader<'b, const N: usize> {
    pub buffer: &'b [u8; N],
    pub offset: usize,
    pub len: usize,
}

impl<'b, const N: usize> BufferReader<'b, N> {
    pub(crate) fn new(buffer: &'b [u8; N], len: usize) -> Self {
        Self {
            buffer,
            offset: 0,
            len,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.len - self.offset
    }
}

impl<'b, const N: usize> embedded_io_async::ErrorType for BufferReader<'b, N> {
    type Error = Infallible;
}

impl<'b, const N: usize> embedded_io_async::Read for BufferReader<'b, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if self.len() == 0 {
            Ok(0)
        } else if buf.len() > self.len() {
            let len = self.len();
            (&mut buf[..len]).copy_from_slice(&self.buffer[self.offset..self.len]);
            self.offset = self.len;
            Ok(len)
        } else {
            buf.copy_from_slice(&self.buffer[self.offset..self.offset + buf.len()]);
            self.offset += buf.len();
            Ok(buf.len())
        }
    }
}
