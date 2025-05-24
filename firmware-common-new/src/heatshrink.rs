use heatshrink::{
    HSfinishRes, HSpollRes, HSsinkRes, decoder::HeatshrinkDecoder, encoder::HeatshrinkEncoder,
};

pub trait HSEncoderDecoder: Default {
    fn sink(&mut self, input_buffer: &[u8]) -> (HSsinkRes, usize);
    fn poll(&mut self, output_buffer: &mut [u8]) -> (HSpollRes, usize);
    fn finish(&mut self) -> HSfinishRes;
}

impl HSEncoderDecoder for HeatshrinkEncoder {
    fn sink(&mut self, input_buffer: &[u8]) -> (HSsinkRes, usize) {
        HeatshrinkEncoder::sink(self, input_buffer)
    }

    fn poll(&mut self, output_buffer: &mut [u8]) -> (HSpollRes, usize) {
        HeatshrinkEncoder::poll(self, output_buffer)
    }

    fn finish(&mut self) -> HSfinishRes {
        HeatshrinkEncoder::finish(self)
    }
}

impl HSEncoderDecoder for HeatshrinkDecoder {
    fn sink(&mut self, input_buffer: &[u8]) -> (HSsinkRes, usize) {
        HeatshrinkDecoder::sink(self, input_buffer)
    }

    fn poll(&mut self, output_buffer: &mut [u8]) -> (HSpollRes, usize) {
        HeatshrinkDecoder::poll(self, output_buffer)
    }

    fn finish(&mut self) -> HSfinishRes {
        HeatshrinkDecoder::finish(self)
    }
}

pub struct HeatshrinkWrapper<'a, T: HSEncoderDecoder> {
    enc: T,
    out_buffer: &'a mut [u8],
    out_buffer_offset: usize,
}

impl<'a, T: HSEncoderDecoder> HeatshrinkWrapper<'a, T> {
    pub fn new(out_buffer: &'a mut [u8]) -> Self {
        Self {
            enc: T::default(),
            out_buffer,
            out_buffer_offset: 0,
        }
    }

    /// returns free space avaliable in output buffer
    pub fn sink(&mut self, mut data: &[u8]) -> Result<(), ()> {
        while data.len() > 0 {
            if let (HSsinkRes::SinkOK, consumed_len) = self.enc.sink(data) {
                data = &data[consumed_len..];
            } else {
                return Err(());
            }

            self.poll_all()?;
        }
        Ok(())
    }

    pub fn buffer_free_space(&self) -> usize {
        self.out_buffer.len() - self.out_buffer_offset
    }

    /// returns length of compressed buffer
    pub fn finish(mut self) -> Result<usize, ()> {
        while let HSfinishRes::FinishMore = self.enc.finish() {
            self.poll_all()?;
        }

        Ok(self.out_buffer_offset)
    }

    fn poll_all(&mut self) -> Result<(), ()> {
        loop {
            match self
                .enc
                .poll(&mut self.out_buffer[self.out_buffer_offset..])
            {
                (HSpollRes::PollMore, output_len) => {
                    self.out_buffer_offset += output_len;
                }
                (HSpollRes::PollEmpty, output_len) => {
                    self.out_buffer_offset += output_len;
                    break;
                }
                (HSpollRes::PollErrorMisuse, _) => {
                    return Err(());
                }
            }
        }
        Ok(())
    }
}
