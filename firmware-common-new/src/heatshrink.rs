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

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum HeatshrinkError {
    OutBufferFull,
    Internal,
}

pub struct HeatshrinkWrapper<'a, T: HSEncoderDecoder> {
    enc: T,
    in_len: usize,
    out_buffer: &'a mut [u8],
    out_buffer_offset: usize,
}

impl<'a, T: HSEncoderDecoder> HeatshrinkWrapper<'a, T> {
    pub fn new(out_buffer: &'a mut [u8]) -> Self {
        Self {
            in_len: 0,
            enc: T::default(),
            out_buffer,
            out_buffer_offset: 0,
        }
    }

    /// returns free space avaliable in output buffer
    pub fn sink(&mut self, mut data: &[u8]) -> Result<(), HeatshrinkError> {
        self.in_len += data.len();
        while data.len() > 0 {
            if let (HSsinkRes::SinkOK, consumed_len) = self.enc.sink(data) {
                data = &data[consumed_len..];
            } else {
                return Err(HeatshrinkError::Internal);
            }

            self.poll_all()?;
        }
        Ok(())
    }

    pub fn in_len(&self) -> usize {
        self.in_len
    }

    pub fn out_buffer_len(&self) -> usize {
        self.out_buffer.len()
    }

    pub fn buffer_free_space(&self) -> usize {
        self.out_buffer.len() - self.out_buffer_offset
    }

    /// returns length of compressed buffer
    pub fn finish(mut self) -> Result<usize, HeatshrinkError> {
        while let HSfinishRes::FinishMore = self.enc.finish() {
            self.poll_all()?;
        }

        Ok(self.out_buffer_offset)
    }

    fn poll_all(&mut self) -> Result<(), HeatshrinkError> {
        loop {
            match self
                .enc
                .poll(&mut self.out_buffer[self.out_buffer_offset..])
            {
                (HSpollRes::PollMore, output_len) => {
                    self.out_buffer_offset += output_len;
                    if output_len == 0 {
                        return Err(HeatshrinkError::OutBufferFull);
                    }
                }
                (HSpollRes::PollEmpty, output_len) => {
                    self.out_buffer_offset += output_len;
                    break;
                }
                (HSpollRes::PollErrorMisuse, _) => {
                    return Err(HeatshrinkError::Internal);
                }
            }
        }
        Ok(())
    }
}
