use crate::driver::{
    crc::Crc,
    flash::{SpiFlash, SpiFlashError},
};

use super::{
    io_traits::{AsyncReader, AsyncWriter},
    u8_crc::U8Crc,
};

pub struct FlashReader<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    address: u32,
    flash: &'a mut F,
    crc: U8Crc<'a, C>,
}

impl<'a, 'b, F, C> FlashReader<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    pub fn new(start_address: u32, flash: &'a mut F, crc: &'a mut C) -> Self {
        crc.reset();
        Self {
            address: start_address,
            flash,
            crc: U8Crc::new(crc),
        }
    }

    pub fn get_crc(&self) -> u32 {
        self.crc.read_crc()
    }
}

impl<'a, F, C> AsyncReader for FlashReader<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    type Error = SpiFlashError;

    // maximum read length is the length of buffer - 5 bytes
    // TODO implement read-ahead
    async fn read_slice<'b>(
        &mut self,
        buffer: &'b mut [u8],
        length: usize,
    ) -> Result<&'b [u8], SpiFlashError> {
        self.flash.read(self.address, length, buffer).await?;
        self.address += length as u32;

        let read_result = &buffer[5..(length + 5)];
        self.crc.process(read_result);
        Ok(read_result)
    }
}

pub struct FlashWriter<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    page_address: u32,
    flash: &'a mut F,
    crc: U8Crc<'a, C>,
    buffer: [u8; 5 + 256],
    buffer_offset: usize,
}

impl<'a, F, C> FlashWriter<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    pub fn new(start_address: u32, flash: &'a mut F, crc: &'a mut C) -> Self {
        crc.reset();
        Self {
            page_address: start_address,
            flash,
            crc: U8Crc::new(crc),
            buffer: [0xFF; 5 + 256],
            buffer_offset: 5,
        }
    }

    pub fn get_crc(&self) -> u32 {
        self.crc.read_crc()
    }

    pub async fn flush(&mut self) -> Result<(), SpiFlashError> {
        self.flash
            .write_256b(self.page_address, &mut self.buffer)
            .await?;
        self.buffer = [0xFF; 5 + 256];
        self.buffer_offset = 5;
        Ok(())
    }
}

impl<'a, F, C> AsyncWriter for FlashWriter<'a, F, C>
where
    F: SpiFlash,
    C: Crc,
{
    type Error = SpiFlashError;

    async fn extend_from_slice(&mut self, slice: &[u8]) -> Result<(), SpiFlashError> {
        self.crc.process(slice);

        let mut slice = slice;
        while slice.len() > 0 {
            let buffer_free = self.buffer.len() - self.buffer_offset;

            if slice.len() < buffer_free {
                (&mut self.buffer[self.buffer_offset..(self.buffer_offset + slice.len())])
                    .copy_from_slice(slice);
                self.buffer_offset += slice.len();

                slice = &[];
            } else {
                (&mut self.buffer[self.buffer_offset..]).copy_from_slice(&slice[..buffer_free]);
                self.buffer_offset += buffer_free;

                self.flush().await?;

                slice = &slice[buffer_free..];
            }
        }

        Ok(())
    }
}