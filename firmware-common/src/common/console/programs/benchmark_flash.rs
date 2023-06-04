use core::hint::black_box;

use defmt::*;

use rand::{rngs::SmallRng, RngCore, SeedableRng};
use vlfs::{
    io_traits::{AsyncReader, AsyncWriter},
    Crc, Flash, VLFS,
};

use crate::{
    common::files::BENCHMARK_FILE_TYPE,
    driver::{serial::Serial, timer::Timer},
};

pub struct BenchmarkFlash {}

impl BenchmarkFlash {
    pub fn new() -> Self {
        Self {}
    }

    pub fn id(&self) -> u64 {
        0x2
    }

    pub async fn start<T: Serial, F: Flash, C: Crc, I: Timer>(
        &self,
        _serial: &mut T,
        vlfs: &VLFS<F, C>,
        timer: I,
    ) -> Result<(), ()> {
        let rounds = 10000usize;
        let length = rounds * 64;

        let random_time = {
            let mut rng = SmallRng::seed_from_u64(
                0b1010011001010000010000000111001110111101011110001100000011100000u64,
            );

            let start_time = timer.now_mills();
            let mut buffer = [0u8; 64];
            for _ in 0..rounds {
                rng.fill_bytes(&mut buffer);
                black_box(&buffer);
            }
            timer.now_mills() - start_time
        };

        let mut max_64b_write_time = 0f64;
        let file_id = unwrap!(vlfs.create_file(BENCHMARK_FILE_TYPE).await);

        let write_time = {
            let mut rng = SmallRng::seed_from_u64(
                0b1010011001010000010000000111001110111101011110001100000011100000u64,
            );

            let start_time = timer.now_mills();

            let mut file = unwrap!(vlfs.open_file_for_write(file_id).await);
            let mut buffer = [0u8; 64];
            for _ in 0..rounds {
                rng.fill_bytes(&mut buffer);
                let write_64b_start_time = timer.now_mills();
                unwrap!(file.extend_from_slice(&buffer).await);
                let write_64b_end_time = timer.now_mills() - write_64b_start_time;
                if write_64b_end_time > max_64b_write_time {
                    max_64b_write_time = write_64b_end_time;
                }
            }
            unwrap!(file.close().await);
            timer.now_mills() - start_time - random_time
        };

        let read_time = {
            let mut rng = SmallRng::seed_from_u64(
                0b1010011001010000010000000111001110111101011110001100000011100000u64,
            );

            let start_time = timer.now_mills();
            let mut buffer = [0u8; 64];
            let mut buffer_expected = [0u8; 64];
            let mut file = unwrap!(vlfs.open_file_for_read(file_id).await);
            for _ in 0..rounds {
                unwrap!(file.read_slice(&mut buffer, 64).await);
                rng.fill_bytes(&mut buffer_expected);
                if buffer != buffer_expected {
                    warn!(
                        "Buffer mismatch! actual: {=[u8]:X}, expected: {=[u8]:X}",
                        buffer, buffer_expected
                    );
                }
            }
            file.close().await;
            timer.now_mills() - start_time - random_time
        };

        vlfs.remove_file(file_id).await;

        info!(
            "Write speed: {}KiB/s",
            (length as f32 / 1024.0) / (write_time as f32 / 1000.0)
        );
        info!(
            "64 bytes writing time: mean: {}ms, max: {}ms",
            (write_time as f32) / (rounds as f32),
            max_64b_write_time
        );

        info!(
            "Read speed: {}KiB/s",
            (length as f32 / 1024.0) / (read_time as f32 / 1000.0)
        );

        Ok(())
    }
}