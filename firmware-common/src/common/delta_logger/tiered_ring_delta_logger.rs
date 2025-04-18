use futures::join;
use vlfs::{Crc, Flash, VLFSError, VLFS};
use core::mem::size_of;

use crate::{
    common::{
        fixed_point::F64FixedPointFactory,
        sensor_reading::{SensorData, SensorReading},
    },
    driver::{clock::Clock, delay::Delay, timestamp::BootTimestamp},
};

use super::{
    delta_logger::UnixTimestampLog, prelude::DeltaLoggerTrait, ring_delta_logger::{RingDeltaLogger, RingDeltaLoggerConfig}
};

pub struct TieredRingDeltaLogger<'a, D, C, F, FF1, FF2, DL, CL>
where
    C: Crc,
    F: Flash,
    F::Error: defmt::Format,
    D: SensorData,
    FF1: F64FixedPointFactory,
    FF2: F64FixedPointFactory,
    DL: Delay,
    CL: Clock,
    [(); size_of::<D>() + 10]:,
{
    delta_logger_1: RingDeltaLogger<'a,'a, D, C, F, FF1, DL, CL>,
    delta_logger_2: RingDeltaLogger<'a,'a, D, C, F, FF2, DL, CL>,
}

impl<'a, D, C, F, FF1, FF2, DL, CL> TieredRingDeltaLogger<'a, D, C, F, FF1, FF2, DL, CL>
where
    C: Crc,
    F: Flash,
    F::Error: defmt::Format,
    D: SensorData,
    FF1: F64FixedPointFactory,
    FF2: F64FixedPointFactory,
    DL: Delay,
    CL: Clock,
    [(); size_of::<D>() + 10]:,
{
    pub async fn new(
        fs: &'a VLFS<F, C>,
        configs: (RingDeltaLoggerConfig, RingDeltaLoggerConfig),
        delay: DL,
        clock: CL,
    ) -> Result<Self, VLFSError<F::Error>> {
        todo!()
    }

    pub async fn log(
        &self,
        value: SensorReading<BootTimestamp, D>,
    ) -> Result<(), VLFSError<F::Error>> {
        todo!()
        // let result_1 = self.delta_logger_1.log(value.clone()).await;
        // let result_2 = self.delta_logger_2.log(value).await;
        // result_1?;
        // result_2?;
        // Ok(())
    }

    pub async fn log_unix_time(&self, log: UnixTimestampLog) -> Result<(), VLFSError<F::Error>> {
        todo!()
    }

    pub fn close(&self) {
        todo!()
    }

    pub async fn run(&self) {
        todo!()
    }

    pub fn log_stats(&self) {
        log_info!("Tier 1:");
        // self.delta_logger_1.log_stats();
        log_info!("Tier 2:");
        // self.delta_logger_2.log_stats();
    }
}
