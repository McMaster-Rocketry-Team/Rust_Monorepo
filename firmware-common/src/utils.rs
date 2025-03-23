use core::future::Future;

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_async::delay::DelayNs;

use crate::driver::clock::Clock;

#[macro_export]
macro_rules! try_or_warn {
    ($e: expr) => {{
        if let Err(e) = $e {
            log_warn!("`{}` failed: {:?}", stringify!($e), e);
        }
    }};
}

pub async fn run_with_timeout<F: Future>(
    delay: &mut impl DelayNs,
    ms: f64,
    future: F,
) -> Result<F::Output, f64> {
    let timeout_fut = delay.delay_us((ms * 1_000.0) as u32);
    match select(timeout_fut, future).await {
        Either::First(_) => Err(ms),
        Either::Second(result) => Ok(result),
    }
}

pub struct Debouncer<M: RawMutex, V: Send, C: Clock, D: DelayNs + Copy> {
    signal: Signal<M, (V, f64)>,
    duration_ms: f64,
    clock: C,
    delay: D,
}

impl<M: RawMutex, V: Send, C: Clock, D: DelayNs + Copy> Debouncer<M, V, C, D> {
    pub fn new(duration_ms: f64, clock: C, delay: D) -> Self {
        Self {
            signal: Signal::new(),
            duration_ms,
            clock,
            delay,
        }
    }

    pub fn put(&self, value: V) {
        self.signal.signal((value, self.clock.now_ms()));
    }

    pub async fn wait_next(&self) -> V {
        let mut value = self.signal.wait().await;
        if value.1 + self.duration_ms < self.clock.now_ms() {
            return value.0;
        }
        loop {
            let mut delay = self.delay;
            match run_with_timeout(
                &mut delay,
                self.duration_ms - (self.clock.now_ms() - value.1),
                self.signal.wait(),
            )
            .await
            {
                Ok(v) => {
                    value = v;
                }
                Err(_) => {
                    return value.0;
                }
            }
        }
    }
}
