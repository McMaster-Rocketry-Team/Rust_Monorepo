use core::fmt::Display;
use core::fmt::Write;
#[cfg(feature = "log")]
use log::LevelFilter;
use nalgebra::SMatrix;

pub mod plot;

pub fn init_logger() {
    #[cfg(feature = "log")]
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Warn)
        .filter(Some("air_brakes_controller_core"), LevelFilter::Trace)
        .is_test(true)
        .try_init();
}

pub fn to_matlab<T, const R: usize, const C: usize>(m: &SMatrix<T, R, C>) -> String
where
    T: Display, // Display gives plain "1.23", change to LowerExp if needed
{
    let mut s = String::with_capacity(R * C * 8); // crude pre-allocation
    s.push('[');

    for r in 0..R {
        for c in 0..C {
            // ── value ─────────────────────────────
            write!(&mut s, "{}", m[(r, c)]).unwrap();

            // ── column separator ────────────────
            if c < C - 1 {
                s.push(' '); // or ',' if you prefer "1, 2, 3"
            }
        }

        // ── row separator ───────────────────────
        if r < R - 1 {
            s.push_str("; "); // or '\n' for multi-line output
        }
    }

    s.push(']');
    s
}
