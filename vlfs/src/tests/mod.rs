#[cfg(feature = "log")]
use log::LevelFilter;

mod utils;
mod harness;
mod debug_flash;
mod functional;
mod concurrent_files_iter;

fn init_logger() {
    #[cfg(feature = "log")]
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Error)
        .filter(Some("vlfs"), LevelFilter::Trace)
        .filter(Some("vlfs_host"), LevelFilter::Trace)
        .is_test(true)
        .try_init();
}