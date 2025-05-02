#[cfg(feature = "log")]
use log::LevelFilter;

pub fn init_logger() {
    #[cfg(feature = "log")]
    let _ = env_logger::builder()
        .filter_level(LevelFilter::Warn)
        .filter(Some("firmware_common_new"), LevelFilter::Trace)
        .is_test(true)
        .try_init();
}
