mod list_files;
mod probe_device_type;
pub(crate) mod pull_delta_readings;
mod pull_file;
pub(crate) mod pull_serialized_enums;
pub(crate) mod readers;
pub(crate) mod sensor_reading_csv_writer;
pub(crate) mod unix_timestamp_lut;

use std::path::PathBuf;

pub use list_files::list_files;
pub use probe_device_type::probe_device_type;
pub use pull_file::pull_file;

pub fn extend_path(path: &PathBuf, extend: &str) -> PathBuf {
    let mut path = path.clone();
    path.push(extend);
    path
}