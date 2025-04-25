pub fn timestamp_to_residue(timestamp_ms: f64) -> u16 {
    let residue = (timestamp_ms as u64) % 1000;
    residue as u16
}

pub fn residue_to_timestamp(
    last_unix_time_message_received_boot_timestamp: f64,
    last_unix_time_message_timestamp: u64,
    residue_message_received_boot_timestamp: f64,
    residue: u16,
) -> f64 {
    todo!()
}
