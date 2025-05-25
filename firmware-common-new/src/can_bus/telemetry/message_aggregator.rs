use serde::{Deserialize, Serialize};
use heapless::Vec;
use crate::can_bus::messages::CanBusMessageEnum;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct MessageKey {
    node_type: u8,
    node_id: u16,
}

impl MessageKey {
    pub fn new(node_type: u8, node_id: u16) -> Self {
        Self { node_type, node_id }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Serialize, Deserialize)]
struct MessageEntry {
    key: MessageKey,
    message: CanBusMessageEnum,
    count: usize,
    last_received_time: u64,
}

pub struct CanBusMessageAggregator {
    entries: Vec<MessageEntry, 32>,
}

impl CanBusMessageAggregator {
    pub fn new() -> Self {
        Self { entries: Vec::new() }
    }
}