use std::time::Instant;

use firmware_common_new::can_bus::{
    messages::{CanBusMessageEnum, node_status::NodeStatusMessage},
    telemetry::message_aggregator::DecodedMessage,
};

use crate::args::NodeTypeEnum;

use super::message_viewer_message::CanMessageViewerMessage;

pub struct CanMessageViewerNode {
    node_type: u8,
    node_id: u16,
    status: Option<(NodeStatusMessage, Instant)>,
    pub messages: Vec<CanMessageViewerMessage>,
}

impl CanMessageViewerNode {
    pub fn new(node_type: u8, node_id: u16) -> Self {
        Self {
            node_type,
            node_id,
            status: None,
            messages: Vec::new(),
        }
    }

    pub fn update(&mut self, message: DecodedMessage) {
        if message.node_type != self.node_type || message.node_id != self.node_id {
            panic!("node type or ID mismatch");
        }

        if let CanBusMessageEnum::NodeStatus(status) = message.message {
            self.status = Some((status, Instant::now()));
        } else {
            if let Some(existing_message) = self
                .messages
                .iter_mut()
                .find(|m| m.message.get_message_type() == message.message.get_message_type())
            {
                existing_message.update(message);
            } else {
                self.messages.push(CanMessageViewerMessage::new(
                    message.message,
                    message.count,
                    self.node_type_enum().background_color(),
                ));
                self.messages.sort_unstable();
            };
        }
    }

    pub fn node_type_enum(&self) -> NodeTypeEnum {
        self.node_type.into()
    }
}

impl PartialEq for CanMessageViewerNode {
    fn eq(&self, other: &Self) -> bool {
        self.node_type_enum() == other.node_type_enum()
    }
}

impl Eq for CanMessageViewerNode {}

impl PartialOrd for CanMessageViewerNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CanMessageViewerNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.node_type_enum().cmp(&other.node_type_enum())
    }
}
