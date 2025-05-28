use std::sync::{Arc, RwLock};

use cursive::{Printer, Vec2, View, view::ViewWrapper, views::LinearLayout, wrap_impl};
use firmware_common_new::can_bus::{
    messages::CanBusMessageEnum, telemetry::message_aggregator::DecodedMessage,
};
use message_viewer_node::CanMessageViewerNode;
use tokio::sync::broadcast;

pub mod message_saver;
mod message_viewer_message;
mod message_viewer_node;

pub struct CanMessageViewer {
    root: LinearLayout,
    messages_rx: Arc<RwLock<broadcast::Receiver<DecodedMessage>>>,
}

impl CanMessageViewer {
    pub fn new(messages_rx: broadcast::Receiver<DecodedMessage>) -> Self {
        let messages_rx = Arc::new(RwLock::new(messages_rx));
        let root = LinearLayout::vertical();
        Self { root, messages_rx }
    }
}

impl ViewWrapper for CanMessageViewer {
    wrap_impl!(self.root: LinearLayout);
}

struct CanMessageViewerChild {
    nodes: Vec<CanMessageViewerNode>,
}

impl CanMessageViewerChild {
    fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    fn update(&mut self, message: DecodedMessage) {
        if let CanBusMessageEnum::PreUnixTime(_) = message.message {
            return;
        }

        if let Some(node) = self
            .nodes
            .iter_mut()
            .find(|n| n.node_type_enum() == message.node_type.into())
        {
            node.update(message);
        } else {
            let mut node = CanMessageViewerNode::new(message.node_type, message.node_id);
            node.update(message);
            self.nodes.push(node);
            self.nodes.sort_unstable();
        }
    }
}

impl View for CanMessageViewerChild {
    fn draw(&self, printer: &Printer) {
        todo!()
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        let h = self.nodes.iter().map(|n| n.height()).sum();
        Vec2::new(constraint.x, h)
    }
}
