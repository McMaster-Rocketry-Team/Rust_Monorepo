use std::sync::{Arc, RwLock};

use cursive::{
    Printer, Rect, Vec2, View,
    view::{Finder, Nameable, Scrollable, ViewWrapper},
    views::{BoxedView, ScrollView},
    wrap_impl,
};
use firmware_common_new::can_bus::{
    messages::CanBusMessageEnum, telemetry::message_aggregator::DecodedMessage,
};
use node_view::NodeView;
use tokio::sync::broadcast;

mod message_row;
pub mod message_saver;
mod node_view;
pub mod status_row;

pub struct CanMessageViewer {
    root: ScrollView<BoxedView>,
    messages_rx: Arc<RwLock<broadcast::Receiver<DecodedMessage>>>,
}

impl CanMessageViewer {
    pub fn new(messages_rx: broadcast::Receiver<DecodedMessage>) -> Self {
        let messages_rx = Arc::new(RwLock::new(messages_rx));
        let root =
            BoxedView::boxed(CanMessageViewerChild::new().with_name("can_message_viewer_child"))
                .scrollable();
        Self { root, messages_rx }
    }

    pub fn receive_messages(&mut self) {
        let mut can_message_viewer = self
            .root
            .find_name::<CanMessageViewerChild>("can_message_viewer_child")
            .unwrap();

        let mut messages_rx = self.messages_rx.write().unwrap();
        while let Ok(message) = messages_rx.try_recv() {
            can_message_viewer.update(&message);
        }
    }
}

impl ViewWrapper for CanMessageViewer {
    wrap_impl!(self.root: ScrollView<BoxedView>);
}

struct CanMessageViewerChild {
    nodes: Vec<NodeView>,
}

impl CanMessageViewerChild {
    fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    fn update(&mut self, message: &DecodedMessage) {
        if let CanBusMessageEnum::PreUnixTime(_) = message.message {
            return;
        }

        if let Some(node) = self
            .nodes
            .iter_mut()
            .find(|n| n.node_type_enum() == message.node_type.into())
        {
            node.update(&message);
        } else {
            let mut node = NodeView::new(message.node_type, message.node_id);
            node.update(&message);
            self.nodes.push(node);
            self.nodes.sort_unstable();
        }
    }
}

impl View for CanMessageViewerChild {
    fn draw(&self, printer: &Printer) {
        let mut y_offset = 0;
        for node in &self.nodes {
            node.draw(&printer.windowed(Rect::from_size(
                Vec2::new(0, y_offset),
                (printer.size.x, node.height()),
            )));
            y_offset += node.height();
        }
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        let h = self.nodes.iter().map(|n| n.height()).sum();
        Vec2::new(constraint.x, h)
    }
}
