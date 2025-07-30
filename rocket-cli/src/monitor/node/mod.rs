use std::sync::{Arc, RwLock};

use cursive::{
    Rect, Vec2, View,
    theme::ColorStyle,
    view::{Finder as _, Nameable as _, Scrollable as _, ViewWrapper},
    views::{BoxedView, ScrollView},
    wrap_impl,
};
use firmware_common_new::can_bus::{node_types::*, telemetry::message_aggregator::DecodedMessage};
use tokio::sync::broadcast;

use super::message::status_row::NodeStatusRow;

pub struct NodeStatusViewer {
    root: ScrollView<BoxedView>,
    messages_rx: Arc<RwLock<broadcast::Receiver<DecodedMessage>>>,
}

impl NodeStatusViewer {
    pub fn new(messages_rx: broadcast::Receiver<DecodedMessage>) -> Self {
        let messages_rx = Arc::new(RwLock::new(messages_rx));
        let root =
            BoxedView::boxed(NodeStatusViewerChild::new().with_name("node_status_viewer_child"))
                .scrollable();
        Self { root, messages_rx }
    }

    pub fn receive_messages(&mut self) {
        let mut can_message_viewer = self
            .root
            .find_name::<NodeStatusViewerChild>("node_status_viewer_child")
            .unwrap();

        let mut messages_rx = self.messages_rx.write().unwrap();
        while let Ok(message) = messages_rx.try_recv() {
            can_message_viewer.update(&message);
        }
    }
}

impl ViewWrapper for NodeStatusViewer {
    wrap_impl!(self.root: ScrollView<BoxedView>);
}

struct NodeStatusViewerChild {
    nodes: Vec<NodeStatusRow>,
}

impl NodeStatusViewerChild {
    fn new() -> Self {
        Self {
            nodes: vec![
                NodeStatusRow::new(VOID_LAKE_NODE_TYPE),
                NodeStatusRow::new(AMP_NODE_TYPE),
                NodeStatusRow::new(ICARUS_NODE_TYPE),
                NodeStatusRow::new(PAYLOAD_ACTIVATION_NODE_TYPE),
                NodeStatusRow::new(PAYLOAD_ROCKET_WIFI_NODE_TYPE),
                NodeStatusRow::new(OZYS_NODE_TYPE),
                NodeStatusRow::new(OZYS_NODE_TYPE),
                NodeStatusRow::new(BULKHEAD_NODE_TYPE),
                NodeStatusRow::new(BULKHEAD_NODE_TYPE),
                NodeStatusRow::new(PAYLOAD_EPS1_NODE_TYPE),
                NodeStatusRow::new(PAYLOAD_EPS2_NODE_TYPE),
                NodeStatusRow::new(AERO_RUST_NODE_TYPE),
            ],
        }
    }

    fn update(&mut self, message: &DecodedMessage) {
        if let Some(node) = self
            .nodes
            .iter_mut()
            .find(|n| n.node_id() == Some(message.node_id))
        {
            node.update(message);
            return;
        }

        if let Some(node) = self
            .nodes
            .iter_mut()
            .find(|n| n.node_id().is_none() && n.node_type() == message.node_type)
        {
            node.update(message);
            return;
        }

        let mut node = NodeStatusRow::new(message.node_type);
        node.update(message);
        self.nodes.push(node);
    }
}

impl View for NodeStatusViewerChild {
    fn draw(&self, printer: &cursive::Printer) {
        for (i, node) in self.nodes.iter().enumerate() {
            let bg = node.node_type_enum().background_color();
            printer.with_color(ColorStyle::back(bg), |printer| {
                let printer = printer.windowed(Rect::from_size(Vec2::new(0, i), (printer.size.x, 1)));
                printer.print_hline(Vec2::zero(), printer.size.x, " ");
                node.draw(&printer);
            });
        }
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        Vec2::new(constraint.x, self.nodes.len())
    }
}
