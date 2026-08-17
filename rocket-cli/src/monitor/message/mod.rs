use std::sync::{Arc, RwLock};

use cursive::{
    Printer, Rect, Vec2, View,
    view::{Finder, Nameable, Scrollable, ViewWrapper},
    views::{BoxedView, ScrollView},
    wrap_impl,
};
use firmware_common_new::can_bus::telemetry::message_aggregator::DecodedMessage;
use node_view::NodeView;
use tokio::sync::broadcast;

pub mod message_row;
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

    /// Drain this frame's messages.
    ///
    /// A `Lagged` here heals by itself — the loop is re-entered next frame —
    /// but the per-message `x{count}` totals each row shows are cumulative, so
    /// silently skipping a lag makes every one of them under-report by the
    /// number of messages that never arrived. Count them separately and show
    /// the total, so the row counts can be read as "at least this many" with a
    /// known shortfall rather than as a number that is simply wrong.
    pub fn receive_messages(&mut self) {
        let mut can_message_viewer = self
            .root
            .find_name::<CanMessageViewerChild>("can_message_viewer_child")
            .unwrap();

        let mut messages_rx = self.messages_rx.write().unwrap();
        loop {
            match messages_rx.try_recv() {
                Ok(message) => can_message_viewer.update(&message),
                Err(broadcast::error::TryRecvError::Lagged(dropped)) => {
                    can_message_viewer.dropped_messages += dropped;
                }
                Err(broadcast::error::TryRecvError::Empty)
                | Err(broadcast::error::TryRecvError::Closed) => break,
            }
        }
    }
}

impl ViewWrapper for CanMessageViewer {
    wrap_impl!(self.root: ScrollView<BoxedView>);
}

struct CanMessageViewerChild {
    nodes: Vec<NodeView>,
    /// Messages the broadcast channel dropped before this viewer could read
    /// them. Not attributable to any one row — the channel does not say what it
    /// threw away — so it is shown once, at the top, rather than folded into a
    /// row's count where it would look like a message that was actually seen.
    dropped_messages: u64,
}

impl CanMessageViewerChild {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            dropped_messages: 0,
        }
    }

    /// One extra line at the top once anything has been dropped.
    fn header_height(&self) -> usize {
        if self.dropped_messages > 0 { 1 } else { 0 }
    }

    fn update(&mut self, message: &DecodedMessage) {
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
        if self.dropped_messages > 0 {
            printer.print(
                (0, 0),
                &format!(
                    "{} message(s) dropped — the per-message counts below under-report by \
                     that much",
                    self.dropped_messages
                ),
            );
            y_offset += 1;
        }
        for node in &self.nodes {
            node.draw(&printer.windowed(Rect::from_size(
                Vec2::new(0, y_offset),
                (printer.size.x, node.height()),
            )));
            y_offset += node.height();
        }
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        let h = self.header_height() + self.nodes.iter().map(|n| n.height()).sum::<usize>();
        Vec2::new(constraint.x, h)
    }
}
