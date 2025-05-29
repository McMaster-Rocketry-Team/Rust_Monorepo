use cursive::{Printer, Rect, Vec2, theme::ColorStyle};
use firmware_common_new::can_bus::{
    messages::CanBusMessageEnum, telemetry::message_aggregator::DecodedMessage,
};
use log::warn;

use crate::args::NodeTypeEnum;

use super::{message_row::MessageRow, status_row::NodeStatusRow};

pub struct NodeView {
    node_type: u8,
    node_id: u16,
    status_row: NodeStatusRow,
    messages: Vec<MessageRow>,
}

impl NodeView {
    pub fn new(node_type: u8, node_id: u16) -> Self {
        Self {
            node_type,
            node_id,
            status_row: NodeStatusRow::new(node_type),
            messages: Vec::new(),
        }
    }

    pub fn update(&mut self, message: &DecodedMessage) {
        if message.node_type != self.node_type || message.node_id != self.node_id {
            warn!("node type or ID mismatch");
            return;
        }

        self.status_row.update(&message);

        if let CanBusMessageEnum::NodeStatus(_) = message.message {
            // noop
        } else {
            if let Some(existing_message) = self
                .messages
                .iter_mut()
                .find(|m| m.message.get_message_type() == message.message.get_message_type())
            {
                existing_message.update(message);
            } else {
                self.messages.push(MessageRow::new(
                    message.message.clone(),
                    message.count,
                    self.node_type_enum().background_color(),
                ));
                self.messages.sort_unstable();
            };
        }
    }

    pub fn height(&self) -> usize {
        1 + self.messages.iter().map(|m| m.height()).sum::<usize>()
    }

    pub fn node_type_enum(&self) -> NodeTypeEnum {
        self.node_type.into()
    }

    pub fn draw(&self, printer: &Printer) {
        let bg = self.node_type_enum().background_color();
        printer.with_color(ColorStyle::back(bg), |printer| {
            // fill background color
            printer.print_rect(Rect::from_size(Vec2::zero(), printer.size), " ");

            // node status line
            self.status_row.draw(printer);

            // message lines
            if !self.messages.is_empty() {
                printer.print_vline((0, 1), printer.size.y - 2, "│");
                printer.print((0, printer.size.y - 1), "└─");
            }
            let mut y_offset = 1;
            for message in &self.messages {
                message.draw(&printer.windowed(Rect::from_corners(
                    Vec2::new(3, y_offset),
                    Vec2::new(printer.size.x, y_offset + message.height()),
                )));
                y_offset += message.height();
            }
        });
    }
}

impl PartialEq for NodeView {
    fn eq(&self, other: &Self) -> bool {
        self.node_type_enum() == other.node_type_enum()
    }
}

impl Eq for NodeView {}

impl PartialOrd for NodeView {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for NodeView {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.node_type_enum().cmp(&other.node_type_enum())
    }
}
