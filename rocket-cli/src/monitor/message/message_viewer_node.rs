use std::time::Instant;

use cursive::{
    Printer, Rect, Vec2,
    theme::{BaseColor, Color, ColorStyle, Style},
    utils::markup::StyledString,
};
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
    messages: Vec<CanMessageViewerMessage>,
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
            printer.print((0, 0), &format!("{}", self.node_type_enum()));

            printer.print((19, 0), &format!("{:0>3X}", self.node_id));

            let status_str = if let Some((status, last_status_received_time)) = &self.status {
                printer.print((26, 0), &format!("{:?}", status.health));
                printer.print((35, 0), &format!("{:?}", status.mode));

                let last_status_elapsed_s = last_status_received_time.elapsed().as_secs();
                if last_status_elapsed_s < 5 {
                    // online
                    StyledString::single_span(
                        &format!("up {}s", status.uptime_s),
                        Style::from_color_style(ColorStyle::front(BaseColor::Green.dark())),
                    )
                } else {
                    // offline
                    StyledString::single_span(
                        &format!("offline {}s", last_status_elapsed_s),
                        Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
                    )
                }
            } else {
                StyledString::single_span(
                    "unknown status",
                    Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
                )
            };

            printer.print_styled((printer.size.x - status_str.width(), 0), &status_str);

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
