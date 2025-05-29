use std::time::Instant;

use cursive::{
    Printer,
    theme::{BaseColor, Color, ColorStyle, Style},
    utils::markup::StyledString,
};
use firmware_common_new::can_bus::{
    messages::{CanBusMessageEnum, node_status::NodeStatusMessage},
    telemetry::message_aggregator::DecodedMessage,
};

use crate::args::NodeTypeEnum;

pub enum NodeStatusRow {
    // never received any message from this node
    Missing {
        node_type: u8,
    },
    // received some message other than node status message from this node
    Unknown {
        node_type: u8,
        node_id: u16,
    },
    // received node status message from this node, based on last_status_received_time,
    // node could be online or offline
    Normal {
        node_type: u8,
        node_id: u16,
        status: NodeStatusMessage,
        last_status_received_time: Instant,
    },
}

impl NodeStatusRow {
    pub fn new(node_type: u8) -> Self {
        Self::Missing { node_type }
    }

    pub fn node_type(&self) -> u8 {
        match self {
            NodeStatusRow::Missing { node_type } => *node_type,
            NodeStatusRow::Unknown { node_type, .. } => *node_type,
            NodeStatusRow::Normal { node_type, .. } => *node_type,
        }
    }

    pub fn node_type_enum(&self) -> NodeTypeEnum {
        self.node_type().into()
    }

    pub fn node_id(&self) -> Option<u16> {
        match self {
            NodeStatusRow::Missing { .. } => None,
            NodeStatusRow::Unknown { node_id, .. } => Some(*node_id),
            NodeStatusRow::Normal { node_id, .. } => Some(*node_id),
        }
    }

    pub fn update(&mut self, message: &DecodedMessage) {
        if message.node_type != self.node_type() {
            panic!("node type mismatch");
        }

        match self {
            NodeStatusRow::Missing { .. } => {
                if let CanBusMessageEnum::NodeStatus(status) = &message.message {
                    *self = NodeStatusRow::Normal {
                        node_type: message.node_type,
                        node_id: message.node_id,
                        status: status.clone(),
                        last_status_received_time: Instant::now(),
                    }
                } else {
                    *self = NodeStatusRow::Unknown {
                        node_type: message.node_type,
                        node_id: message.node_id,
                    }
                }
            }
            NodeStatusRow::Unknown { node_id, .. } => {
                if *node_id != message.node_id {
                    panic!("node id mismatch");
                }
                if let CanBusMessageEnum::NodeStatus(status) = &message.message {
                    *self = NodeStatusRow::Normal {
                        node_type: message.node_type,
                        node_id: message.node_id,
                        status: status.clone(),
                        last_status_received_time: Instant::now(),
                    }
                }
            }
            NodeStatusRow::Normal {
                node_id,
                status,
                last_status_received_time,
                ..
            } => {
                if *node_id != message.node_id {
                    panic!("node id mismatch");
                }
                if let CanBusMessageEnum::NodeStatus(new_status) = &message.message {
                    *status = new_status.clone();
                    *last_status_received_time = Instant::now();
                }
            }
        }
    }

    pub fn draw(&self, printer: &Printer) {
        printer.print((0, 0), &format!("{}", self.node_type_enum()));

        if let Some(node_id) = self.node_id() {
            printer.print((19, 0), &format!("{:0>3X}", node_id));
        }

        let status_str = match self {
            NodeStatusRow::Missing { .. } => StyledString::single_span(
                "missing",
                Style::from_color_style(ColorStyle::front(Color::Rgb(249, 115, 22))),
            ),
            NodeStatusRow::Unknown { .. } => StyledString::single_span(
                "unknown",
                Style::from_color_style(ColorStyle::front(Color::Rgb(249, 115, 22))),
            ),
            NodeStatusRow::Normal {
                status,
                last_status_received_time,
                ..
            } => {
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
            }
        };

        printer.print_styled((printer.size.x - status_str.width(), 0), &status_str);
    }
}
