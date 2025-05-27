use std::{sync::RwLock, time::Instant};

use cursive::{
    Printer,
    theme::{Color, ColorStyle},
    utils::markup::StyledString,
};
use firmware_common_new::can_bus::{
    messages::CanBusMessageEnum, telemetry::message_aggregator::DecodedMessage,
};
use pad::{Alignment, PadStr as _};

struct FieldWidget {
    name: String,
    value: StyledString,
    last_changed_time: Instant,
    should_highlight: bool,
    bg: Color,
}

impl FieldWidget {
    fn new(name: String, value: StyledString, should_highlight: bool, bg: Color) -> Self {
        Self {
            name,
            value,
            last_changed_time: Instant::now(),
            should_highlight,
            bg,
        }
    }

    fn update(&mut self, value: StyledString) {
        if value != self.value {
            self.value = value;
            self.last_changed_time = Instant::now();
        }
    }

    fn lerp(&self, color: Color, t: f32) -> Color {
        match (color, self.bg) {
            (Color::Rgb(r1, g1, b1), Color::Rgb(r2, g2, b2)) => {
                let r = ((r1 as f32 * (1.0 - t) + r2 as f32 * t).round()) as u8;
                let g = ((g1 as f32 * (1.0 - t) + g2 as f32 * t).round()) as u8;
                let b = ((b1 as f32 * (1.0 - t) + b2 as f32 * t).round()) as u8;
                Color::Rgb(r, g, b)
            }
            _ => unimplemented!(),
        }
    }

    fn draw(&self, x_offset: &mut usize, printer: &Printer) {
        let value_bg = if self.should_highlight {
            let change_elapsed_ms = (Instant::now() - self.last_changed_time)
                .as_millis()
                .min(300) as f32;
            self.lerp(Color::Rgb(249, 115, 22), change_elapsed_ms / 300.0)
        } else {
            self.bg
        };

        printer.print((*x_offset, 0), &self.name);
        printer.print((*x_offset + 2, 0), ": ");
        *x_offset += self.name.len() + 2;

        printer.with_color(ColorStyle::back(value_bg), |printer| {
            printer.print_styled((*x_offset, 0), &self.value);
        });
        *x_offset += self.value.width() + 2;
    }
}

pub struct CanMessageViewerMessage {
    pub message: CanBusMessageEnum,
    count: usize,
    last_received_time: Instant,
    fields: RwLock<Vec<FieldWidget>>,
    bg: Color,
}

impl CanMessageViewerMessage {
    pub fn new(message: CanBusMessageEnum, count: usize, bg: Color) -> Self {
        Self {
            message,
            count,
            last_received_time: Instant::now(),
            fields: RwLock::new(Vec::new()),
            bg,
        }
    }

    pub fn update(&mut self, message: DecodedMessage) {
        if message.message.get_message_type() != self.message.get_message_type() {
            panic!("message type mismatch")
        }

        self.message = message.message;
        self.count += message.count;
        self.last_received_time = Instant::now();
    }

    fn message_name(&self) -> &'static str {
        match self.message {
            CanBusMessageEnum::NodeStatus(_) => "Node Status",
            CanBusMessageEnum::Reset(_) => "Reset",
            CanBusMessageEnum::PreUnixTime(_) => "Pre Unix Time",
            CanBusMessageEnum::UnixTime(_) => "Unix Time",
            CanBusMessageEnum::BaroMeasurement(_) => "Baro Measurement",
            CanBusMessageEnum::IMUMeasurement(_) => "IMU Measurement",
            CanBusMessageEnum::BrightnessMeasurement(_) => "Brightness Measurement",
            CanBusMessageEnum::AmpStatus(_) => "AMP Status",
            CanBusMessageEnum::AmpOverwrite(_) => "AMP Overwrite",
            CanBusMessageEnum::AmpControl(_) => "AMP Control",
            CanBusMessageEnum::PayloadEPSStatus(_) => "EPS Status",
            CanBusMessageEnum::PayloadEPSOutputOverwrite(_) => "EPS Output Overwrite",
            CanBusMessageEnum::PayloadEPSSelfTest(_) => "EPS Self Test",
            CanBusMessageEnum::AvionicsStatus(_) => "Avionics Status",
            CanBusMessageEnum::IcarusStatus(_) => "Icarus Status",
            CanBusMessageEnum::DataTransfer(_) => "Data Transfer",
            CanBusMessageEnum::Ack(_) => "Ack",
        }
    }

    fn draw_fields(&self, printer: &Printer, fields: &[(&str, bool, StyledString)]) {
        let mut self_fields = self.fields.write().unwrap();
        if self_fields.is_empty() {
            for field in fields {
                self_fields.push(FieldWidget::new(
                    field.0.into(),
                    field.2.clone(),
                    field.1,
                    self.bg,
                ));
            }
        } else {
            for (i, field) in fields.into_iter().enumerate() {
                self_fields[i].update(field.2.clone());
            }
        }

        let mut x_offset = 23usize;
        for field in self_fields.iter() {
            field.draw(&mut x_offset, printer);
        }
    }

    fn format_bool(value: bool) -> StyledString {
        let s = if value { "T" } else { "F" };
        String::from(s).into()
    }

    fn draw(&self, printer: &Printer) {
        // max length 22 characters
        printer.print((0, 0), &self.message_name());

        // display each fields
        match &self.message {
            CanBusMessageEnum::Reset(m) => self.draw_fields(
                printer,
                &[
                    (
                        "reset node id",
                        true,
                        format!("{:X}", m.node_id)
                            .pad(3, '0', pad::Alignment::Right, false)
                            .into(),
                    ),
                    (
                        "into bootloader",
                        true,
                        Self::format_bool(m.into_bootloader),
                    ),
                    ("reset all", true, Self::format_bool(m.reset_all)),
                ],
            ),
            CanBusMessageEnum::UnixTime(m) => self.draw_fields(
                printer,
                &[
                    ("timestamp us", false, m.timestamp_us.to_string().into()),
                    (
                        "formatted",
                        false,
                        chrono::DateTime::from_timestamp((m.timestamp_us / 1_000_000) as i64, 0)
                            .map_or("invalid time".into(), |dt| {
                                dt.format("%Y/%m/%d %H:%M:%S").to_string().into()
                            }),
                    ),
                ],
            ),
            CanBusMessageEnum::BaroMeasurement(m) => self.draw_fields(
                printer,
                &[(
                    "pressure",
                    false,
                    // TODO
                    format!("{}", m.pressure())
                        .pad(3, '0', pad::Alignment::Right, false)
                        .into(),
                )],
            ),
            CanBusMessageEnum::NodeStatus(_) => unreachable!(),
            CanBusMessageEnum::PreUnixTime(_) => unreachable!(),
            _ => todo!(),
        }

        let count_str = format!("x{}", self.count).pad_to_width(6);
        printer.print((printer.size.x - 6, 0), &count_str);

        let time_str = format!(
            "{}s ago",
            (Instant::now() - self.last_received_time).as_secs()
        )
        .pad_to_width_with_alignment(10, Alignment::Right);
        printer.print((printer.size.x - 17, 0), &time_str);
    }
}
impl PartialEq for CanMessageViewerMessage {
    fn eq(&self, other: &Self) -> bool {
        self.message == other.message
    }
}

impl Eq for CanMessageViewerMessage {}

impl PartialOrd for CanMessageViewerMessage {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CanMessageViewerMessage {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.message.cmp(&other.message)
    }
}
