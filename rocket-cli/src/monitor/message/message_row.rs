use core::f32;
use std::{sync::RwLock, time::Instant};

use convert_case::{Case, Casing};
use cursive::{
    Printer, Rect, Vec2,
    theme::{BaseColor, Color, ColorStyle, Style},
    utils::markup::StyledString,
};
use firmware_common_new::can_bus::{
    messages::{
        CanBusMessageEnum,
        amp_overwrite::PowerOutputOverwrite,
        amp_status::{AmpOutputStatus, PowerOutputStatus},
    },
    telemetry::message_aggregator::DecodedMessage,
};
use log::warn;
use pad::{Alignment, PadStr as _};

pub struct FieldWidget {
    name: String,
    value: StyledString,
    last_changed_time: Instant,
    should_highlight: bool,
    bg: Color,
}

impl FieldWidget {
    pub fn new(name: String, value: StyledString, should_highlight: bool, bg: Color) -> Self {
        Self {
            name,
            value,
            last_changed_time: Instant::now(),
            should_highlight,
            bg,
        }
    }

    pub fn update(&mut self, value: StyledString) {
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

    pub fn draw(&self, x_offset: &mut usize, printer: &Printer) {
        let value_bg = if self.should_highlight {
            let change_elapsed_ms = (Instant::now() - self.last_changed_time)
                .as_millis()
                .min(300) as f32;
            self.lerp(Color::Rgb(249, 115, 22), change_elapsed_ms / 300.0)
        } else {
            self.bg
        };

        printer.print_styled(
            (*x_offset, 0),
            &StyledString::single_span(
                &format!("{}: ", self.name),
                Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
            ),
        );
        *x_offset += self.name.len() + 2;

        printer.with_color(ColorStyle::back(value_bg), |printer| {
            printer.print_styled((*x_offset, 0), &self.value);
        });
        *x_offset += self.value.width() + 2;
    }
}

pub struct MessageRow {
    pub message: CanBusMessageEnum,
    count: usize,
    last_received_time: Instant,
    fields_line1: RwLock<Vec<FieldWidget>>,
    fields_line2: RwLock<Vec<FieldWidget>>,
    bg: Color,
}

impl MessageRow {
    pub fn new(message: CanBusMessageEnum, count: usize, bg: Color) -> Self {
        Self {
            message,
            count,
            last_received_time: Instant::now(),
            fields_line1: RwLock::new(vec![]),
            fields_line2: RwLock::new(vec![]),
            bg,
        }
    }

    pub fn update(&mut self, message: &DecodedMessage) {
        if message.message.get_message_type() != self.message.get_message_type() {
            warn!("message type mismatch");
            return;
        }

        self.message = message.message.clone();
        self.count += message.count;
        self.last_received_time = Instant::now();
    }

    pub fn height(&self) -> usize {
        match self.message {
            CanBusMessageEnum::PayloadEPSStatus(_) => 2,
            _ => 1,
        }
    }

    fn message_name(&self) -> &'static str {
        match self.message {
            CanBusMessageEnum::NodeStatus(_) => "Node Status",
            CanBusMessageEnum::Reset(_) => "Reset",
            CanBusMessageEnum::PreUnixTime(_) => "Pre Unix Time",
            CanBusMessageEnum::UnixTime(_) => "Unix Time",
            CanBusMessageEnum::BaroMeasurement(_) => "Baro Measurement",
            CanBusMessageEnum::IMUMeasurement(_) => "IMU Measurement",
            CanBusMessageEnum::MagMeasurement(_) => "Mag Measurement",
            CanBusMessageEnum::BrightnessMeasurement(_) => "Brightness Measurement",
            CanBusMessageEnum::OzysMeasurement(_) => "OZYS Measurement",
            CanBusMessageEnum::AmpStatus(_) => "AMP Status",
            CanBusMessageEnum::AmpOverwrite(_) => "AMP Overwrite",
            CanBusMessageEnum::AmpControl(_) => "AMP Control",
            CanBusMessageEnum::AmpResetOutput(_) => "AMP Reset Output",
            CanBusMessageEnum::PayloadEPSStatus(_) => "EPS Status",
            CanBusMessageEnum::PayloadEPSOutputOverwrite(_) => "EPS Output Overwrite",
            CanBusMessageEnum::VLStatus(_) => "VL Status",
            CanBusMessageEnum::RocketState(_) => "Rocket State",
            CanBusMessageEnum::IcarusStatus(_) => "Icarus Status",
            CanBusMessageEnum::DataTransfer(_) => "Data Transfer",
            CanBusMessageEnum::Ack(_) => "Ack",
        }
    }

    fn draw_fields(&self, printer: &Printer, line: usize, fields: &[(&str, bool, StyledString)]) {
        let mut self_fields = if line == 1 {
            self.fields_line1.write().unwrap()
        } else {
            self.fields_line2.write().unwrap()
        };
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

    fn format_amp_output_status(status: &AmpOutputStatus) -> StyledString {
        let mut s = StyledString::new();

        if status.overwrote {
            s.append_plain("overwrote, ");
        } else {
            s.append_plain("auto, ");
        }

        match status.status {
            PowerOutputStatus::Disabled => s.append_styled(
                "disabled",
                Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
            ),
            PowerOutputStatus::PowerGood => s.append_styled(
                "power good",
                Style::from_color_style(ColorStyle::front(BaseColor::Green.dark())),
            ),
            PowerOutputStatus::PowerBad => s.append_styled(
                "power bad",
                Style::from_color_style(ColorStyle::front(BaseColor::Red.dark())),
            ),
        }

        s.append_plain("".pad_to_width(21 - s.width()));

        s
    }

    fn format_power_output_overwrite(overwrite: PowerOutputOverwrite) -> StyledString {
        let mut s = match overwrite {
            PowerOutputOverwrite::NoOverwrite => StyledString::single_span(
                "no overwrite",
                Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
            ),
            PowerOutputOverwrite::ForceEnabled => StyledString::single_span(
                "force enabled",
                Style::from_color_style(ColorStyle::front(BaseColor::Yellow.dark())),
            ),
            PowerOutputOverwrite::ForceDisabled => StyledString::single_span(
                "force disabled",
                Style::from_color_style(ColorStyle::front(BaseColor::Yellow.dark())),
            ),
        };

        s.append_plain("".pad_to_width(14 - s.width()));
        s
    }

    pub fn draw(&self, printer: &Printer) {
        // max length 22 characters
        printer.print((0, 0), &self.message_name());

        // display each fields
        match &self.message {
            CanBusMessageEnum::Reset(m) => self.draw_fields(
                printer,
                1,
                &[
                    ("reset node id", true, format!("{:0>3X}", m.node_id).into()),
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
                1,
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
                1,
                &[
                    (
                        "pressure",
                        false,
                        format!("{:.1}Pa", m.pressure())
                            .pad_to_width_with_alignment(10, Alignment::Left)
                            .into(),
                    ),
                    (
                        "altitude asl",
                        false,
                        format!("{:.1}m", m.altitude_asl())
                            .pad_to_width_with_alignment(7, Alignment::Left)
                            .into(),
                    ),
                    (
                        "temperature",
                        false,
                        format!("{:.1}C", m.temperature())
                            .pad_to_width_with_alignment(5, Alignment::Left)
                            .into(),
                    ),
                ],
            ),
            CanBusMessageEnum::IMUMeasurement(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "acc (g)",
                        false,
                        format!(
                            "{:>5.2}, {:>5.2}, {:>5.2}",
                            m.acc()[0] / 9.81,
                            m.acc()[1] / 9.81,
                            m.acc()[2] / 9.81
                        )
                        .into(),
                    ),
                    (
                        "gyro (deg/s)",
                        false,
                        format!(
                            "{:>5.1}, {:>5.1}, {:>5.1}",
                            m.gyro()[0],
                            m.gyro()[1],
                            m.gyro()[2]
                        )
                        .into(),
                    ),
                ],
            ),
            CanBusMessageEnum::MagMeasurement(m) => self.draw_fields(
                printer,
                1,
                &[(
                    "mag (T)",
                    false,
                    format!(
                        "{:>5.2}, {:>5.2}, {:>5.2}",
                        m.mag()[0],
                        m.mag()[1],
                        m.mag()[2]
                    )
                    .into(),
                )],
            ),
            CanBusMessageEnum::BrightnessMeasurement(m) => self.draw_fields(
                printer,
                1,
                &[(
                    "brightness (lux)",
                    false,
                    format!("{:>10.2}", m.brightness_lux()).into(),
                )],
            ),
            CanBusMessageEnum::OzysMeasurement(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "sg 1",
                        false,
                        format!("{:>8.5}", m.sg_1().unwrap_or(f32::NAN)).into(),
                    ),
                    (
                        "sg 2",
                        false,
                        format!("{:>8.5}", m.sg_2().unwrap_or(f32::NAN)).into(),
                    ),
                    (
                        "sg 3",
                        false,
                        format!("{:>8.5}", m.sg_3().unwrap_or(f32::NAN)).into(),
                    ),
                    (
                        "sg 4",
                        false,
                        format!("{:>8.5}", m.sg_4().unwrap_or(f32::NAN)).into(),
                    ),
                ],
            ),
            CanBusMessageEnum::AmpStatus(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "shared bat",
                        false,
                        format!("{:.2}V", m.shared_battery_mv as f32 / 1000.0).into(),
                    ),
                    ("out 1", true, Self::format_amp_output_status(&m.out1)),
                    ("out 2", true, Self::format_amp_output_status(&m.out2)),
                    ("out 3", true, Self::format_amp_output_status(&m.out3)),
                    ("out 4", true, Self::format_amp_output_status(&m.out4)),
                ],
            ),
            CanBusMessageEnum::AmpOverwrite(m) => self.draw_fields(
                printer,
                1,
                &[
                    ("out 1", true, Self::format_power_output_overwrite(m.out1)),
                    ("out 2", true, Self::format_power_output_overwrite(m.out2)),
                    ("out 3", true, Self::format_power_output_overwrite(m.out3)),
                    ("out 4", true, Self::format_power_output_overwrite(m.out4)),
                ],
            ),
            CanBusMessageEnum::AmpControl(m) => self.draw_fields(
                printer,
                1,
                &[
                    ("out 1 enable", true, Self::format_bool(m.out1_enable)),
                    ("out 2 enable", true, Self::format_bool(m.out2_enable)),
                    ("out 3 enable", true, Self::format_bool(m.out3_enable)),
                    ("out 4 enable", true, Self::format_bool(m.out4_enable)),
                ],
            ),
            CanBusMessageEnum::AmpResetOutput(m) => self.draw_fields(
                printer,
                1,
                &[("output to reset", true, format!("{}", m.output).into())],
            ),
            CanBusMessageEnum::PayloadEPSStatus(m) => {
                self.draw_fields(
                    printer,
                    1,
                    &[
                        (
                            "bat 1",
                            false,
                            format!(
                                "{:.2}V, {:.1}C",
                                m.battery1_mv as f32 / 1000.0,
                                m.battery1_temperature()
                            )
                            .into(),
                        ),
                        (
                            "bat 2",
                            false,
                            format!(
                                "{:.2}V, {:.1}C",
                                m.battery2_mv as f32 / 1000.0,
                                m.battery2_temperature()
                            )
                            .into(),
                        ),
                    ],
                );
                let printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
                self.draw_fields(
                    &printer,
                    2,
                    &[
                        (
                            "3v3 out current",
                            false,
                            format!("{:>4}mA", m.output_3v3.current_ma).into(),
                        ),
                        (
                            "status",
                            true,
                            Self::format_amp_output_status(&AmpOutputStatus {
                                overwrote: m.output_3v3.overwrote,
                                status: m.output_3v3.status,
                            }),
                        ),
                        (
                            "5v out current",
                            false,
                            format!("{:>4}mA", m.output_5v.current_ma).into(),
                        ),
                        (
                            "status",
                            true,
                            Self::format_amp_output_status(&AmpOutputStatus {
                                overwrote: m.output_5v.overwrote,
                                status: m.output_5v.status,
                            }),
                        ),
                        (
                            "9v out current",
                            false,
                            format!("{:>4}mA", m.output_9v.current_ma).into(),
                        ),
                        (
                            "status",
                            true,
                            Self::format_amp_output_status(&AmpOutputStatus {
                                overwrote: m.output_9v.overwrote,
                                status: m.output_9v.status,
                            }),
                        ),
                    ],
                );
            }
            CanBusMessageEnum::PayloadEPSOutputOverwrite(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "3v3 out",
                        true,
                        Self::format_power_output_overwrite(m.out_3v3),
                    ),
                    (
                        "5v out",
                        true,
                        Self::format_power_output_overwrite(m.out_5v),
                    ),
                    (
                        "9v out",
                        true,
                        Self::format_power_output_overwrite(m.out_9v),
                    ),
                ],
            ),
            CanBusMessageEnum::VLStatus(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "flight stage",
                        true,
                        format!("{:?}", m.flight_stage).to_case(Case::Lower).into(),
                    ),
                    (
                        "battery",
                        false,
                        format!("{:.2}V", m.battery_mv as f32 / 1000.0,).into(),
                    ),
                ],
            ),
            CanBusMessageEnum::RocketState(m) => {
                self.draw_fields(
                    printer,
                    1,
                    &[
                        (
                            "velocity (m/s)",
                            false,
                            format!(
                                "{:>5.2}, {:>5.2}",
                                m.velocity()[0],
                                m.velocity()[1],
                            )
                            .into(),
                        ),
                        (
                            "altitude agl",
                            false,
                            format!("{:.1}m", m.altitude_asl() - m.launch_pad_altitude_asl())
                                .pad_to_width_with_alignment(7, Alignment::Left)
                                .into(),
                        ),
                    ],
                );

                let printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
                self.draw_fields(
                    &printer,
                    2,
                    &[
                        ("Tilt deg", false, format!("{:>5.2}", m.tilt_deg(),).into()),
                        // TODO velocity
                    ],
                );
            }
            CanBusMessageEnum::IcarusStatus(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "commanded extension",
                        false,
                        format!("{:>4.1}%", m.commanded_extension_percentage() * 100.0).into(),
                    ),
                    (
                        "actual extension",
                        false,
                        format!("{:>4.1}%", m.actual_extension_percentage() * 100.0).into(),
                    ),
                    (
                        "servo current",
                        false,
                        format!("{:.2}A", m.servo_current()).into(),
                    ),
                    (
                        "servo speed",
                        false,
                        format!("{:>4}deg/s", m.servo_angular_velocity).into(),
                    ),
                ],
            ),
            CanBusMessageEnum::DataTransfer(m) => self.draw_fields(
                printer,
                1,
                &[
                    (
                        "destination node id",
                        true,
                        format!("{:0>3X}", m.destination_node_id).into(),
                    ),
                    ("data len", false, format!("{:>2}", m.data().len()).into()),
                    ("start", true, Self::format_bool(m.start_of_transfer)),
                    ("end", true, Self::format_bool(m.end_of_transfer)),
                ],
            ),
            CanBusMessageEnum::Ack(m) => self.draw_fields(
                printer,
                1,
                &[
                    ("node id", true, format!("{:0>3X}", m.node_id).into()),
                    ("crc", false, format!("{:0>4X}", m.crc).into()),
                ],
            ),
            CanBusMessageEnum::NodeStatus(_) => unreachable!(),
            CanBusMessageEnum::PreUnixTime(_) => unreachable!(),
        }

        let time_str = format!(
            "{:>5}s ago  x{}",
            (Instant::now() - self.last_received_time).as_secs(),
            self.count
        );
        printer.print((printer.size.x - time_str.len(), 0), &time_str);
    }
}
impl PartialEq for MessageRow {
    fn eq(&self, other: &Self) -> bool {
        self.message == other.message
    }
}

impl Eq for MessageRow {}

impl PartialOrd for MessageRow {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MessageRow {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.message.cmp(&other.message)
    }
}
