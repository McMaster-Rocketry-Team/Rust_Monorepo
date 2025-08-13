use std::{sync::RwLock, time::Instant};

use cursive::{
    Printer, Rect, Vec2, View,
    theme::{BaseColor, Color, ColorStyle, Style},
    utils::markup::StyledString,
};
use firmware_common_new::{
    can_bus::messages::amp_status::PowerOutputStatus, vlp::packets::VLPDownlinkPacket,
};
use lora_phy::mod_params::PacketStatus;
use pad::PadStr as _;

use crate::monitor::FieldWidget;

struct Packet {
    packet: VLPDownlinkPacket,
    status: PacketStatus,
    received_time: Instant,
}

pub struct DownlinkPacketDisplay {
    packet: Option<Packet>,
    fields: RwLock<Vec<Vec<FieldWidget>>>,
}

impl DownlinkPacketDisplay {
    pub fn new() -> Self {
        Self {
            packet: None,
            fields: RwLock::new(vec![]),
        }
    }

    pub fn update(&mut self, packet: VLPDownlinkPacket, status: PacketStatus) {
        if let Some(Packet {
            packet: old_packet, ..
        }) = &self.packet
            && old_packet.packet_type() != packet.packet_type()
        {
            let mut fields = self.fields.write().unwrap();
            fields.clear();
        }

        self.packet = Some(Packet {
            packet,
            status,
            received_time: Instant::now(),
        });
    }

    fn packet_name(&self) -> &'static str {
        if let Some(Packet { packet, .. }) = &self.packet {
            match packet {
                VLPDownlinkPacket::GPSBeacon(_) => "GPS Beacon",
                VLPDownlinkPacket::Ack(_) => "Ack",
                VLPDownlinkPacket::LowPowerTelemetry(_) => "Low Power Telemetry",
                VLPDownlinkPacket::Telemetry(_) => "Telemetry",
                VLPDownlinkPacket::SelfTestResult(_) => "Self Test Result",
                VLPDownlinkPacket::LandedTelemetry(_) => "Landed Telemetry",
            }
        } else {
            ""
        }
    }

    fn format_bool(value: bool) -> StyledString {
        let s = if value { "T" } else { "F" };
        String::from(s).into()
    }

    fn format_amp_output_status(overwrote: bool, status: PowerOutputStatus) -> StyledString {
        let mut s = StyledString::new();

        if overwrote {
            s.append_plain("overwrote, ");
        } else {
            s.append_plain("auto, ");
        }

        match status {
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

    fn draw_fields(&self, printer: &Printer, fields: &[&[(&str, bool, StyledString)]]) {
        let mut self_fields = self.fields.write().unwrap();

        if self_fields.is_empty() {
            *self_fields = fields
                .into_iter()
                .map(|line| {
                    line.into_iter()
                        .map(|field| {
                            FieldWidget::new(
                                field.0.into(),
                                field.2.clone(),
                                field.1,
                                Color::Rgb(248, 248, 248),
                            )
                        })
                        .collect()
                })
                .collect();
        } else {
            let self_fields_iter = self_fields
                .iter_mut()
                .map(|self_fields_line| self_fields_line.iter_mut())
                .flatten();
            let fields_iter = fields.iter().map(|line| line.iter()).flatten();
            for (field_widget, field) in self_fields_iter.zip(fields_iter) {
                field_widget.update(field.2.clone());
            }
        }

        let mut printer = printer.clone();
        for self_fields_line in self_fields.iter() {
            let mut x_offset = 0usize;
            for field in self_fields_line {
                field.draw(&mut x_offset, &printer);
            }

            printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
        }
    }
}

impl View for DownlinkPacketDisplay {
    fn draw(&self, printer: &Printer) {
        if let Some(Packet {
            packet,
            status,
            received_time,
        }) = &self.packet
        {
            printer.print(
                (0, 0),
                &format!(
                    "{} rssi: {} snr: {}",
                    self.packet_name(),
                    status.rssi,
                    status.snr
                ),
            );

            let time_str = format!(
                "{:>5}s ago",
                (Instant::now() - received_time.clone()).as_secs(),
            );
            printer.print((printer.size.x - time_str.len(), 0), &time_str);

            let printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
            match packet {
                VLPDownlinkPacket::GPSBeacon(p) => self.draw_fields(
                    &printer,
                    &[
                        &[
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, p.lat().to_string().into()),
                            ("lon", false, p.lon().to_string().into()),
                        ],
                        &[
                            (
                                "altitude asl",
                                false,
                                format!("{:.1}m", p.altitude_asl()).into(),
                            ),
                            (
                                "air temperature",
                                false,
                                format!("{:.1}C", p.air_temperature()).into(),
                            ),
                        ],
                        &[("vl battery", false, format!("{:.2}V", p.battery_v()).into())],
                        &[(
                            "pyro short circuit",
                            true,
                            Self::format_bool(p.pyro_short_circuit),
                        )],
                        &[
                            (
                                "main continuity",
                                true,
                                Self::format_bool(p.pyro_main_continuity),
                            ),
                            ("main fire", true, Self::format_bool(p.pyro_main_fire)),
                        ],
                        &[
                            (
                                "drogue continuity",
                                true,
                                Self::format_bool(p.pyro_drogue_continuity),
                            ),
                            ("drogue fire", true, Self::format_bool(p.pyro_drogue_fire)),
                        ],
                    ],
                ),
                VLPDownlinkPacket::LowPowerTelemetry(p) => self.draw_fields(
                    &printer,
                    &[
                        &[
                            ("gps fixed", true, Self::format_bool(p.gps_fixed)),
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                        ],
                        &[(
                            "air temperature",
                            false,
                            format!("{:.1}C", p.air_temperature()).into(),
                        )],
                        &[
                            (
                                "vl battery",
                                false,
                                format!("{:.2}V", p.vl_battery_v()).into(),
                            ),
                            (
                                "shared battery",
                                false,
                                format!("{:.2}V", p.shared_battery_v()).into(),
                            ),
                        ],
                        &[("amp online", true, Self::format_bool(p.amp_online))],
                    ],
                ),
                VLPDownlinkPacket::LandedTelemetry(p) => self.draw_fields(
                    &printer,
                    &[
                        &[
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, p.lat().to_string().into()),
                            ("lon", false, p.lon().to_string().into()),
                        ],
                        &[
                            ("vl battery", false, format!("{:.2}V", p.battery_v()).into()),
                            (
                                "shared battery",
                                false,
                                format!("{:.2}V", p.shared_battery_v()).into(),
                            ),
                        ],
                        &[
                            ("amp online", true, Self::format_bool(p.amp_online())),
                            (
                                "amp rebooted",
                                true,
                                Self::format_bool(p.amp_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            (
                                "amp out 1",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out1_overwrote(),
                                    p.amp_out1(),
                                ),
                            ),
                            (
                                "amp out 2",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out2_overwrote(),
                                    p.amp_out2(),
                                ),
                            ),
                            (
                                "amp out 3",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out3_overwrote(),
                                    p.amp_out3(),
                                ),
                            ),
                            (
                                "amp out 4",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out4_overwrote(),
                                    p.amp_out4(),
                                ),
                            ),
                        ],
                    ],
                ),
                VLPDownlinkPacket::Telemetry(p) => self.draw_fields(
                    &printer,
                    &[
                        &[
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("unix clock", true, Self::format_bool(p.unix_clock_ready())),
                            ("lat", false, p.lat().to_string().into()),
                            ("lon", false, p.lon().to_string().into()),
                        ],
                        &[
                            (
                                "vl battery",
                                false,
                                format!("{:.2}V", p.vl_battery_v()).into(),
                            ),
                            (
                                "shared battery",
                                false,
                                format!("{:.2}V", p.shared_battery_v()).into(),
                            ),
                            (
                                "main continuity",
                                true,
                                Self::format_bool(p.pyro_main_continuity()),
                            ),
                            (
                                "drogue continuity",
                                true,
                                Self::format_bool(p.pyro_drogue_continuity()),
                            ),
                        ],
                        &[
                            (
                                "air temperature",
                                false,
                                format!("{:.1}C", p.air_temperature()).into(),
                            ),
                            (
                                "servo temp",
                                false,
                                format!("{:.1}C", p.air_brakes_servo_temp()).into(),
                            ),
                        ],
                        &[
                            ("state", true, format!("{:?}", p.flight_stage()).into()),
                            (
                                "altitude agl",
                                false,
                                format!("{:.1}m", p.altitude_agl()).into(),
                            ),
                            (
                                "max altitude agl",
                                false,
                                format!("{:.1}C", p.max_altitude_agl()).into(),
                            ),
                            (
                                "air speed",
                                false,
                                format!("{:.1}m/s", p.air_speed()).into(),
                            ),
                            (
                                "max air speed",
                                false,
                                format!("{:.1}m/s", p.max_air_speed()).into(),
                            ),
                            ("tilt", false, format!("{:.1}deg", p.tilt_deg()).into()),
                        ],
                        &[
                            ("icarus online", true, Self::format_bool(p.icarus_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.icarus_rebooted_in_last_5s()),
                            ),
                            (
                                "commanded extension",
                                false,
                                format!(
                                    "{}%",
                                    (p.air_brakes_commanded_extension_percentage() * 100.0).round()
                                )
                                .into(),
                            ),
                            (
                                "actual extension",
                                false,
                                format!(
                                    "{}%",
                                    (p.air_brakes_actual_extension_percentage() * 100.0).round()
                                )
                                .into(),
                            ),
                        ],
                        &[
                            ("amp online", true, Self::format_bool(p.amp_online())),
                            (
                                "amp rebooted",
                                true,
                                Self::format_bool(p.amp_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            (
                                "amp out 1",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out1_overwrote(),
                                    p.amp_out1(),
                                ),
                            ),
                            (
                                "amp out 2",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out2_overwrote(),
                                    p.amp_out2(),
                                ),
                            ),
                            (
                                "amp out 3",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out3_overwrote(),
                                    p.amp_out3(),
                                ),
                            ),
                            (
                                "amp out 4",
                                true,
                                Self::format_amp_output_status(
                                    p.amp_out4_overwrote(),
                                    p.amp_out4(),
                                ),
                            ),
                        ],
                        &[
                            (
                                "main bulkhead online",
                                true,
                                Self::format_bool(p.main_bulkhead_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.main_bulkhead_rebooted_in_last_5s()),
                            ),
                            (
                                "brightness",
                                true,
                                format!("{:.2}lux", p.main_bulkhead_brightness_lux()).into(),
                            ),
                            (
                                "drogue bulkhead online",
                                true,
                                Self::format_bool(p.drogue_bulkhead_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.drogue_bulkhead_rebooted_in_last_5s()),
                            ),
                            (
                                "brightness",
                                true,
                                format!("{:.2}lux", p.drogue_bulkhead_brightness_lux()).into(),
                            ),
                        ],
                        &[
                            ("ozys 1 online", true, Self::format_bool(p.ozys1_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.ozys1_rebooted_in_last_5s()),
                            ),
                            ("ozys 2 online", true, Self::format_bool(p.ozys2_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.ozys2_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            (
                                "aero rust online",
                                true,
                                Self::format_bool(p.aero_rust_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.aero_rust_rebooted_in_last_5s()),
                            ),
                            ("health", true, format!("{:?}", p.aero_rust_health()).into()),
                        ],
                        &[
                            (
                                "payload activation pcb online",
                                true,
                                Self::format_bool(p.payload_activation_pcb_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.payload_activation_pcb_rebooted_in_last_5s()),
                            ),
                            (
                                "rocket wifi online",
                                true,
                                Self::format_bool(p.rocket_wifi_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.rocket_wifi_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            ("eps 1 online", true, Self::format_bool(p.eps1_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.eps1_rebooted_in_last_5s()),
                            ),
                            (
                                "batt 1 v",
                                false,
                                format!("{:.2}V", p.eps1_battery1_v()).into(),
                            ),
                            (
                                "batt 1 temp",
                                false,
                                format!("{:.1}C", p.eps1_battery1_temperature()).into(),
                            ),
                            (
                                "batt 2 v",
                                false,
                                format!("{:.2}V", p.eps1_battery2_v()).into(),
                            ),
                            (
                                "batt 2 temp",
                                false,
                                format!("{:.1}C", p.eps1_battery2_temperature()).into(),
                            ),
                        ],
                        &[
                            (
                                "3v3 out current",
                                false,
                                format!("{}mA", (p.eps1_output_3v3_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps1_output_3v3_overwrote(),
                                    p.eps1_output_3v3_status(),
                                ),
                            ),
                            (
                                "5v out current",
                                false,
                                format!("{}mA", (p.eps1_output_5v_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps1_output_5v_overwrote(),
                                    p.eps1_output_5v_status(),
                                ),
                            ),
                            (
                                "9v out current",
                                false,
                                format!("{}mA", (p.eps1_output_9v_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps1_output_9v_overwrote(),
                                    p.eps1_output_9v_status(),
                                ),
                            ),
                        ],
                        &[
                            ("eps 2 online", true, Self::format_bool(p.eps2_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.eps2_rebooted_in_last_5s()),
                            ),
                            (
                                "batt 1 v",
                                false,
                                format!("{:.2}V", p.eps2_battery1_v()).into(),
                            ),
                            (
                                "batt 1 temp",
                                false,
                                format!("{:.1}C", p.eps2_battery1_temperature()).into(),
                            ),
                            (
                                "batt 2 v",
                                false,
                                format!("{:.2}V", p.eps2_battery2_v()).into(),
                            ),
                            (
                                "batt 2 temp",
                                false,
                                format!("{:.1}C", p.eps2_battery2_temperature()).into(),
                            ),
                        ],
                        &[
                            (
                                "3v3 out current",
                                false,
                                format!("{}mA", (p.eps2_output_3v3_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps2_output_3v3_overwrote(),
                                    p.eps2_output_3v3_status(),
                                ),
                            ),
                            (
                                "5v out current",
                                false,
                                format!("{}mA", (p.eps2_output_5v_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps2_output_5v_overwrote(),
                                    p.eps2_output_5v_status(),
                                ),
                            ),
                            (
                                "9v out current",
                                false,
                                format!("{}mA", (p.eps2_output_9v_current() * 1000.0).round())
                                    .into(),
                            ),
                            (
                                "status",
                                true,
                                Self::format_amp_output_status(
                                    p.eps2_output_9v_overwrote(),
                                    p.eps2_output_9v_status(),
                                ),
                            ),
                        ],
                    ],
                ),
                VLPDownlinkPacket::SelfTestResult(p) => todo!(),
                VLPDownlinkPacket::Ack(p) => unreachable!(),
            }
        }
    }
}
