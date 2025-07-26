use std::{sync::RwLock, time::Instant};

use cursive::{Printer, Rect, Vec2, View, theme::Color, utils::markup::StyledString};
use firmware_common_new::vlp::packets::VLPDownlinkPacket;
use lora_phy::mod_params::PacketStatus;

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
                                Color::Rgb(255, 255, 255),
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
                VLPDownlinkPacket::LowPowerTelemetry(p) => todo!(),
                VLPDownlinkPacket::Telemetry(p) => todo!(),
                VLPDownlinkPacket::SelfTestResult(p) => todo!(),
                VLPDownlinkPacket::LandedTelemetry(p) => todo!(),
                VLPDownlinkPacket::Ack(p) => unreachable!(),
            }
        }
    }
}
