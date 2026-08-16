use std::{sync::RwLock, time::Instant};

use cursive::{
    Printer, Rect, Vec2, View,
    theme::{BaseColor, Color, ColorStyle, Style},
    utils::markup::StyledString,
};
use firmware_common_new::{
    can_bus::{
        custom_status::{NodeCustomStatusExt, ozys_custom_status::OzysCustomStatus},
        messages::amp_status::PowerOutputStatus,
    },
    vlp::packets::{VLPDownlinkPacket, self_test_result::NodeStatus},
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
        // Acks are consumed by the uplink `tx()` path, not shown as telemetry. One can
        // still reach here if it arrives after the ack-listen window closed and the
        // daemon picks it up in continuous rx. Ignore it so it neither panics `draw`
        // (its match arm is `unreachable!`) nor clobbers the live telemetry panel.
        if matches!(packet, VLPDownlinkPacket::Ack(_)) {
            return;
        }

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

    /// How an absent reading looks on the panel: dimmed and non-numeric, so it
    /// reads as "the rocket did not report this" at a glance rather than as a
    /// value.
    ///
    /// This is not cosmetic. The deployment KF is absent for the whole Mach
    /// lockout, and the alternative rendering — the 0.0 the packet's unused
    /// bits hold — puts "0.0m, 0.0m/s" on the screen at Mach 1, which is what
    /// a landed rocket looks like. Same placeholder as the CAN monitor's
    /// unavailable payload readings, so one convention covers both panels.
    fn format_unavailable() -> StyledString {
        StyledString::single_span(
            "n/a",
            Style::from_color_style(ColorStyle::front(Color::Rgb(127, 127, 127))),
        )
    }

    /// Render a reading that the packet may not carry. Every `Option`-returning
    /// getter goes through here, so no call site can quietly `unwrap_or(0.0)`
    /// its way back to a number the rocket never sent.
    fn format_optional<T>(value: Option<T>, format: impl FnOnce(T) -> String) -> StyledString {
        match value {
            Some(value) => format(value).into(),
            None => Self::format_unavailable(),
        }
    }

    /// The two coordinate fields, from the one `Option` that carries both.
    /// Taking them from a single `lat_lon()` is what keeps them blanking
    /// together: a latitude shown next to an "n/a" longitude would be a
    /// bearing the recovery team could act on and shouldn't.
    fn format_lat_lon(lat_lon: Option<(f64, f64)>) -> (StyledString, StyledString) {
        (
            Self::format_optional(lat_lon, |(lat, _)| lat.to_string()),
            Self::format_optional(lat_lon, |(_, lon)| lon.to_string()),
        )
    }

    fn format_node_status(value: &NodeStatus) -> StyledString {
        String::from(format!(
            "{:?}, {:?}{}",
            value.health,
            value.mode,
            if value.rebooted_in_last_5s {
                " rebooted"
            } else {
                ""
            }
        ))
        .into()
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
                VLPDownlinkPacket::LowPowerTelemetry(p) => {
                    let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                    self.draw_fields(
                    &printer,
                    &[
                        &[
                            ("gps fixed", true, Self::format_bool(p.gps_fixed)),
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, lat),
                            ("lon", false, lon),
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
                    )
                }
                VLPDownlinkPacket::LandedTelemetry(p) => {
                    let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                    self.draw_fields(
                    &printer,
                    &[
                        &[
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, lat),
                            ("lon", false, lon),
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
                            // (
                            //     "amp out 4",
                            //     true,
                            //     Self::format_amp_output_status(
                            //         p.amp_out4_overwrote(),
                            //         p.amp_out4(),
                            //     ),
                            // ),
                        ],
                    ],
                    )
                }
                VLPDownlinkPacket::Telemetry(p) => {
                    let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                    self.draw_fields(
                    &printer,
                    &[
                        &[
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("unix clock", true, Self::format_bool(p.unix_clock_ready())),
                            ("lat", false, lat),
                            ("lon", false, lon),
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
                                Self::format_optional(p.air_brakes_servo_temp(), |v| {
                                    format!("{:.1}C", v)
                                }),
                            ),
                        ],
                        // The deployment KF goes absent for the whole Mach
                        // lockout, so these three read "n/a" there while the
                        // stage still says Ascent. That pairing is the tell:
                        // an operator who sees blanks under an ascending
                        // rocket is looking at a frozen filter, not at a
                        // rocket sitting at 0m doing 0m/s.
                        &[
                            ("state", true, format!("{:?}", p.flight_stage()).into()),
                            (
                                "altitude agl",
                                false,
                                Self::format_optional(p.deployment_kf_altitude_agl(), |v| {
                                    format!("{:.1}m", v)
                                }),
                            ),
                            (
                                "max altitude agl",
                                false,
                                Self::format_optional(p.max_deployment_kf_altitude_agl(), |v| {
                                    format!("{:.1}m", v)
                                }),
                            ),
                            (
                                "vertical velocity",
                                false,
                                Self::format_optional(p.deployment_kf_vertical_velocity(), |v| {
                                    format!("{:.1}m/s", v)
                                }),
                            ),
                            (
                                "tilt",
                                false,
                                Self::format_optional(p.airbrakes_kf_tilt_deg(), |v| {
                                    format!("{:.1}deg", v)
                                }),
                            ),
                        ],
                        &[
                            (
                                "predicted apogee agl",
                                false,
                                Self::format_optional(p.mpc_predicted_apogee_agl(), |v| {
                                    format!("{:.1}m", v)
                                }),
                            ),
                            (
                                "target apogee agl",
                                false,
                                format!("{:.1}m", p.target_apogee_agl()).into(),
                            ),
                            ("airbrakes born", true, Self::format_bool(p.airbrakes_born())),
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
                            // Absent until Icarus sends its first status
                            // message, which is later than "icarus online"
                            // going true — an "n/a" next to an online Icarus
                            // means it has not reported the brakes yet, not
                            // that they are stowed.
                            (
                                "actual extension",
                                false,
                                Self::format_optional(
                                    p.air_brakes_actual_extension_percentage(),
                                    |v| format!("{}%", (v * 100.0).round()),
                                ),
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
                            // (
                            //     "amp out 4",
                            //     true,
                            //     Self::format_amp_output_status(
                            //         p.amp_out4_overwrote(),
                            //         p.amp_out4(),
                            //     ),
                            // ),
                        ],
                        &[
                            ("ozys online", true, Self::format_bool(p.ozys_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.ozys_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            (
                                "payload sdrm online",
                                true,
                                Self::format_bool(p.payload_sdrm_online()),
                            ),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.payload_sdrm_rebooted_in_last_5s()),
                            ),
                        ],
                        &[
                            ("epm alive", true, Self::format_bool(p.payload_epm_alive())),
                            ("sem alive", true, Self::format_bool(p.payload_sem_alive())),
                            (
                                "epm rails on",
                                true,
                                Self::format_bool(p.payload_epm_rails_on()),
                            ),
                            (
                                "sdrm sd log",
                                true,
                                Self::format_bool(p.payload_sdrm_sd_logging()),
                            ),
                            ("sem sd log", true, Self::format_bool(p.payload_sem_sd_logging())),
                        ],
                        &[
                            ("exp 1", true, Self::format_bool(p.payload_exp1_active())),
                            ("exp 2", true, Self::format_bool(p.payload_exp2_active())),
                            ("exp 3", true, Self::format_bool(p.payload_exp3_active())),
                        ],
                        // Each payload reading is separately absent: one dead
                        // sensor blanks its own column and leaves the rest
                        // readable. A rail that is switched off still shows
                        // 0mA, which is why "n/a" has to look different from a
                        // zero here.
                        &[
                            (
                                "epm batt",
                                false,
                                Self::format_optional(p.epm_batt_v(), |v| format!("{:.2}V", v)),
                            ),
                            (
                                "sys 3v3",
                                false,
                                Self::format_optional(p.epm_sys_3v3_ma(), |v| format!("{}mA", v)),
                            ),
                            (
                                "sys 5v",
                                false,
                                Self::format_optional(p.epm_sys_5v_ma(), |v| format!("{}mA", v)),
                            ),
                            (
                                "per 3v3",
                                false,
                                Self::format_optional(p.epm_per_3v3_ma(), |v| format!("{}mA", v)),
                            ),
                            (
                                "per 5v",
                                false,
                                Self::format_optional(p.epm_per_5v_ma(), |v| format!("{}mA", v)),
                            ),
                            (
                                "per 9v",
                                false,
                                Self::format_optional(p.epm_per_9v_ma(), |v| format!("{}mA", v)),
                            ),
                            (
                                "per 12v",
                                false,
                                Self::format_optional(p.epm_per_12v_ma(), |v| format!("{}mA", v)),
                            ),
                        ],
                        &[
                            (
                                "act 1",
                                false,
                                Self::format_optional(p.sem_actuator_1_steps(), |v| v.to_string()),
                            ),
                            (
                                "act 2",
                                false,
                                Self::format_optional(p.sem_actuator_2_steps(), |v| v.to_string()),
                            ),
                            (
                                "act 3",
                                false,
                                Self::format_optional(p.sem_actuator_3_steps(), |v| v.to_string()),
                            ),
                        ],
                    ],
                    )
                }
                VLPDownlinkPacket::SelfTestResult(p) => self.draw_fields(
                    &printer,
                    &[
                        &[
                            ("imu ok", true, Self::format_bool(p.imu_ok)),
                            ("baro ok", true, Self::format_bool(p.baro_ok)),
                            ("mag ok", true, Self::format_bool(p.mag_ok)),
                            ("gps ok", true, Self::format_bool(p.gps_ok)),
                            ("sd ok", true, Self::format_bool(p.sd_ok)),
                            ("can bus ok", true, Self::format_bool(p.can_bus_ok)),
                        ],
                        &[
                            (
                                "main continuity",
                                true,
                                Self::format_bool(p.main_continuity),
                            ),
                            (
                                "drogue continuity",
                                true,
                                Self::format_bool(p.drogue_continuity),
                            ),
                        ],
                        &[
                            ("amp", true, Self::format_node_status(&p.amp)),
                            ("out 1 good", true, Self::format_bool(p.amp_out1_power_good)),
                            ("out 2 good", true, Self::format_bool(p.amp_out2_power_good)),
                            ("out 3 good", true, Self::format_bool(p.amp_out3_power_good)),
                            // ("out 4 good", true, Self::format_bool(p.amp_out4_power_good)),
                        ],
                        &[
                            ("icarus", true, Self::format_node_status(&p.icarus)),
                            ("ozys", true, Self::format_node_status(&p.ozys)),
                            (
                                "ozys disk",
                                false,
                                format!(
                                    "{}%",
                                    (OzysCustomStatus::from_u16(p.ozys.custom_status)
                                        .disk_usage()
                                        * 100.0)
                                        .round()
                                )
                                .into(),
                            ),
                        ],
                        &[
                            (
                                "payload sdrm",
                                true,
                                Self::format_node_status(&p.payload_sdrm),
                            ),
                        ],
                    ],
                ),
                // Filtered out in `update`; handle defensively so a stray ack can
                // never panic the TUI.
                VLPDownlinkPacket::Ack(_) => {}
            }
        }
    }
}
