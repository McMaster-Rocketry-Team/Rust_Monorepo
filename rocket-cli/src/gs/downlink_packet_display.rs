use std::{sync::RwLock, time::Instant};

use cursive::{
    Printer, Rect, Vec2, View, XY,
    theme::{BaseColor, Color, ColorStyle, Effect, Style},
    utils::markup::StyledString,
};
use firmware_common_new::{
    can_bus::{
        custom_status::{
            NodeCustomStatusExt, ozys_custom_status::OzysCustomStatus,
            payload_sdrm_custom_status::PayloadSDRMCustomStatus,
        },
        messages::{amp_status::PowerOutputStatus, node_status::NodeMode},
    },
    vlp::packets::{VLPDownlinkPacket, self_test_result::NodeStatus},
};
use lora_phy::mod_params::PacketStatus;
use pad::PadStr as _;

use crate::monitor::FieldWidget;

/// Dimmed grey, used for anything that is not a live reading: field labels,
/// the absent-value placeholder, and a disabled AMP output.
const MUTED: Color = Color::Rgb(127, 127, 127);

/// One labelled group of fields — a heading line followed by its rows.
///
/// The grouping is by subsystem rather than by packet bit order, because the
/// question an operator actually asks is "is Icarus healthy" or "is the
/// payload stack up", and the answer to each is spread across several
/// non-adjacent fields of the packet.
struct Section {
    title: &'static str,
    rows: Vec<Vec<(&'static str, bool, StyledString)>>,
}

impl Section {
    fn new(
        title: &'static str,
        rows: Vec<Vec<(&'static str, bool, StyledString)>>,
    ) -> Self {
        Self { title, rows }
    }

    /// Lines this section occupies: its heading plus one per row.
    fn height(&self) -> usize {
        1 + self.rows.len()
    }
}

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

    /// `T` green, `F` red, so the state of a flag is legible from across the
    /// tent without reading the letter.
    ///
    /// The colouring is by VALUE, not by whether the value is good news: it is
    /// one rule an operator can learn once and apply everywhere, and a panel
    /// where green sometimes means "bad" would be worse than no colour at all.
    /// The two flags this reads backwards for are `rebooted` (a green `T`
    /// there is a node that reset mid-flight) and `overwrote` on the AMP
    /// outputs — both are rare enough that the field name carries the meaning.
    fn format_bool(value: bool) -> StyledString {
        StyledString::single_span(
            if value { "T" } else { "F" },
            Style::from_color_style(ColorStyle::front(if value {
                BaseColor::Green.dark()
            } else {
                BaseColor::Red.dark()
            })),
        )
    }

    /// A flag that only means something while the node carrying it is
    /// reporting.
    ///
    /// The payload stack flags are the case this exists for. Both firmware
    /// paths fabricate an all-false stack status when the SDRM is not on the
    /// bus — `PayloadSDRMCustomStatus::new()` in armed mode, and a zero
    /// `custom_status` via `NodeStatus::offline()` in the self test — so the
    /// eight flags decode to `false` whether the stack is genuinely down or
    /// simply silent. Rendering that as eight red `F`s would put what looks
    /// like eight separate hardware failures on the screen of an operator
    /// whose actual problem is one missing CAN node.
    fn format_bool_reported(reported: bool, value: bool) -> StyledString {
        if reported {
            Self::format_bool(value)
        } else {
            Self::format_unavailable()
        }
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
        StyledString::single_span("n/a", Style::from_color_style(ColorStyle::front(MUTED)))
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
            PowerOutputStatus::Disabled => {
                s.append_styled("disabled", Style::from_color_style(ColorStyle::front(MUTED)))
            }
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

    /// The whole panel body for the packet currently held, grouped by
    /// subsystem.
    ///
    /// Built by both `draw` and `required_size` so the height the view asks
    /// for and the height it actually paints cannot disagree — a mismatch
    /// silently clips the bottom rows, which for this panel would mean losing
    /// the payload readings with nothing on screen to say so.
    fn sections(&self) -> Vec<Section> {
        let Some(Packet { packet, .. }) = &self.packet else {
            return vec![];
        };

        match packet {
            VLPDownlinkPacket::LowPowerTelemetry(p) => {
                let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                vec![
                    Section::new(
                        "GPS",
                        vec![vec![
                            ("gps fixed", true, Self::format_bool(p.gps_fixed)),
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, lat),
                            ("lon", false, lon),
                        ]],
                    ),
                    Section::new(
                        "VL",
                        vec![vec![
                            (
                                "vl battery",
                                false,
                                format!("{:.2}V", p.vl_battery_v()).into(),
                            ),
                            (
                                "air temperature",
                                false,
                                format!("{:.1}C", p.air_temperature()).into(),
                            ),
                        ]],
                    ),
                    Section::new(
                        "AMP",
                        vec![vec![
                            ("online", true, Self::format_bool(p.amp_online)),
                            (
                                "shared battery",
                                false,
                                format!("{:.2}V", p.shared_battery_v()).into(),
                            ),
                        ]],
                    ),
                ]
            }
            VLPDownlinkPacket::LandedTelemetry(p) => {
                let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                vec![
                    Section::new(
                        "GPS",
                        vec![vec![
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("lat", false, lat),
                            ("lon", false, lon),
                        ]],
                    ),
                    Section::new(
                        "VL",
                        vec![vec![(
                            "vl battery",
                            false,
                            format!("{:.2}V", p.battery_v()).into(),
                        )]],
                    ),
                    Section::new(
                        "AMP",
                        vec![
                            vec![
                                ("online", true, Self::format_bool(p.amp_online())),
                                (
                                    "rebooted",
                                    true,
                                    Self::format_bool(p.amp_rebooted_in_last_5s()),
                                ),
                                (
                                    "shared battery",
                                    false,
                                    format!("{:.2}V", p.shared_battery_v()).into(),
                                ),
                            ],
                            vec![
                                (
                                    "out 1",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out1_overwrote(),
                                        p.amp_out1(),
                                    ),
                                ),
                                (
                                    "out 2",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out2_overwrote(),
                                        p.amp_out2(),
                                    ),
                                ),
                                (
                                    "out 3",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out3_overwrote(),
                                        p.amp_out3(),
                                    ),
                                ),
                            ],
                        ],
                    ),
                ]
            }
            VLPDownlinkPacket::Telemetry(p) => {
                let (lat, lon) = Self::format_lat_lon(p.lat_lon());
                vec![
                    // The deployment KF goes absent for the whole Mach
                    // lockout, so altitude and vertical velocity read "n/a"
                    // there while the state still says Ascent. That pairing is
                    // the tell: an operator who sees blanks under an ascending
                    // rocket is looking at a frozen filter, not at a rocket
                    // sitting at 0m doing 0m/s. `max altitude agl` is latched
                    // and survives the window, so it stays the number to read.
                    Section::new(
                        "Flight",
                        vec![vec![
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
                        ]],
                    ),
                    // `predicted` against `target` is the pair worth watching
                    // on ascent: while they agree the brakes have authority,
                    // and the gap between them is the overshoot the MPC cannot
                    // fix.
                    Section::new(
                        "Airbrakes",
                        vec![
                            vec![
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
                                ("born", true, Self::format_bool(p.airbrakes_born())),
                            ],
                            vec![
                                (
                                    "commanded extension",
                                    false,
                                    format!(
                                        "{}%",
                                        (p.air_brakes_commanded_extension_percentage() * 100.0)
                                            .round()
                                    )
                                    .into(),
                                ),
                                // Absent until Icarus sends its first status
                                // message, which is later than "icarus online"
                                // going true — an "n/a" next to an online
                                // Icarus means it has not reported the brakes
                                // yet, not that they are stowed.
                                (
                                    "actual extension",
                                    false,
                                    Self::format_optional(
                                        p.air_brakes_actual_extension_percentage(),
                                        |v| format!("{}%", (v * 100.0).round()),
                                    ),
                                ),
                                (
                                    "servo temp",
                                    false,
                                    Self::format_optional(p.air_brakes_servo_temp(), |v| {
                                        format!("{:.1}C", v)
                                    }),
                                ),
                            ],
                        ],
                    ),
                    Section::new(
                        "GPS",
                        vec![vec![
                            (
                                "satellites",
                                false,
                                p.num_of_fix_satellites().to_string().into(),
                            ),
                            ("unix clock", true, Self::format_bool(p.unix_clock_ready())),
                            ("lat", false, lat),
                            ("lon", false, lon),
                        ]],
                    ),
                    Section::new(
                        "VL",
                        vec![vec![
                            (
                                "vl battery",
                                false,
                                format!("{:.2}V", p.vl_battery_v()).into(),
                            ),
                            (
                                "air temperature",
                                false,
                                format!("{:.1}C", p.air_temperature()).into(),
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
                        ]],
                    ),
                    Section::new(
                        "AMP",
                        vec![
                            vec![
                                ("online", true, Self::format_bool(p.amp_online())),
                                (
                                    "rebooted",
                                    true,
                                    Self::format_bool(p.amp_rebooted_in_last_5s()),
                                ),
                                (
                                    "shared battery",
                                    false,
                                    format!("{:.2}V", p.shared_battery_v()).into(),
                                ),
                            ],
                            vec![
                                (
                                    "out 1",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out1_overwrote(),
                                        p.amp_out1(),
                                    ),
                                ),
                                (
                                    "out 2",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out2_overwrote(),
                                        p.amp_out2(),
                                    ),
                                ),
                                (
                                    "out 3",
                                    true,
                                    Self::format_amp_output_status(
                                        p.amp_out3_overwrote(),
                                        p.amp_out3(),
                                    ),
                                ),
                            ],
                        ],
                    ),
                    Section::new(
                        "Icarus",
                        vec![vec![
                            ("online", true, Self::format_bool(p.icarus_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.icarus_rebooted_in_last_5s()),
                            ),
                        ]],
                    ),
                    Section::new(
                        "OZYS",
                        vec![vec![
                            ("online", true, Self::format_bool(p.ozys_online())),
                            (
                                "rebooted",
                                true,
                                Self::format_bool(p.ozys_rebooted_in_last_5s()),
                            ),
                        ]],
                    ),
                    Section::new(
                        "Payload",
                        vec![
                            // The eight stack flags are only a reading while
                            // the SDRM is online; off the bus they are the
                            // firmware's all-false filler, not observations.
                            vec![
                                (
                                    "sdrm online",
                                    true,
                                    Self::format_bool(p.payload_sdrm_online()),
                                ),
                                (
                                    "rebooted",
                                    true,
                                    Self::format_bool(p.payload_sdrm_rebooted_in_last_5s()),
                                ),
                                (
                                    "epm alive",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_epm_alive(),
                                    ),
                                ),
                                (
                                    "sem alive",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_sem_alive(),
                                    ),
                                ),
                                (
                                    "rails on",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_epm_rails_on(),
                                    ),
                                ),
                            ],
                            vec![
                                (
                                    "exp 1",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_exp1_active(),
                                    ),
                                ),
                                (
                                    "exp 2",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_exp2_active(),
                                    ),
                                ),
                                (
                                    "exp 3",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_exp3_active(),
                                    ),
                                ),
                                (
                                    "sdrm sd log",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_sdrm_sd_logging(),
                                    ),
                                ),
                                (
                                    "sem sd log",
                                    true,
                                    Self::format_bool_reported(
                                        p.payload_sdrm_online(),
                                        p.payload_sem_sd_logging(),
                                    ),
                                ),
                            ],
                            // Each payload reading is separately absent: one
                            // dead sensor blanks its own column and leaves the
                            // rest readable. A rail that is switched off still
                            // shows 0mA, which is why "n/a" has to look
                            // different from a zero here.
                            vec![
                                (
                                    "epm batt",
                                    false,
                                    Self::format_optional(p.epm_batt_v(), |v| {
                                        format!("{:.2}V", v)
                                    }),
                                ),
                                (
                                    "sys 3v3",
                                    false,
                                    Self::format_optional(p.epm_sys_3v3_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                                (
                                    "sys 5v",
                                    false,
                                    Self::format_optional(p.epm_sys_5v_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                                (
                                    "per 3v3",
                                    false,
                                    Self::format_optional(p.epm_per_3v3_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                                (
                                    "per 5v",
                                    false,
                                    Self::format_optional(p.epm_per_5v_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                                (
                                    "per 9v",
                                    false,
                                    Self::format_optional(p.epm_per_9v_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                                (
                                    "per 12v",
                                    false,
                                    Self::format_optional(p.epm_per_12v_ma(), |v| {
                                        format!("{}mA", v)
                                    }),
                                ),
                            ],
                            vec![
                                (
                                    "act 1",
                                    false,
                                    Self::format_optional(p.sem_actuator_1_steps(), |v| {
                                        v.to_string()
                                    }),
                                ),
                                (
                                    "act 2",
                                    false,
                                    Self::format_optional(p.sem_actuator_2_steps(), |v| {
                                        v.to_string()
                                    }),
                                ),
                                (
                                    "act 3",
                                    false,
                                    Self::format_optional(p.sem_actuator_3_steps(), |v| {
                                        v.to_string()
                                    }),
                                ),
                            ],
                        ],
                    ),
                ]
            }
            VLPDownlinkPacket::SelfTestResult(p) => {
                // The payload stack flags ride in the SDRM's node custom
                // status, the same 11 bits the telemetry packet unpacks into
                // named fields. Decoding them here too means the pre-flight
                // check and the in-flight panel answer "is the stack up" the
                // same way, instead of the self test showing only a health
                // enum and leaving the operator to guess.
                //
                // `NodeStatus::offline()` zeroes the custom status, so an SDRM
                // that never appeared on the bus decodes to eight `false`
                // flags that are filler rather than observations. Gate them on
                // the node having actually reported.
                let stack = PayloadSDRMCustomStatus::from_u16(p.payload_sdrm.custom_status);
                let stack_reported = p.payload_sdrm.mode != NodeMode::Offline;
                vec![
                    Section::new(
                        "VL",
                        vec![
                            vec![
                                ("imu ok", true, Self::format_bool(p.imu_ok)),
                                ("baro ok", true, Self::format_bool(p.baro_ok)),
                                ("mag ok", true, Self::format_bool(p.mag_ok)),
                                ("gps ok", true, Self::format_bool(p.gps_ok)),
                                ("sd ok", true, Self::format_bool(p.sd_ok)),
                                ("can bus ok", true, Self::format_bool(p.can_bus_ok)),
                            ],
                            vec![
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
                        ],
                    ),
                    Section::new(
                        "AMP",
                        vec![vec![
                            ("status", true, Self::format_node_status(&p.amp)),
                            ("out 1 good", true, Self::format_bool(p.amp_out1_power_good)),
                            ("out 2 good", true, Self::format_bool(p.amp_out2_power_good)),
                            ("out 3 good", true, Self::format_bool(p.amp_out3_power_good)),
                        ]],
                    ),
                    Section::new(
                        "Icarus",
                        vec![vec![(
                            "status",
                            true,
                            Self::format_node_status(&p.icarus),
                        )]],
                    ),
                    Section::new(
                        "OZYS",
                        vec![vec![
                            ("status", true, Self::format_node_status(&p.ozys)),
                            (
                                "disk",
                                false,
                                format!(
                                    "{}%",
                                    (OzysCustomStatus::from_u16(p.ozys.custom_status).disk_usage()
                                        * 100.0)
                                        .round()
                                )
                                .into(),
                            ),
                        ]],
                    ),
                    Section::new(
                        "Payload",
                        vec![
                            vec![
                                ("sdrm status", true, Self::format_node_status(&p.payload_sdrm)),
                                ("epm alive", true, Self::format_bool_reported(stack_reported, stack.epm_alive)),
                                ("sem alive", true, Self::format_bool_reported(stack_reported, stack.sem_alive)),
                                ("rails on", true, Self::format_bool_reported(stack_reported, stack.epm_rails_on)),
                            ],
                            vec![
                                ("exp 1", true, Self::format_bool_reported(stack_reported, stack.exp1_active)),
                                ("exp 2", true, Self::format_bool_reported(stack_reported, stack.exp2_active)),
                                ("exp 3", true, Self::format_bool_reported(stack_reported, stack.exp3_active)),
                                (
                                    "sdrm sd log",
                                    true,
                                    Self::format_bool_reported(stack_reported, stack.sdrm_sd_logging),
                                ),
                                ("sem sd log", true, Self::format_bool_reported(stack_reported, stack.sem_sd_logging)),
                            ],
                        ],
                    ),
                ]
            }
            // Filtered out in `update`; handle defensively so a stray ack can
            // never panic the TUI.
            VLPDownlinkPacket::Ack(_) => vec![],
        }
    }

    /// Paint the sections, reusing one `FieldWidget` per field across frames so
    /// the change-highlight animation survives a redraw.
    ///
    /// The widget cache is flat over every row of every section, in order.
    /// That is safe because a given packet type always produces the same shape
    /// — and `update` clears the cache whenever the type changes, which is the
    /// only time the shape can move.
    fn draw_sections(&self, printer: &Printer, sections: &[Section]) {
        let mut self_fields = self.fields.write().unwrap();

        if self_fields.is_empty() {
            *self_fields = sections
                .iter()
                .flat_map(|section| section.rows.iter())
                .map(|row| {
                    row.iter()
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
                .flat_map(|self_fields_line| self_fields_line.iter_mut());
            let fields_iter = sections
                .iter()
                .flat_map(|section| section.rows.iter())
                .flat_map(|row| row.iter());
            for (field_widget, field) in self_fields_iter.zip(fields_iter) {
                field_widget.update(field.2.clone());
            }
        }

        let heading_style = Style::from(Effect::Bold)
            .combine(ColorStyle::front(Color::Rgb(120, 170, 255)));

        let mut printer = printer.clone();
        let mut row_index = 0usize;
        for section in sections {
            printer.print_styled(
                (0, 0),
                &StyledString::single_span(section.title, heading_style),
            );
            printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));

            for _ in &section.rows {
                let Some(self_fields_line) = self_fields.get(row_index) else {
                    break;
                };
                let mut x_offset = 2usize;
                for field in self_fields_line {
                    field.draw(&mut x_offset, &printer);
                }
                row_index += 1;
                printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
            }
        }
    }
}

impl View for DownlinkPacketDisplay {
    /// Ask for exactly the height the sections need.
    ///
    /// Without this the panel is sized by its parent and `draw` silently
    /// clips whatever does not fit — which, now that the body is grouped into
    /// headed sections, is tall enough to lose the payload rows on a short
    /// terminal with nothing on screen to indicate it. The parent wraps this
    /// in a scroll view, so an honest answer here is what makes the rest
    /// reachable.
    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        let height: usize = self.sections().iter().map(Section::height).sum();
        // +1 for the rssi / snr status line `draw` prints above the sections.
        XY::new(constraint.x, height + 1)
    }

    fn draw(&self, printer: &Printer) {
        if let Some(Packet {
            status,
            received_time,
            ..
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
            if printer.size.x > time_str.len() {
                printer.print((printer.size.x - time_str.len(), 0), &time_str);
            }

            let printer = printer.windowed(Rect::from_corners(Vec2::new(0, 1), printer.size));
            self.draw_sections(&printer, &self.sections());
        }
    }
}
