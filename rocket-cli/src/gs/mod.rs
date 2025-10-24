use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use anyhow::Result;
use cursive::{
    Cursive,
    theme::{Color, ColorStyle, Palette, PaletteStyle, Style},
    view::{Nameable, Resizable},
    views::{
        Button, Dialog, EditView, HideableView, LinearLayout, PaddedView, Panel, RadioGroup,
        TextView,
    },
};
use cursive_aligned_view::Alignable;
use embassy_sync::blocking_mutex::raw::RawMutex;
use firmware_common_new::{
    can_bus::messages::amp_overwrite::PowerOutputOverwrite,
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::{VLPGroundStation, VLPTXError},
        lora_config::LoraConfig,
        packets::{
            VLPDownlinkPacket, VLPUplinkPacket,
            amp_output_overwrite::AMPOutputOverwritePacket,
            change_mode::{ChangeModePacket, Mode},
            fire_pyro::{FirePyroPacket, PyroSelect},
            payload_eps_output_overwrite::PayloadEPSOutputOverwritePacket,
            reset::{DeviceToReset, ResetPacket},
            set_target_apogee::SetTargetApogeePacket,
        },
    },
};
use lora_phy::mod_params::PacketStatus;
use tokio::task::spawn_blocking;

use crate::{
    enable_stdout_logging,
    gs::{
        config::GroundStationConfig, downlink_packet_display::DownlinkPacketDisplay,
        rpc_radio::RpcRadio, serial_wrapper::SerialWrapper, vlp_client::VLPClientTrait,
    },
};

pub mod config;
mod downlink_packet_display;
pub mod find_ground_station;
pub mod rpc_radio;
pub mod serial_wrapper;
pub mod vlp_client;

struct MultiThreadRawMutex {
    mutex: std::sync::Mutex<()>,
}

unsafe impl Send for MultiThreadRawMutex {}
unsafe impl Sync for MultiThreadRawMutex {}

impl MultiThreadRawMutex {
    pub const fn new() -> Self {
        Self {
            mutex: std::sync::Mutex::new(()),
        }
    }
}

unsafe impl RawMutex for MultiThreadRawMutex {
    const INIT: Self = Self::new();
    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        let g = self.mutex.lock().unwrap();
        let result = f();
        drop(g);
        result
    }
}

pub async fn ground_station_tui(serial_path: &str) -> Result<()> {
    let serial = serialport::new(serial_path, 115200)
        .timeout(Duration::from_secs(5))
        .open()
        .unwrap();
    let mut serial = SerialWrapper::new(serial);

    let config = Arc::new(RwLock::new(GroundStationConfig::load()?));
    let mut client = LoraRpcClient::new(&mut serial);
    client.reset().await.unwrap();
    client
        .configure(LoraConfig {
            frequency: config.read().unwrap().frequency,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: config.read().unwrap().power,
        })
        .await
        .unwrap();
    let mut rpc_radio = RpcRadio::new(client, None);
    let vlp_gcm_client = Box::leak(Box::new(VLPGroundStation::<MultiThreadRawMutex>::new()));
    let vlp_key = config.read().unwrap().vlp_key.clone();
    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &vlp_key);

    struct VLPClientWrapper(&'static VLPGroundStation<MultiThreadRawMutex>);

    impl VLPClientTrait for VLPClientWrapper {
        fn send_nb(&self, packet: VLPUplinkPacket) {
            self.0.send_nb(packet);
        }

        fn try_get_send_result(&self) -> Option<std::result::Result<PacketStatus, VLPTXError>> {
            self.0.try_get_send_result()
        }

        fn try_receive(&self) -> Option<(VLPDownlinkPacket, PacketStatus)> {
            self.0.try_receive()
        }
    }

    let vlp_client = Box::leak(Box::new(VLPClientWrapper(vlp_gcm_client)));
    let config = config.clone();

    tokio::select! {
        _ = daemon.run() => {}
        _ = spawn_blocking(move || {
            tui_task(vlp_client, config)
        }) => {}
    }

    Ok(())
}

pub fn tui_task(
    client: &'static impl VLPClientTrait,
    config: Arc<RwLock<GroundStationConfig>>,
) -> Result<()> {
    let last_uplink_packet: &RwLock<Option<VLPUplinkPacket>> =
        Box::leak(Box::new(RwLock::new(None)));

    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();
    theme.palette = Palette::terminal_default();
    theme.palette[PaletteStyle::View] =
        Style::from_color_style(ColorStyle::back(Color::Rgb(248, 248, 248)));
    theme.palette[PaletteStyle::EditableTextCursor] = theme.palette[PaletteStyle::EditableText];
    theme.palette[PaletteStyle::EditableText] = theme.palette[PaletteStyle::Primary];
    siv.set_theme(theme);
    siv.set_autorefresh(true);

    let send_packet = |s: &mut Cursive, packet: VLPUplinkPacket| {
        client.send_nb(packet.clone());
        let mut last_uplink_packet = last_uplink_packet.write().unwrap();
        *last_uplink_packet = Some(packet.clone());

        s.find_name::<HideableView<LinearLayout>>("uplink_buttons_hideable")
            .unwrap()
            .set_visible(false);
        s.find_name::<HideableView<TextView>>("uplink_sending_hideable")
            .unwrap()
            .set_visible(true);
    };

    let create_set_target_altitude_input = || {
        Button::new("Target apogee", move |s| {
            s.add_layer(
                Dialog::new()
                    .title("Target apogee")
                    .content(
                        LinearLayout::horizontal()
                            .child(TextView::new("Target Altitude (Meters): "))
                            .child(
                                LinearLayout::vertical()
                                    .child(
                                        EditView::new().content("").with_name("target_apogee"),
                                    )
                                    .fixed_width(10),
                            ),
                    )
                    .dismiss_button("Cancel")
                    .button("Confirm", {
                        move |s| {
                            let target_apogee = s
                                .find_name::<EditView>("target_apogee")
                                .unwrap()
                                .get_content();
                            let target_apogee = match target_apogee.parse::<f32>() {
                                Ok(f) => f,
                                Err(_) => {
                                    s.add_layer(Dialog::info("Invalid target apogee value"));
                                    return;
                                }
                            };

                            let packet = VLPUplinkPacket::SetTargetApogee(
                                SetTargetApogeePacket::new(target_apogee),
                            );

                            s.pop_layer().unwrap();

                            send_packet(s, packet);

                            s.add_layer(Dialog::info(
                                format!("Sending new target apogee of {} meters", target_apogee).as_str(),
                            ));
                        }
                    }),
            );
        })
        .align_center_left()
    };
    let create_simple_packet_button =
        |button_text: &str, dialog_message: &str, packet: VLPUplinkPacket| {
            let button_text = button_text.to_string();
            let dialog_message = dialog_message.to_string();
            Button::new(button_text, move |s| {
                s.add_layer(
                    Dialog::around(TextView::new(dialog_message.clone()))
                        .dismiss_button("Cancel")
                        .button("Confirm", {
                            let packet = packet.clone();
                            move |s| {
                                send_packet(s, packet.clone());
                                s.pop_layer().unwrap();
                            }
                        }),
                );
            })
            .align_center_left()
        };

    let create_reset_device_button = || {
        Button::new("Reset Device", move |s| {
            let mut reset_device_selection_group: RadioGroup<DeviceToReset> = RadioGroup::new();

            s.add_layer(
                Dialog::new()
                    .title("Select a device to reset")
                    .content(
                        LinearLayout::vertical()
                            .child(reset_device_selection_group.button(DeviceToReset::All, "All"))
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::VoidLake, "Void Lake"),
                            )
                            .child(reset_device_selection_group.button(DeviceToReset::AMP, "AMP"))
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::AMPOut1, "AMP Out 1"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::AMPOut2, "AMP Out 2"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::AMPOut3, "AMP Out 3"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::AMPOut4, "AMP Out 4"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::Icarus, "ICARUS"),
                            )
                            .child(reset_device_selection_group.button(
                                DeviceToReset::PayloadActivationPCB,
                                "Payload Activation PCB",
                            ))
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::RocketWifi, "Rocket WiFi"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::OzysAll, "OZYS (All)"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::MainBulkhead, "Main Bulkhead PCB"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::DrogueBulkhead, "Drogue Bulkhead PCB"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::PayloadEPS1, "EPS 1"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::PayloadEPS2, "EPS 2"),
                            )
                            .child(
                                reset_device_selection_group
                                    .button(DeviceToReset::AeroRust, "AeroRust"),
                            ),
                    )
                    .dismiss_button("Cancel")
                    .button("Confirm", move |s| {
                        let device_to_reset = reset_device_selection_group.selection();
                        send_packet(
                            s,
                            ResetPacket {
                                device: *device_to_reset,
                            }
                            .into(),
                        );
                        s.pop_layer().unwrap();
                    }),
            );
        })
        .align_center_left()
    };

    let create_overwrite_amp_button = || {
        Button::new("Overwrite AMP", move |s| {
            let mut out1_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut out2_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut out3_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut out4_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();

            s.add_layer(
                Dialog::new()
                    .title("Overwrite AMP")
                    .content(
                        LinearLayout::vertical()
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("Out 1:  "))
                                    .child(
                                        out1_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out1_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out1_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("Out 2:  "))
                                    .child(
                                        out2_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out2_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out2_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("Out 3:  "))
                                    .child(
                                        out3_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out3_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out3_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("Out 4:  "))
                                    .child(
                                        out4_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out4_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        out4_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            ),
                    )
                    .dismiss_button("Cancel")
                    .button("Confirm", move |s| {
                        send_packet(
                            s,
                            AMPOutputOverwritePacket {
                                out1: *out1_selection_group.selection(),
                                out2: *out2_selection_group.selection(),
                                out3: *out3_selection_group.selection(),
                                out4: *out4_selection_group.selection(),
                            }
                            .into(),
                        );
                        s.pop_layer().unwrap();
                    }),
            );
        })
        .align_center_left()
    };

    let create_overwrite_eps_button = || {
        Button::new("Overwrite EPS", move |s| {
            let mut eps1_3v3_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut eps1_5v_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut eps1_9v_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut eps2_3v3_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut eps2_5v_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();
            let mut eps2_9v_selection_group: RadioGroup<PowerOutputOverwrite> = RadioGroup::new();

            s.add_layer(
                Dialog::new()
                    .title("Overwrite EPS")
                    .content(
                        LinearLayout::vertical()
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 1 3.3V:  "))
                                    .child(
                                        eps1_3v3_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_3v3_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_3v3_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 1   5V:  "))
                                    .child(
                                        eps1_5v_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_5v_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_5v_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 1   9V:  "))
                                    .child(
                                        eps1_9v_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_9v_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps1_9v_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(TextView::new(" "))
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 2 3.3V:  "))
                                    .child(
                                        eps2_3v3_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_3v3_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_3v3_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 2   5V:  "))
                                    .child(
                                        eps2_5v_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_5v_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_5v_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            )
                            .child(
                                LinearLayout::horizontal()
                                    .child(TextView::new("EPS 2   9V:  "))
                                    .child(
                                        eps2_9v_selection_group.button(
                                            PowerOutputOverwrite::NoOverwrite,
                                            "No Overwrite",
                                        ),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_9v_selection_group
                                            .button(PowerOutputOverwrite::ForceEnabled, "Enable"),
                                    )
                                    .child(TextView::new("  "))
                                    .child(
                                        eps2_9v_selection_group
                                            .button(PowerOutputOverwrite::ForceDisabled, "Disable"),
                                    ),
                            ),
                    )
                    .dismiss_button("Cancel")
                    .button("Confirm", move |s| {
                        send_packet(
                            s,
                            PayloadEPSOutputOverwritePacket {
                                eps1_3v3: *eps1_3v3_selection_group.selection(),
                                eps1_5v: *eps1_5v_selection_group.selection(),
                                eps1_9v: *eps1_9v_selection_group.selection(),
                                eps2_3v3: *eps2_3v3_selection_group.selection(),
                                eps2_5v: *eps2_5v_selection_group.selection(),
                                eps2_9v: *eps2_9v_selection_group.selection(),
                            }
                            .into(),
                        );
                        s.pop_layer().unwrap();
                    }),
            );
        })
        .align_center_left()
    };

    let create_config_button = || {
        let config = config.clone();
        Button::new("Config", move |s| {
            s.add_layer(
                Dialog::new()
                    .title("Config")
                    .content(
                        LinearLayout::horizontal()
                            .child(TextView::new("Frequency: \nPower: "))
                            .child(
                                LinearLayout::vertical()
                                    .child(
                                        EditView::new()
                                            .content(config.read().unwrap().frequency.to_string())
                                            .with_name("frequency"),
                                    )
                                    .child(
                                        EditView::new()
                                            .content(config.read().unwrap().power.to_string())
                                            .with_name("power"),
                                    )
                                    .fixed_width(10),
                            ),
                    )
                    .dismiss_button("Cancel")
                    .button("Confirm", {
                        let config = config.clone();
                        move |s| {
                            let frequency =
                                s.find_name::<EditView>("frequency").unwrap().get_content();
                            let power = s.find_name::<EditView>("power").unwrap().get_content();
                            let frequency = match frequency.parse::<u32>() {
                                Ok(f) => f,
                                Err(_) => {
                                    s.add_layer(Dialog::info("Invalid frequency value"));
                                    return;
                                }
                            };

                            let power = match power.parse::<i32>() {
                                Ok(p) => p,
                                Err(_) => {
                                    s.add_layer(Dialog::info("Invalid power value"));
                                    return;
                                }
                            };

                            let mut config = config.write().unwrap();
                            config.frequency = frequency;
                            config.power = power;
                            config.save().unwrap();
                            s.pop_layer().unwrap();
                            s.add_layer(Dialog::info(
                                "Config will take effect after rocket-cli restart",
                            ));
                        }
                    }),
            );
        })
        .align_center_left()
    };

    siv.add_fullscreen_layer(
        LinearLayout::horizontal()
            .child(
                Panel::new(PaddedView::lrtb(
                    1,
                    1,
                    0,
                    0,
                    LinearLayout::vertical()
                        .child(
                            HideableView::new(
                                LinearLayout::vertical()
                                    .child(create_config_button())
                                    .child(create_set_target_altitude_input())
                                    .child(create_simple_packet_button(
                                        "Low Power Mode",
                                        "Change rocket to low power mode?",
                                        ChangeModePacket {
                                            mode: Mode::LowPower,
                                        }
                                        .into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Self Test Mode",
                                        "Change rocket to self test mode?",
                                        ChangeModePacket {
                                            mode: Mode::SelfTest,
                                        }
                                        .into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Armed Mode",
                                        "Change rocket to armed mode?",
                                        ChangeModePacket { mode: Mode::Armed }.into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Landed Mode",
                                        "Change rocket to landed mode?",
                                        ChangeModePacket { mode: Mode::Landed }.into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Demo Mode",
                                        "Change rocket to demo mode?",
                                        ChangeModePacket { mode: Mode::Demo }.into(),
                                    ))
                                    .child(create_reset_device_button())
                                    .child(create_overwrite_amp_button())
                                    .child(create_overwrite_eps_button())
                                    .child(create_simple_packet_button(
                                        "Fire Main Pyro",
                                        "Manually fire main pyro?",
                                        FirePyroPacket {
                                            pyro: PyroSelect::PyroMain,
                                        }
                                        .into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Fire Drogue Pyro",
                                        "Manually fire drogue pyro?",
                                        FirePyroPacket {
                                            pyro: PyroSelect::PyroDrogue,
                                        }
                                        .into(),
                                    )),
                            )
                            .visible(true)
                            .with_name("uplink_buttons_hideable"),
                        )
                        .child(
                            HideableView::new(TextView::new("Sending....."))
                                .visible(false)
                                .with_name("uplink_sending_hideable"),
                        ),
                ))
                .title("Send Uplink")
                .fixed_width(22)
                .full_height(),
            )
            .child(
                Panel::new(DownlinkPacketDisplay::new().with_name("downlink_packet"))
                    .title("Ground Station Downlink")
                    .full_screen(),
            ),
    );

    enable_stdout_logging(false);
    let mut runner = siv.runner();
    runner.refresh();

    while runner.is_running() {
        if let Some(result) = client.try_get_send_result() {
            runner
                .find_name::<HideableView<TextView>>("uplink_sending_hideable")
                .unwrap()
                .set_visible(false);
            runner
                .find_name::<HideableView<LinearLayout>>("uplink_buttons_hideable")
                .unwrap()
                .set_visible(true);

            match result {
                Ok(status) => {
                    runner.add_layer(
                        Dialog::new()
                            .title("Uplink Success")
                            .content(TextView::new(format!(
                                "ACK rssi={} snr={}",
                                status.rssi, status.snr
                            )))
                            .dismiss_button("OK"),
                    );
                }
                Err(e) => {
                    runner.add_layer(
                        Dialog::new()
                            .title("Uplink Error")
                            .content(TextView::new(format!("{:?}", e)))
                            .button("retry", move |s| {
                                send_packet(s, last_uplink_packet.read().unwrap().clone().unwrap())
                            })
                            .dismiss_button("OK"),
                    );
                }
            }
        }

        if let Some((packet, status)) = client.try_receive() {
            let mut downlink_packet_display = runner
                .find_name::<DownlinkPacketDisplay>("downlink_packet")
                .unwrap();
            downlink_packet_display.update(packet, status);
        }

        runner.step();
    }
    enable_stdout_logging(false);

    Ok(())
}
