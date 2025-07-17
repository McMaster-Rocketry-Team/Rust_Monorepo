mod rpc_radio;

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::Result;
use cursive::{
    Cursive,
    align::HAlign,
    theme::{Palette, PaletteStyle},
    view::{Nameable, Resizable},
    views::{Button, Dialog, HideableView, LinearLayout, PaddedView, Panel, RadioGroup, TextView},
};
use cursive_aligned_view::Alignable;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use firmware_common_new::{
    can_bus::messages::amp_overwrite::PowerOutputOverwrite,
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPGroundStation,
        lora_config::LoraConfig,
        packets::{
            VLPUplinkPacket,
            amp_output_overwrite::AMPOutputOverwritePacket,
            change_mode::{ChangeModePacket, Mode},
            fire_pyro::{FirePyroPacket, PyroSelect},
            payload_eps_output_overwrite::PayloadEPSOutputOverwritePacket,
            reset::{DeviceToReset, ResetPacket},
        },
    },
};
use log::{error, info, warn};
use tokio::time;
use tokio_serial::SerialPortBuilderExt as _;

use crate::gs::{
    rpc_radio::RpcRadio,
    serial_wrapper::{Delay, SerialWrapper},
};

mod serial_wrapper;

const VLP_KEY: [u8; 32] = [42u8; 32];

pub async fn ground_station_tui(serial_port: &str) -> Result<()> {
    let serial = tokio_serial::new(serial_port, 115200)
        .open_native_async()
        .expect("open serial port");
    let mut serial = SerialWrapper(serial);

    let mut client = LoraRpcClient::new(&mut serial, Delay);

    client.reset().await.unwrap();

    client
        .configure(LoraConfig {
            frequency: 915_100_000,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        })
        .await
        .unwrap();

    let mut rpc_radio = RpcRadio::new(client);

    let vlp_gcm_client = Arc::new(VLPGroundStation::<ThreadModeRawMutex>::new());

    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &VLP_KEY);

    let daemon_fut = daemon.run();

    let tui_fut = tui_task(vlp_gcm_client.clone());

    tokio::join!(daemon_fut, tui_fut);

    Ok(())
}

async fn tui_task(client: Arc<VLPGroundStation<ThreadModeRawMutex>>) -> Result<()> {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();
    theme.palette = Palette::terminal_default();
    theme.palette[PaletteStyle::EditableTextCursor] = theme.palette[PaletteStyle::EditableText];
    theme.palette[PaletteStyle::EditableText] = theme.palette[PaletteStyle::Primary];
    siv.set_theme(theme);
    siv.set_autorefresh(true);

    let enter_uplink_loading_state = |s: &mut Cursive| {
        s.find_name::<HideableView<LinearLayout>>("uplink_buttons_hideable")
            .unwrap()
            .set_visible(false);
        s.find_name::<HideableView<TextView>>("uplink_sending_hideable")
            .unwrap()
            .set_visible(true);
    };

    let create_simple_packet_button =
        |button_text: &str, dialog_message: &str, packet: VLPUplinkPacket| {
            let client = client.clone();
            let button_text = button_text.to_string();
            let dialog_message = dialog_message.to_string();
            Button::new(button_text, move |s| {
                s.add_layer(
                    Dialog::around(TextView::new(dialog_message.clone()))
                        .dismiss_button("Cancel")
                        .button("Confirm", {
                            let client = client.clone();
                            let packet = packet.clone();
                            move |s| {
                                client.send_nb(packet.clone());
                                enter_uplink_loading_state(s);
                                s.pop_layer().unwrap();
                            }
                        }),
                );
            })
            .align_center_left()
        };

    let create_reset_device_button = || {
        let client = client.clone();
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
                    .button("Confirm", {
                        let client = client.clone();
                        move |s| {
                            let device_to_reset = reset_device_selection_group.selection();
                            client.send_nb(
                                ResetPacket {
                                    device: *device_to_reset,
                                }
                                .into(),
                            );
                            enter_uplink_loading_state(s);
                            s.pop_layer().unwrap();
                        }
                    }),
            );
        })
        .align_center_left()
    };

    let create_overwrite_amp_button = || {
        let client = client.clone();
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
                    .button("Confirm", {
                        let client = client.clone();
                        move |s| {
                            client.send_nb(
                                AMPOutputOverwritePacket {
                                    out1: *out1_selection_group.selection(),
                                    out2: *out2_selection_group.selection(),
                                    out3: *out3_selection_group.selection(),
                                    out4: *out4_selection_group.selection(),
                                }
                                .into(),
                            );
                            enter_uplink_loading_state(s);
                            s.pop_layer().unwrap();
                        }
                    }),
            );
        })
        .align_center_left()
    };

    let create_overwrite_eps_button = || {
        let client = client.clone();
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
                    .button("Confirm", {
                        let client = client.clone();
                        move |s| {
                            client.send_nb(
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
                            enter_uplink_loading_state(s);
                            s.pop_layer().unwrap();
                        }
                    }),
            );
        })
        .align_center_left()
    };

    siv.add_fullscreen_layer(
        LinearLayout::horizontal()
            .child(
                Panel::new(TextView::new("abc"))
                    .title("Ground Station Downlink")
                    .title_position(HAlign::Left)
                    .full_screen(),
            )
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
                                    .child(create_reset_device_button())
                                    .child(create_overwrite_amp_button())
                                    .child(create_overwrite_eps_button())
                                    .child(create_simple_packet_button(
                                        "Fire Main Pyro",
                                        "Manually fire main pyro?",
                                        FirePyroPacket {
                                            pyro: PyroSelect::Pyro1,
                                        }
                                        .into(),
                                    ))
                                    .child(create_simple_packet_button(
                                        "Fire Drogue Pyro",
                                        "Manually fire drogue pyro?",
                                        FirePyroPacket {
                                            pyro: PyroSelect::Pyro2,
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
            ),
    );

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));

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
                            .dismiss_button("OK"),
                    );
                }
            }
        }

        runner.step();
        interval.tick().await;
    }
    Ok(())
}
