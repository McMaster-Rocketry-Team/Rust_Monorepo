mod rpc_radio;

use std::time::Duration;

use anyhow::Result;
use cursive::{
    align::HAlign,
    theme::{Palette, PaletteStyle},
    view::Resizable,
    views::{Button, Dialog, LinearLayout, PaddedView, Panel, RadioGroup, TextView},
};
use cursive_aligned_view::Alignable;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use firmware_common_new::{
    can_bus::messages::amp_overwrite::PowerOutputOverwrite,
    rpc::lora_rpc::LoraRpcClient,
    vlp::{
        client::VLPGroundStation,
        lora_config::LoraConfig,
        packets::{
            fire_pyro::{FirePyroPacket, PyroSelect},
            reset::DeviceToReset,
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
    tui_task().await;
    return Ok(());
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

    let vlp_gcm_client = VLPGroundStation::<NoopRawMutex>::new();

    let mut daemon = vlp_gcm_client.daemon(&mut rpc_radio, &VLP_KEY);

    let daemon_fut = daemon.run();

    // let (mut rl, mut writer) =
    //     Readline::new("Select pyro to fire (main, drogue): ".to_owned()).unwrap();

    let print_telemetry_fut = async {
        loop {
            let (packet, status) = vlp_gcm_client.receive().await;
            info!("{:?} rssi={} snr={}\n", packet, status.rssi, status.snr);
        }
    };

    let test_fut = async {
        vlp_gcm_client
            .send(
                FirePyroPacket {
                    pyro: PyroSelect::Pyro1,
                }
                .into(),
            )
            .await
            .unwrap();
    };

    tokio::join!(daemon_fut, print_telemetry_fut, test_fut);

    Ok(())
}

async fn tui_task() -> Result<()> {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();
    theme.palette = Palette::terminal_default();
    theme.palette[PaletteStyle::EditableTextCursor] = theme.palette[PaletteStyle::EditableText];
    theme.palette[PaletteStyle::EditableText] = theme.palette[PaletteStyle::Primary];
    siv.set_theme(theme);
    siv.set_autorefresh(true);

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
                            Button::new("Low Power Mode", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new(
                                        "Change rocket to low power mode?",
                                    ))
                                    .dismiss_button("Cancel")
                                    .button("Confirm", |s| {
                                        s.pop_layer().unwrap();
                                    }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Self Test Mode", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new(
                                        "Change rocket to self test mode?",
                                    ))
                                    .dismiss_button("Cancel")
                                    .button("Confirm", |s| {
                                        s.pop_layer().unwrap();
                                    }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Armed Mode", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new("Change rocket to armed mode?"))
                                        .dismiss_button("Cancel")
                                        .button("Confirm", |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Landed Mode", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new("Change rocket to landed mode?"))
                                        .dismiss_button("Cancel")
                                        .button("Confirm", |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Reset Device", |s| {
                                let mut reset_device_selection_group: RadioGroup<DeviceToReset> =
                                    RadioGroup::new();

                                s.add_layer(
                                    Dialog::new()
                                        .title("Select a device to reset")
                                        .content(
                                            LinearLayout::vertical()
                                                .child(
                                                    reset_device_selection_group
                                                        .button(DeviceToReset::All, "All"),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::VoidLake,
                                                        "Void Lake",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group
                                                        .button(DeviceToReset::AMP, "AMP"),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::AMPOut1,
                                                        "AMP Out 1",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::AMPOut2,
                                                        "AMP Out 2",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::AMPOut3,
                                                        "AMP Out 3",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::AMPOut4,
                                                        "AMP Out 4",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group
                                                        .button(DeviceToReset::Icarus, "ICARUS"),
                                                )
                                                .child(reset_device_selection_group.button(
                                                    DeviceToReset::PayloadActivationPCB,
                                                    "Payload Activation PCB",
                                                ))
                                                .child(reset_device_selection_group.button(
                                                    DeviceToReset::RocketWifi,
                                                    "Rocket WiFi",
                                                ))
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::OzysAll,
                                                        "OZYS (All)",
                                                    ),
                                                )
                                                .child(reset_device_selection_group.button(
                                                    DeviceToReset::MainBulkhead,
                                                    "Main Bulkhead PCB",
                                                ))
                                                .child(reset_device_selection_group.button(
                                                    DeviceToReset::DrogueBulkhead,
                                                    "Drogue Bulkhead PCB",
                                                ))
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::PayloadEPS1,
                                                        "EPS 1",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::PayloadEPS2,
                                                        "EPS 2",
                                                    ),
                                                )
                                                .child(
                                                    reset_device_selection_group.button(
                                                        DeviceToReset::AeroRust,
                                                        "AeroRust",
                                                    ),
                                                ),
                                        )
                                        .dismiss_button("Cancel")
                                        .button("Confirm", move |s| {
                                            let device_to_reset =
                                                reset_device_selection_group.selection();
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Overwrite AMP", |s| {
                                let mut out1_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut out2_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut out3_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut out4_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();

                                s.add_layer(
                                    Dialog::new()
                                        .title("Overwrite AMP")
                                        .content(
                                            LinearLayout::vertical()
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("Out 1:  "))
                                                        .child(out1_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out1_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out1_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("Out 2:  "))
                                                        .child(out2_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out2_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out2_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("Out 3:  "))
                                                        .child(out3_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out3_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out3_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("Out 4:  "))
                                                        .child(out4_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out4_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(out4_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                ),
                                        )
                                        .dismiss_button("Cancel")
                                        .button("Confirm", move |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Overwrite EPS", |s| {
                                let mut eps1_3v3_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut eps1_5v_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut eps1_9v_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut eps2_3v3_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut eps2_5v_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();
                                let mut eps2_9v_selection_group: RadioGroup<PowerOutputOverwrite> =
                                    RadioGroup::new();

                                s.add_layer(
                                    Dialog::new()
                                        .title("Overwrite EPS")
                                        .content(
                                            LinearLayout::vertical()
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 1 3.3V:  "))
                                                        .child(eps1_3v3_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_3v3_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_3v3_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 1   5V:  "))
                                                        .child(eps1_5v_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_5v_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_5v_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 1   9V:  "))
                                                        .child(eps1_9v_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_9v_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps1_9v_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(TextView::new(" "))
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 2 3.3V:  "))
                                                        .child(eps2_3v3_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_3v3_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_3v3_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 2   5V:  "))
                                                        .child(eps2_5v_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_5v_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_5v_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                )
                                                .child(
                                                    LinearLayout::horizontal()
                                                        .child(TextView::new("EPS 2   9V:  "))
                                                        .child(eps2_9v_selection_group.button(
                                                            PowerOutputOverwrite::NoOverwrite,
                                                            "No Overwrite",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_9v_selection_group.button(
                                                            PowerOutputOverwrite::ForceEnabled,
                                                            "Enable",
                                                        ))
                                                        .child(TextView::new("  "))
                                                        .child(eps2_9v_selection_group.button(
                                                            PowerOutputOverwrite::ForceDisabled,
                                                            "Disable",
                                                        )),
                                                ),
                                        )
                                        .dismiss_button("Cancel")
                                        .button("Confirm", move |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                )
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Fire Main Pyro", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new("Manually fire main pyro?"))
                                        .dismiss_button("Cancel")
                                        .button("Confirm", |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        )
                        .child(
                            Button::new("Fire Drogue Pyro", |s| {
                                s.add_layer(
                                    Dialog::around(TextView::new("Manually fire drogue pyro?"))
                                        .dismiss_button("Cancel")
                                        .button("Confirm", |s| {
                                            s.pop_layer().unwrap();
                                        }),
                                );
                            })
                            .align_center_left(),
                        ),
                ))
                .title("Send Uplink")
                .full_height(),
            ),
    );

    let mut runner = siv.runner();
    runner.refresh();
    let mut interval = time::interval(Duration::from_millis(1000 / 30));

    while runner.is_running() {
        // TODO

        runner.step();
        interval.tick().await;
    }
    Ok(())
}
