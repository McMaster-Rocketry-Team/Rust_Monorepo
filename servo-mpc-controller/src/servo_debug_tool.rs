use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        mpsc::{Receiver, SyncSender, TryRecvError, sync_channel},
    },
    time::Duration,
};

use anyhow::Result;
use cursive::{
    theme::{Palette, PaletteStyle},
    view::Nameable,
    views::{LinearLayout, SliderView, TextView},
};
use dspower_servo::DSPowerServo;
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use embedded_hal_async::delay::DelayNs;
use tokio::{
    fs::OpenOptions,
    io::{AsyncWriteExt, BufWriter},
    runtime::Runtime,
    spawn,
    time::{self, Instant, interval, sleep},
};
use tokio_serial::SerialPortBuilderExt as _;

use crate::serial::SerialWrapper;
mod serial;

/// One telemetry sample
#[derive(Clone, Copy)]
struct Sample {
    t: f64,    // seconds since start
    cmd: f32,  // commanded angle  [-145, 145]
    act: f32,  // actual angle     [-145, 145]
    duty: f32, // duty cycle       [0, 1]
}

const MAX_POINTS: usize = 20 * 100; // 20 s × 100 Hz

async fn servo_worker(
    mut servo: DSPowerServo<SerialWrapper, Delay>,
    cmd_rx: Receiver<f32>,         // GUI → servo (std sync_channel)
    sample_tx: SyncSender<Sample>, // servo → GUI
    mut current_cmd: f32,          // last command seen
) {
    let start = Instant::now();
    let mut tk = interval(Duration::from_millis(10)); // 100 Hz

    loop {
        tk.tick().await;

        while let Ok(new_cmd) = cmd_rx.try_recv() {
            current_cmd = new_cmd;
        }

        let m = servo.batch_read_measurements().await.unwrap();
        sample_tx
            .try_send(Sample {
                t: start.elapsed().as_secs_f64(),
                cmd: current_cmd,
                act: m.angle,
                duty: m.pwm_duty_cycle,
            })
            .unwrap();

        servo.move_to(current_cmd).await.unwrap()
    }
}

struct Delay;

impl DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        sleep(Duration::from_nanos(ns as u64)).await;
    }
}

const MIN_ANGLE: f32 = -145.0;
const MAX_ANGLE: f32 = 145.0;

struct GuiApp {
    rx: Receiver<Sample>,    // telemetry
    cmd_tx: SyncSender<f32>, // GUI → servo
    buf: VecDeque<Sample>,   // sliding window
    slider: f32,             // commanded angle (deg)
}

impl GuiApp {
    fn new(rx: Receiver<Sample>, cmd_tx: SyncSender<f32>, initial_angle: f32) -> Self {
        Self {
            rx,
            cmd_tx,
            buf: VecDeque::with_capacity(MAX_POINTS),
            slider: initial_angle, // start slider at real servo angle
        }
    }

    fn ingest(&mut self) {
        loop {
            match self.rx.try_recv() {
                Ok(s) => {
                    if self.buf.len() == MAX_POINTS {
                        self.buf.pop_front();
                    }
                    self.buf.push_back(s);
                }
                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
            }
        }
    }
}

impl eframe::App for GuiApp {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        self.ingest();

        // ─── slider UI ───
        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            ui.spacing_mut().slider_width = ui.available_width();
            let changed = ui
                .add(
                    egui::Slider::new(&mut self.slider, -145.0..=145.0)
                        .step_by(0.1),
                )
                .changed();
            if changed {
                // non-blocking; drop if worker hasn’t consumed the previous value
                let _ = self.cmd_tx.try_send(self.slider);
            }
        });

        // ─── plots ───
        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("angles + duty")
                .include_y(-145.0)
                .include_y(145.0)
                .legend(Legend::default())
                .show(ui, |plot_ui| {
                    let cmd:  PlotPoints = self.buf.iter().map(|s| [s.t, s.cmd  as f64]).collect();
                    let act:  PlotPoints = self.buf.iter().map(|s| [s.t, s.act  as f64]).collect();
                    let duty: PlotPoints = self.buf.iter().map(|s| [s.t, s.duty as f64 * 100.0]).collect();

                    plot_ui.line(Line::new("commanded", cmd ));
                    plot_ui.line(Line::new("actual", act ));
                    plot_ui.line(Line::new("duty cycle", duty)); // values 0‥1
                });
        });

        ctx.request_repaint_after(Duration::from_millis(16)); // ~60 fps
    }
}

fn main() -> eframe::Result<()> {
    // std-lib channels
    let (sample_tx, sample_rx) = sync_channel::<Sample>(MAX_POINTS);
    let (cmd_tx, cmd_rx) = sync_channel::<f32>(1); // cap 1 → latest wins

    // dedicated Tokio runtime (GUI stays on the main thread)
    let rt = Runtime::new().expect("tokio runtime");

    // initialise the servo inside the runtime so we can `.await`
    let initial_angle = rt.block_on(async {
        let serial = tokio_serial::new("/dev/ttyUSB0", 115200)
            .open_native_async()
            .expect("open serial port");

        let mut servo = DSPowerServo::new(SerialWrapper(serial), Delay);
        servo.reset().await.unwrap();
        servo.init(true).await.unwrap();

        // first measurement gives us the true starting angle
        let m = servo.batch_read_measurements().await.unwrap();
        let angle0 = m.angle;

        // kick off the long-running async task
        tokio::spawn(servo_worker(servo, cmd_rx, sample_tx, angle0));

        angle0
    });

    // fire up egui/eframe
    eframe::run_native(
        "Servo live plot",
        eframe::NativeOptions::default(),
        Box::new(|_| Ok(Box::new(GuiApp::new(sample_rx, cmd_tx, initial_angle)))),
    )
}
