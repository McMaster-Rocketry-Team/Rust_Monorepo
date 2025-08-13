use std::{
    collections::VecDeque,
    sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel},
    time::Duration,
};

use dspower_servo::{DSPowerServo, ServoSlidingModeController};
use eframe::egui::{self, Vec2, Vec2b};
use egui_plot::{Corner, Legend, Line, Plot, PlotPoints};
use embedded_hal_async::delay::DelayNs;
use log::LevelFilter;
use tokio::{
    runtime::Runtime,
    time::{Instant, interval, sleep},
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
    mut servo: DSPowerServo<'_, SerialWrapper>,
    cmd_rx: Receiver<f32>,         // GUI → servo (std sync_channel)
    sample_tx: SyncSender<Sample>, // servo → GUI
    mut current_cmd: f32,          // last command seen
) {
    let mut controller = ServoSlidingModeController::new(&mut servo, -145.0..145.0, 3.5, 3.0);
    let start = Instant::now();
    let mut tk = interval(Duration::from_millis(10)); // 100 Hz

    loop {
        tk.tick().await;

        while let Ok(new_cmd) = cmd_rx.try_recv() {
            current_cmd = new_cmd;
        }

        let m = controller.step(current_cmd).await.unwrap();
        sample_tx
            .try_send(Sample {
                t: start.elapsed().as_secs_f64(),
                cmd: current_cmd,
                act: m.angle,
                duty: m.pwm_duty_cycle,
            })
            .unwrap();
    }
}

struct Delay;

impl DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        sleep(Duration::from_nanos(ns as u64)).await;
    }
}
struct GuiApp {
    rx: Receiver<Sample>,    // telemetry
    cmd_tx: SyncSender<f32>, // GUI → servo
    buf: VecDeque<Sample>,   // sliding window
    slider: f32,             // commanded angle
    paused: bool,            // pause flag
}

impl GuiApp {
    fn new(rx: Receiver<Sample>, cmd_tx: SyncSender<f32>, initial_angle: f32) -> Self {
        Self {
            rx,
            cmd_tx,
            buf: VecDeque::with_capacity(MAX_POINTS),
            slider: initial_angle,
            paused: false,
        }
    }

    /// Pull everything from the channel; keep or discard depending on pause.
    fn ingest(&mut self) {
        loop {
            match self.rx.try_recv() {
                Ok(s) => {
                    if !self.paused {
                        if self.buf.len() == MAX_POINTS {
                            self.buf.pop_front();
                        }
                        self.buf.push_back(s);
                    }
                    // if paused: discard sample → channel won’t clog
                }
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => break,
            }
        }
    }
}

impl eframe::App for GuiApp {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        self.ingest();

        // ─── controls panel ───
        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            // 1) Pause/Resume button (first row)
            let lbl = if self.paused { "Resume" } else { "Pause" };
            if ui.button(lbl).clicked() {
                self.paused = !self.paused;
            }

            ui.add_space(4.0); // small gap

            // 2) full-width slider (second row)
            ui.spacing_mut().slider_width = ui.available_width();
            let changed = ui
                .add(
                    egui::Slider::new(&mut self.slider, -145.0..=145.0)
                        .step_by(0.1)
                        .text("Commanded angle (°)"),
                )
                .changed();
            if changed {
                let _ = self.cmd_tx.try_send(self.slider);
            }
        });

        // ─── combined plot ───
        let current_buf = &self.buf; // move a reference into the closure

        let coord_fmt = egui_plot::CoordinatesFormatter::new(move |point, _| {
            // find the sample closest in time to the cursor
            if let Some(s) = current_buf.iter().min_by(|a, b| {
                (a.t - point.x)
                    .abs()
                    .partial_cmp(&(b.t - point.x).abs())
                    .unwrap()
            }) {
                // build one multiline string that shows *all* series values
                format!(
                    "t = {:>6.2}s\ncommanded = {:>6.1}°\nactual = {:>6.1} °\nduty = {:>5.1}%",
                    s.t,
                    s.cmd,
                    s.act,
                    s.duty * 100.0
                )
            } else {
                "".to_owned()
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("angles + duty")
                .allow_zoom(Vec2b::new(false, false))
                .allow_boxed_zoom(false)
                .allow_scroll(Vec2b::new(false, false))
                .allow_drag(Vec2b::new(false, false))
                .default_y_bounds(-145.0, 145.0)
                .default_x_bounds(
                    self.buf.back().map_or(0.0, |s| s.t) - 20.0,
                    self.buf.back().map_or(0.0, |s| s.t),
                )
                .set_margin_fraction(Vec2::new(0.0, 0.0))
                .legend(Legend::default().position(Corner::LeftTop))
                .coordinates_formatter(Corner::LeftBottom, coord_fmt)
                .show(ui, |plot_ui| {
                    let cmd: PlotPoints = self.buf.iter().map(|s| [s.t, s.cmd as f64]).collect();
                    let act: PlotPoints = self.buf.iter().map(|s| [s.t, s.act as f64]).collect();
                    let duty: PlotPoints = self
                        .buf
                        .iter()
                        .map(|s| [s.t, s.duty as f64 * 100.0]) // scale 0–1 → 0–100
                        .collect();

                    plot_ui.line(Line::new("commanded", cmd));
                    plot_ui.line(Line::new("actual", act));
                    plot_ui.line(Line::new("duty %", duty));
                });
        });

        ctx.request_repaint_after(Duration::from_millis(16)); // ~60 fps
    }
}

fn main() -> eframe::Result<()> {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .try_init()
        .unwrap();

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

        let serial = Box::leak(Box::new(SerialWrapper(serial)));
        let mut servo = DSPowerServo::new(serial);
        servo.reset(&mut Delay).await.unwrap();
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
