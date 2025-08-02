use std::{
    collections::HashMap,
    sync::{LazyLock, RwLock},
};

use plotters::{
    chart::ChartBuilder,
    prelude::{BitMapBackend, IntoDrawingArea as _, PathElement},
    series::LineSeries,
    style::{BLACK, Color as _, RED, WHITE},
};

struct GlobalPlotData {
    time: f32,
    points: HashMap<String, Vec<(f32, f32)>>,
}

static PLOT_DATA: LazyLock<RwLock<GlobalPlotData>> = LazyLock::new(|| {
    RwLock::new(GlobalPlotData {
        time: 0.0,
        points: HashMap::new(),
    })
});

pub struct GlobalPlot {}

impl GlobalPlot {
    pub fn set_time(time: f32) {
        let mut plot_data = PLOT_DATA.write().unwrap();
        plot_data.time = time;
    }

    pub fn add_value(name: &str, value: f32) {
        let mut plot_data = PLOT_DATA.write().unwrap();
        let time = plot_data.time;
        let entry = plot_data.points.entry(name.into()).or_default();
        entry.push((time, value));
    }

    pub fn plot_all() {
        // Clean up and recreate plots_out directory
        if std::path::Path::new("plots_out").exists() {
            std::fs::remove_dir_all("plots_out").unwrap();
        }
        std::fs::create_dir_all("plots_out").unwrap();

        let plot_data = PLOT_DATA.write().unwrap();

        for (data_name, data) in plot_data.points.iter() {
            plot_graph(data_name, data).unwrap();
        }
    }
}

fn plot_graph(data_name: &str, data: &Vec<(f32, f32)>) -> Result<(), Box<dyn std::error::Error>> {
    let file_path = format!(
        "plots_out/{}_vs_time.png",
        data_name.to_lowercase().replace(" ", "_")
    );
    let root = BitMapBackend::new(&file_path, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let time_range = min_max_range(&data.iter().map(|(t, _)| *t).collect::<Vec<f32>>());
    let value_range = min_max_range(&data.iter().map(|(_, a)| *a).collect::<Vec<f32>>());
    log_info!("value range for {}: {:?}", data_name, value_range);

    let mut chart = ChartBuilder::on(&root)
        .caption(format!("{data_name} vs Time"), ("sans-serif", 40))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(time_range, value_range)?;

    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc(data_name)
        .draw()?;

    chart
        .draw_series(LineSeries::new(data.iter().cloned(), &RED))?
        .label(data_name)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;
    log_info!("Plot saved as {}", &file_path);
    Ok(())
}

fn min_max_range(values: &[f32]) -> std::ops::Range<f32> {
    let min = values.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max = values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    min..max
}
