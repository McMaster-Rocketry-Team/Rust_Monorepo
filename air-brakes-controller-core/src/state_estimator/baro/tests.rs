use std::fs::File;

use super::*;
use crate::tests::{init_logger, plot::GlobalPlot};
use csv::Reader;
use firmware_common_new::time::BootTimestamp;
use icao_isa::calculate_isa_altitude;
use icao_units::si::Pascals;
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Deserialize)]
struct CsvRecord {
    timestamp_s: f32,
    altitude: f32,
    air_pressure_noisy: f32,
}

fn read_csv_records() -> Vec<CsvRecord> {
    let path = "./test_data/merged.csv";
    let mut reader = Reader::from_reader(File::open(path).unwrap());
    reader.deserialize().map(|row| row.unwrap()).collect()
}

#[test]
fn integration_test() {
    init_logger();

    let csv_records = read_csv_records();

    let mut estimator = BaroStateEstimator::new(FlightProfile {
        drogue_chute_minimum_time_us: 0,
        drogue_chute_minimum_altitude_agl: 2000.0,
        drogue_chute_delay_us: 0,
        main_chute_altitude_agl: 400.0,
        main_chute_delay_us: 0,
        time_above_mach_08_us: 5_000_000,
    });
    for csv_record in csv_records.iter() {
        GlobalPlot::set_time(csv_record.timestamp_s);
        let altitude_asl =
            calculate_isa_altitude(Pascals(csv_record.air_pressure_noisy as f64)).0 as f32;
        let reading: SensorReading<BootTimestamp, Measurement> = SensorReading::new(
            (csv_record.timestamp_s as f64 * 1000_000.0) as u64,
            Measurement::new(&Vector3::zeros(), &Vector3::zeros(), altitude_asl),
        );

        estimator.update(&reading);

        GlobalPlot::add_value("Real Altitude", csv_record.altitude);
        GlobalPlot::add_value(
            "Estimated Altitude",
            estimator.altitude_agl().unwrap_or_default(),
        );
        GlobalPlot::add_value(
            "Estimated Velocity",
            estimator.velocity().unwrap_or_default(),
        );
        GlobalPlot::add_value(
            "Estimated Stddev",
            estimator.altitude_variance().unwrap_or_default().sqrt(),
        );
    }

    GlobalPlot::plot_all();
}
