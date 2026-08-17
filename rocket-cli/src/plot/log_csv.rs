//! Reading a downloaded flight-log CSV back into columns.
//!
//! Everything here is keyed by *header name*, never by column index. That is not
//! defensiveness for its own sake: `source_block_crc_failed` and
//! `slow_timestamp_us` were inserted into the middle of the header rather than
//! appended, so a log downloaded before that change has 81 columns and one
//! downloaded after has 83, with nine columns' worth of shift between them. Both
//! must plot. A name-keyed reader also means a future column can be added
//! anywhere without touching this file.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result, bail};

/// One flight-log CSV, stored column-wise.
///
/// Column-wise because every consumer wants one series at a time across all
/// rows, and because 225k rows × 80 columns of row structs is a great deal of
/// pointer chasing to answer "what did altitude do".
pub struct FlightLog {
    /// Per-row logger sequence number. Kept exact (not `f32`) because session
    /// splitting turns on `<` between adjacent values, and at 427 Hz a long log
    /// passes 2^24 where `f32` stops being able to represent consecutive
    /// integers — the point at which every comparison would silently read equal.
    pub record_count: Vec<u32>,
    /// Boot-relative time. `f64` holds microseconds exactly well past any
    /// realistic log length.
    pub timestamp_us: Vec<f64>,
    /// `FlightStage` discriminant, or `None` where the cell was blank or held a
    /// name this build does not know.
    pub stage: Vec<Option<u8>>,
    /// Every other column that parses as a number, absent cells as `NaN`.
    columns: HashMap<String, Vec<f32>>,
    pub row_count: usize,
    /// Rows whose `source_block_crc_failed` was true. Absent from older logs,
    /// which is not the same as zero — hence the `Option`.
    pub crc_failed_rows: Option<usize>,
}

/// Parse one cell into a plottable number.
///
/// The CSV mixes three spellings of the same idea. Booleans are written by
/// `bit()` as `true`/`false`, but `air_brakes_validation_deploy` goes through
/// `cell(... as u8)` and lands as `0`/`1`, and enum-valued columns arrive as
/// names. Mapping all of them onto a float here is what lets one decimator and
/// one renderer serve every panel; the alternative is a per-column type tag that
/// only ever gets used to do this conversion later.
///
/// An empty cell becomes `NaN` rather than `0.0`. That distinction is the whole
/// reason the exporter stopped writing `*_valid` columns, and collapsing it here
/// would put a Mach-lockout sample on the chart as a plunge to zero.
fn parse_cell(raw: &str) -> f32 {
    match raw {
        "" => f32::NAN,
        "true" | "True" => 1.0,
        "false" | "False" => 0.0,
        other => other.parse::<f32>().unwrap_or(f32::NAN),
    }
}

/// Map a `FlightStage` debug name to its discriminant.
///
/// Written out rather than derived because the CSV holds `{:?}` output from
/// firmware that may be a different version than this CLI, so an unrecognised
/// name has to be survivable.
fn parse_stage(raw: &str) -> Option<u8> {
    Some(match raw {
        "LowPower" => 0,
        "SelfTest" => 1,
        "Armed" => 2,
        "Ascent" => 3,
        "DrogueChute" => 4,
        "MainChute" => 5,
        "Landed" => 6,
        "FailedToReachMinApogee" => 7,
        _ => return None,
    })
}

/// Columns that get their own field on [`FlightLog`] and so are skipped by the
/// generic float path.
const SPECIAL: [&str; 3] = ["record_count", "timestamp_us", "flight_stage"];

impl FlightLog {
    pub fn load(path: &Path) -> Result<Self> {
        let mut reader = csv::ReaderBuilder::new()
            .flexible(true)
            .from_path(path)
            .with_context(|| format!("opening {}", path.display()))?;

        let headers: Vec<String> = reader
            .headers()
            .context("reading the CSV header row")?
            .iter()
            .map(str::to_owned)
            .collect();

        if !headers.iter().any(|h| h == "timestamp_us") {
            bail!(
                "{} has no `timestamp_us` column — is it a flight log downloaded \
                 by `rocket-cli download-flight-log`?",
                path.display()
            );
        }

        let mut record_count = Vec::new();
        let mut timestamp_us = Vec::new();
        let mut stage = Vec::new();
        let mut columns: HashMap<String, Vec<f32>> = headers
            .iter()
            .filter(|h| !SPECIAL.contains(&h.as_str()))
            .map(|h| (h.clone(), Vec::new()))
            .collect();
        let mut crc_failed = 0usize;
        let mut saw_crc_column = false;

        let mut row = csv::StringRecord::new();
        while reader
            .read_record(&mut row)
            .with_context(|| format!("reading {}", path.display()))?
        {
            // A short row is a truncated final line — a download interrupted
            // mid-write. Pad rather than reject: the rows before it are a real
            // flight and refusing the whole file over its tail helps nobody.
            for (i, name) in headers.iter().enumerate() {
                let raw = row.get(i).unwrap_or("");
                match name.as_str() {
                    "record_count" => record_count.push(raw.parse::<u32>().unwrap_or(0)),
                    "timestamp_us" => timestamp_us.push(raw.parse::<f64>().unwrap_or(f64::NAN)),
                    "flight_stage" => stage.push(parse_stage(raw)),
                    _ => {
                        if name == "source_block_crc_failed" {
                            saw_crc_column = true;
                            if raw == "true" {
                                crc_failed += 1;
                            }
                        }
                        // `expect` is sound: the map was built from `headers`.
                        columns
                            .get_mut(name)
                            .expect("column map built from these headers")
                            .push(parse_cell(raw));
                    }
                }
            }
        }

        let row_count = timestamp_us.len();
        if row_count == 0 {
            bail!("{} has a header but no rows", path.display());
        }

        // A file with no `flight_stage` column at all still has to line up.
        if stage.len() != row_count {
            stage.resize(row_count, None);
        }
        if record_count.len() != row_count {
            record_count.resize(row_count, 0);
        }

        Ok(Self {
            record_count,
            timestamp_us,
            stage,
            columns,
            row_count,
            crc_failed_rows: saw_crc_column.then_some(crc_failed),
        })
    }

    /// A column by name, or `None` if this log does not have it.
    ///
    /// Returning `None` for a missing column rather than an all-`NaN` one lets
    /// the renderer tell "this firmware never wrote that field" apart from "it
    /// wrote it and it was always absent" — different messages for the reader.
    pub fn column(&self, name: &str) -> Option<&[f32]> {
        self.columns.get(name).map(Vec::as_slice)
    }
}

/// Shared by the tests in the sibling plotting modules, which all need a
/// `FlightLog` built from a literal rather than from a device.
#[cfg(test)]
pub mod test_support {
    use super::FlightLog;
    use std::io::Write;

    /// Write `body` to a uniquely named temp file and load it. `name` must be
    /// distinct per call site so tests running in parallel cannot collide on
    /// the same path.
    pub fn write_temp(name: &str, body: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("rocket_cli_{name}.csv"));
        std::fs::File::create(&path)
            .unwrap()
            .write_all(body.as_bytes())
            .unwrap();
        path
    }

    pub fn log_from_csv(name: &str, body: &str) -> FlightLog {
        FlightLog::load(&write_temp(name, body)).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::write_temp;
    use super::*;

    /// The reason this reader is name-keyed. These two headers describe the same
    /// three fields, but `source_block_crc_failed` sits between them in one and
    /// not the other, so every index past column 1 disagrees. An index-keyed
    /// reader plots `timestamp_us` as the CRC flag and never says a word.
    #[test]
    fn old_and_new_column_orders_both_land_on_the_right_field() {
        let old = write_temp(
            "plot_old_header",
            "record_count,timestamp_us,pressure\n7,1000,101325\n",
        );
        let new = write_temp(
            "plot_new_header",
            "record_count,source_block_crc_failed,timestamp_us,pressure\n\
             7,false,1000,101325\n",
        );

        let old = FlightLog::load(&old).unwrap();
        let new = FlightLog::load(&new).unwrap();

        for log in [&old, &new] {
            assert_eq!(log.record_count[0], 7);
            assert_eq!(log.timestamp_us[0], 1000.0);
            assert_eq!(log.column("pressure").unwrap()[0], 101325.0);
        }
        // And the flag itself is only counted where it exists.
        assert_eq!(old.crc_failed_rows, None);
        assert_eq!(new.crc_failed_rows, Some(0));
    }

    /// An absent cell must stay absent all the way to the chart. If it became
    /// `0.0` here, a frozen deployment filter would draw as the rocket
    /// instantaneously at sea level doing zero — a reading, not a gap.
    #[test]
    fn an_empty_cell_is_nan_and_never_zero() {
        let path = write_temp(
            "plot_empty_cell",
            "timestamp_us,deployment_kf_altitude_asl\n1000,\n2000,150.5\n",
        );
        let log = FlightLog::load(&path).unwrap();
        let alt = log.column("deployment_kf_altitude_asl").unwrap();
        assert!(alt[0].is_nan());
        assert_eq!(alt[1], 150.5);
    }

    /// The three spellings of a flag that the exporter actually emits.
    #[test]
    fn bools_and_numeric_bools_both_read_as_one_and_zero() {
        let path = write_temp(
            "plot_bool_forms",
            "timestamp_us,pyro_main_fire,air_brakes_validation_deploy\n\
             1000,true,1\n2000,false,0\n",
        );
        let log = FlightLog::load(&path).unwrap();
        assert_eq!(log.column("pyro_main_fire").unwrap(), &[1.0, 0.0]);
        assert_eq!(
            log.column("air_brakes_validation_deploy").unwrap(),
            &[1.0, 0.0]
        );
    }

    #[test]
    fn stage_names_map_to_discriminants_and_unknowns_survive() {
        let path = write_temp(
            "plot_stages",
            "timestamp_us,flight_stage\n1000,Armed\n2000,Ascent\n3000,SomethingNewer\n",
        );
        let log = FlightLog::load(&path).unwrap();
        assert_eq!(log.stage, vec![Some(2), Some(3), None]);
    }

    /// A download cut off mid-row leaves a short final line. The flight before
    /// it is still worth plotting.
    #[test]
    fn a_truncated_final_row_pads_instead_of_failing_the_file() {
        let path = write_temp(
            "plot_truncated",
            "timestamp_us,pressure,temperature\n1000,101325,21.5\n2000,101300\n",
        );
        let log = FlightLog::load(&path).unwrap();
        assert_eq!(log.row_count, 2);
        assert_eq!(log.column("pressure").unwrap()[1], 101300.0);
        assert!(log.column("temperature").unwrap()[1].is_nan());
    }
}
