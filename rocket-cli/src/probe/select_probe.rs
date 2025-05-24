use anyhow::{Result, bail};
use probe_rs::probe::DebugProbeInfo;
use prompted::input;

pub async fn select_probe(probes: Vec<DebugProbeInfo>) -> Result<String> {
    let probe = if probes.len() == 1 {
        probes[0].clone()
    } else {
        for i in 0..probes.len() {
            let probe = &probes[i];

            println!(
                "[{}]: {}, SN {}",
                i + 1,
                probe.identifier,
                probe.serial_number.clone().unwrap_or("N/A".into())
            );
        }

        let selection = input!("Select one probe (1-{}): ", probes.len());

        let selection: usize = match selection.trim().parse() {
            Err(_) => bail!("Invalid selection"),
            Ok(num) if num > probes.len() => bail!("Invalid selection"),
            Ok(num) => num,
        };

        probes[selection].clone()
    };

    let probe_string = format!(
        "{:x}:{:x}{}",
        probe.vendor_id,
        probe.product_id,
        probe
            .serial_number
            .map_or(String::new(), |sn| format!(":{}", sn))
    );

    Ok(probe_string)
}
