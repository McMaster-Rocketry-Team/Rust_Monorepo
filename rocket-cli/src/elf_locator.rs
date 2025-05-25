use std::{
    collections::{BTreeMap, HashMap},
    fs::{self, File},
    io::Read as _,
    path::{Path, PathBuf},
    pin::Pin,
    time::SystemTime,
};

use anyhow::Result;
use defmt_decoder::{Location, Table};
use log::{debug, info, warn};
use pad::PadStr;

use crate::args::NodeTypeEnum;

#[derive(Debug)]
pub struct DefmtElfInfo {
    pub table: Table,
    pub locs: Option<BTreeMap<u64, Location>>,
}

pub type ELFInfoMap = HashMap<NodeTypeEnum, Pin<Box<DefmtElfInfo>>>;

pub fn locate_elf_files(firmware_elf_path: Option<&PathBuf>) -> Result<ELFInfoMap> {
    let mut possible_paths: Vec<String> = vec![
        "./Rust_Monorepo".into(),
        "../Rust_Monorepo".into(),
        "../../Rust_Monorepo".into(),
    ];
    if let Some(firmware_elf_path) = firmware_elf_path {
        possible_paths.push(
            firmware_elf_path
                .join("../../../../../Rust_Monorepo")
                .to_str()
                .unwrap()
                .into(),
        );
        possible_paths.push(
            firmware_elf_path
                .join("../../../../../../Rust_Monorepo")
                .to_str()
                .unwrap()
                .into(),
        );
    }

    let monorepo_path = possible_paths
        .iter()
        .cloned()
        .filter_map(|path| fs::canonicalize(path).ok())
        .next()
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Could not find Rust_Monorepo directory, looked in {:?}",
                possible_paths
            )
        })?;

    let rocketry_path = monorepo_path.parent().unwrap();
    info!("Found rocketry root path: {:?}", rocketry_path);

    let mut result = HashMap::new();

    info!("Locating firmware ELF files.....");

    let mut find_elf_and_add_to_map = |node_type: NodeTypeEnum, path: &str| -> Result<()> {
        let path = rocketry_path.join(Path::new(path));
        if let Some(elf) = find_newest_elf(&path)? {
            let a = format!("ELF for {:?} found:", node_type).pad_to_width(25);
            let b = format!(
                "{} ({})",
                elf.path.file_name().unwrap().to_str().unwrap(),
                elf.profile,
            )
            .pad_to_width(20);
            info!(
                "{}{} built at {}",
                a,
                b,
                chrono::DateTime::<chrono::Local>::from(elf.created_time)
                    .format("%Y-%m-%d %H:%M:%S")
                    .to_string()
            );

            let bytes = fs::read(elf.path)?;
            let table = if let Ok(Some(table)) = Table::parse(&bytes) {
                table
            } else {
                warn!("Failed to parse defmt table");
                return Ok(());
            };
            let locs = table.get_locations(&bytes)?;
            let locs = if table.indices().all(|idx| locs.contains_key(&(idx as u64))) {
                Some(locs)
            } else {
                warn!("Location info is incomplete, it will be omitted from the output");
                None
            };
            result.insert(node_type, Box::pin(DefmtElfInfo { table, locs }));
        } else {
            warn!("ELF for {:?} not found, looked in {:?}", node_type, path);
        }

        Ok(())
    };

    find_elf_and_add_to_map(NodeTypeEnum::VoidLake, "VLF5/firmware")?;
    find_elf_and_add_to_map(NodeTypeEnum::AMP, "Titan_AMP/firmware")?;
    find_elf_and_add_to_map(NodeTypeEnum::ICARUS, "ICARUS/firmware")?;
    find_elf_and_add_to_map(NodeTypeEnum::OZYS, "OZYS_V3/firmware")?;
    find_elf_and_add_to_map(NodeTypeEnum::Bulkhead, "Titan_Bulkhead_PCB/firmware")?;

    Ok(result)
}

#[derive(Debug, Clone)]
struct ElfInfo {
    path: PathBuf,
    profile: String,
    created_time: SystemTime,
}

fn find_newest_elf<P: AsRef<Path>>(project_path: &P) -> Result<Option<ElfInfo>> {
    let pattern = format!(
        "{}/target/**/{{debug,release}}/*",
        project_path.as_ref().to_str().unwrap()
    );
    let elf = globwalk::glob(&pattern)?
        .filter_map(|res| {
            let path = res.ok()?;
            let path = path.into_path();
            if !path.is_file() {
                return None;
            }
            let file_name = path.file_name()?.to_str()?;
            if file_name.contains(".") {
                return None;
            }
            if !is_elf(&path).ok()? {
                return None;
            }

            Some(ElfInfo {
                created_time: fs::metadata(&path).ok()?.created().ok()?,
                profile: path.parent()?.file_name()?.to_str()?.into(),
                path,
            })
        })
        .max_by_key(|info| info.created_time)
        .into_iter()
        .next();

    Ok(elf)
}

fn is_elf<P: AsRef<Path>>(path: P) -> Result<bool> {
    let mut file = File::open(path)?;
    let mut magic = [0u8; 4];
    let n = file.read(&mut magic)?;
    Ok(n == 4 && magic == [0x7F, b'E', b'L', b'F'])
}

#[cfg(test)]
mod test {
    use log::LevelFilter;

    use super::*;

    #[test]
    fn test_locate_elf_files() {
        let _ = env_logger::builder()
            .filter_level(LevelFilter::Info)
            .try_init();
        locate_elf_files(Some(&PathBuf::from("./elf"))).unwrap();
    }
}
