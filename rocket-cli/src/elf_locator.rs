use std::{
    fs::{self, File},
    io::Read as _,
    path::{Path, PathBuf},
    time::SystemTime,
};

use anyhow::Result;

#[derive(Debug, Clone)]
pub struct ElfInfo {
    pub path: PathBuf,
    pub profile: String,
    pub created_time: SystemTime,
}

pub fn find_newest_elf<P: AsRef<Path>>(project_path: &P) -> Result<Option<ElfInfo>> {
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
