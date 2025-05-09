use async_stream::stream;
use firmware_common::common::serialized_enum::SerializedEnumReader;
use futures_core::Stream;
use std::path::PathBuf;
use tokio::{fs::File, io::BufReader};

use anyhow::Result;

use super::readers::BufReaderWrapper;

pub async fn parse_serialized_enums<SR: SerializedEnumReader<BufReaderWrapper<File>>>(
    file_path: PathBuf,
) -> Result<impl Stream<Item = SR::Output>> {
    let stream = stream! {
        let reader = BufReader::new(File::open(file_path).await.unwrap());
        let reader = BufReaderWrapper(reader);
        let mut reader = SR::new(reader);
        while let Some(log) = reader.read_next().await.unwrap() {
            yield log;
        }
    };

    Ok(stream)
}
