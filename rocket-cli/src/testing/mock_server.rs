use std::sync::Arc;
use std::sync::RwLock;

use anyhow::Result;

use tokio::spawn;

use crate::gs::config::GroundStationConfig;
use crate::testing::mock_ground_station::MockVLPClient;
#[cfg(feature = "server")]
use crate::serve::server::start_server;
#[cfg(feature = "server")]
pub async fn mock_server() -> Result<()> {
    let config = Arc::new(RwLock::new(GroundStationConfig::load()?));

    let client = Box::leak(Box::new(MockVLPClient::new()));
    spawn(async move {
        start_server(client, config).await
    }).await??;
    Ok(())
}