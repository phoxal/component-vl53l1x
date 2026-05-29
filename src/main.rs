mod runtime;

use anyhow::Result;
use clap::Parser;
use phoxal_core_engine::{DriverRuntimeArgs, step::RuntimeProcess};
use phoxal_infra_helpers::init_tracing;
use tracing::info;

use crate::runtime::Config;

#[tokio::main]
async fn main() -> Result<()> {
    init_tracing()?;
    let args = DriverRuntimeArgs::parse();
    let robot = args.runtime.robot()?;
    let bus = args.runtime.connect_bus().await?;
    let binding = robot.driver_binding(&args.component_id)?;
    let config = Config::new(&binding.component_id, binding.component)?;

    info!(
        publish_rate_hz = config.publish_rate_hz(),
        "VL53L1X runtime ready"
    );
    RuntimeProcess::new(&bus, args.simulation(), config.clock_period())
        .run::<runtime::Vl53l1xRuntime>(config)
        .await?;

    Ok(())
}
