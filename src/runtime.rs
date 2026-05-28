use anyhow::{Result, bail};
use phoxal_bus::pubsub::Stamped;
use phoxal_component::v1::CapabilityRef;
use phoxal_component::v1::capability::{Capability, Range as RangeConfig};
use phoxal_component_api::v1::capability::range::{self, Sample};
use phoxal_engine::clock::{Schedule, SchedulePolicy, Step};
use phoxal_engine::step::{Io, Publisher, Runtime, RuntimeInputs};
use phoxal_engine::{EmptyArgs, RobotRuntimeArgs};

#[derive(Clone)]
pub struct Config {
    range: SampledSensor,
}

impl Config {
    pub fn new(component_id: &str, component: &phoxal_component::v1::Component) -> Result<Self> {
        Ok(Self {
            range: Self::inspect(component_id, component)?,
        })
    }

    fn inspect(
        component_id: &str,
        component: &phoxal_component::v1::Component,
    ) -> Result<SampledSensor> {
        let mut range = None;

        for (local_capability_id, capability) in &component.capabilities {
            let capability_ref = CapabilityRef::new(component_id, local_capability_id);
            if let Capability::Range(range_config) = capability {
                if range.is_some() {
                    bail!("vl53l1x supports at most one range capability");
                }
                range = Some(SampledSensor::new(capability_ref, range_config)?);
            }
        }

        let range =
            range.ok_or_else(|| anyhow::anyhow!("vl53l1x requires one range capability"))?;
        Ok(range)
    }

    pub fn clock_period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.range.publish_rate_hz())
    }

    pub fn publish_rate_hz(&self) -> f64 {
        self.range.publish_rate_hz()
    }
}

#[derive(Debug, Clone)]
struct SampledSensor {
    capability: CapabilityRef,
    publish_rate_hz: f64,
    default_distance_m: f32,
}

impl SampledSensor {
    fn new(capability: CapabilityRef, config: &RangeConfig) -> Result<Self> {
        if !config.publish_rate_hz.is_finite() || config.publish_rate_hz <= 0.0 {
            bail!("component '{}' publish_rate_hz must be > 0", capability);
        }

        Ok(Self {
            capability,
            publish_rate_hz: config.publish_rate_hz,
            default_distance_m: config.max_range_m as f32,
        })
    }

    fn publish_rate_hz(&self) -> f64 {
        self.publish_rate_hz
    }
}

pub enum Input {}

#[derive(Debug, Default)]
struct StubBackend;

impl StubBackend {
    async fn distance_m(&mut self, default_distance_m: f32) -> Result<f32> {
        Ok(default_distance_m)
    }
}

pub struct Vl53l1xRuntime {
    backend: StubBackend,
    default_distance_m: f32,
    schedule: Schedule,
    range_pub: Publisher<Stamped<Sample>>,
}

#[async_trait::async_trait]
impl Runtime for Vl53l1xRuntime {
    const RUNTIME_ID: &'static str = "vl53l1x";

    type Args = EmptyArgs;
    type Config = Config;
    type Input = Input;

    fn config(_args: &Self::Args, _common: &RobotRuntimeArgs) -> Result<Self::Config> {
        bail!("component drivers resolve config from DriverRuntimeArgs")
    }

    fn clock_period(config: &Self::Config) -> std::time::Duration {
        config.clock_period()
    }

    async fn new(io: &mut Io<Self::Input>, config: Self::Config) -> Result<Self> {
        let range_pub = io
            .publisher::<Stamped<Sample>>(&range::topic(
                &config.range.capability.component_id,
                &config.range.capability.capability_id,
            ))
            .await?;

        Ok(Self {
            backend: StubBackend,
            default_distance_m: config.range.default_distance_m,
            schedule: Schedule::from_publish_hz(config.range.publish_rate_hz, SchedulePolicy::Skip),
            range_pub,
        })
    }

    async fn step(&mut self, step: Step, _inputs: RuntimeInputs<Self::Input>) -> Result<()> {
        if self.schedule.due_steps(step.tick.time_ns()) == 0 {
            return Ok(());
        }

        self.range_pub
            .put(&Stamped::new(
                step.tick.time_ns(),
                Sample::new(self.backend.distance_m(self.default_distance_m).await?),
            ))
            .await?;

        Ok(())
    }
}
