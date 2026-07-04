use std::time::Duration;

use aliveness::{Battery, RobotIdentity};
use booster::MotorState;
use color_eyre::eyre::{eyre, Context, Result};
use kinematics::joints::Joints;
use log::warn;
use robot::extract_version_number;
use ros_z::context::ContextBuilder;
use tokio::{fs, process::Command, sync::watch, task::JoinHandle, time::sleep};

const BOOSTER_VERSION_PATH: &str = "/opt/booster/version.txt";
const TELEMETRY_RETRY_DELAY: Duration = Duration::from_secs(2);
const ROS_Z_ROUTER_ENDPOINT: &str = "tcp/127.0.0.1:7447";

pub struct RobotInfo {
    pub hulks_os_version: String,
    pub hostname: String,
    temperature_receiver: watch::Receiver<Option<Vec<f32>>>,
    tasks: Vec<JoinHandle<()>>,
}

impl RobotInfo {
    pub async fn initialize(ros_namespace: String) -> Result<Self> {
        let hulks_os_version = get_hulks_os_version()
            .await
            .wrap_err("failed to load HULKs-OS version")?;
        let hostname = hostname::get()
            .wrap_err("failed to query hostname")?
            .into_string()
            .map_err(|hostname| eyre!("invalid utf8 in hostname: {hostname:?}"))?;

        let (temperature_sender, temperature_receiver) = watch::channel(None);

        let tasks = vec![tokio::spawn(watch_motor_temperatures(
            ros_namespace,
            temperature_sender,
        ))];

        Ok(Self {
            hulks_os_version,
            hostname,
            temperature_receiver,
            tasks,
        })
    }

    pub fn robot_identity(&self) -> Option<RobotIdentity> {
        None
    }

    pub fn battery(&self) -> Option<Battery> {
        None
    }

    pub fn temperature(&self) -> Option<Vec<f32>> {
        self.temperature_receiver.borrow().clone()
    }
}

impl Drop for RobotInfo {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
    }
}

async fn get_hulks_os_version() -> Result<String> {
    let version = fs::read_to_string(BOOSTER_VERSION_PATH)
        .await
        .wrap_err_with(|| format!("failed to read {BOOSTER_VERSION_PATH}"))?;
    extract_version_number(&version)
        .ok_or_else(|| eyre!("could not extract version number from {BOOSTER_VERSION_PATH}"))
}

async fn watch_motor_temperatures(ros_namespace: String, sender: watch::Sender<Option<Vec<f32>>>) {
    loop {
        match watch_motor_temperatures_until_disconnect(&ros_namespace, &sender).await {
            Ok(()) => warn!("ROS-Z motor state subscription ended"),
            Err(error) => warn!("failed to watch ROS-Z motor temperatures: {error:#}"),
        }
        sleep(TELEMETRY_RETRY_DELAY).await;
    }
}

async fn watch_motor_temperatures_until_disconnect(
    ros_namespace: &str,
    sender: &watch::Sender<Option<Vec<f32>>>,
) -> Result<()> {
    let context = ContextBuilder::default()
        .with_namespace(ros_namespace)
        .with_mode("client")
        .with_connect_endpoints([ROS_Z_ROUTER_ENDPOINT])
        .build()
        .await
        .wrap_err("failed to create ROS-Z context")?;
    let node = context
        .create_node("aliveness")
        .without_schema_service()
        .build()
        .await
        .wrap_err("failed to create ROS-Z aliveness node")?;
    let subscriber = node
        .subscriber::<Option<Joints<MotorState>>>("inputs/parallel_motor_states")
        .build()
        .await
        .wrap_err("failed to subscribe to ROS-Z inputs/parallel_motor_states")?;

    loop {
        let motor_states = subscriber
            .recv()
            .await
            .wrap_err("failed to receive ROS-Z inputs/parallel_motor_states")?;
        if let Some(motor_states) = motor_states {
            let _ = sender.send(motor_temperatures(motor_states));
        }
    }
}

fn motor_temperatures(motor_states: Joints<MotorState>) -> Option<Vec<f32>> {
    let temperatures = motor_states
        .into_iter()
        .filter(|state| state.temperature > 0)
        .map(|state| state.temperature as f32)
        .collect::<Vec<_>>();
    (!temperatures.is_empty()).then_some(temperatures)
}

pub async fn get_network() -> Result<Option<String>> {
    let output = Command::new("nmcli")
        .args([
            "--terse",
            "--escape",
            "no",
            "--fields",
            "TYPE,STATE,CONNECTION",
            "device",
            "status",
        ])
        .output()
        .await
        .wrap_err("failed to execute nmcli command")?;

    if !output.status.success() {
        return Ok(None);
    }

    let output = String::from_utf8(output.stdout).wrap_err("failed to decode UTF-8")?;
    Ok(parse_connected_wifi_network(&output))
}

fn parse_connected_wifi_network(output: &str) -> Option<String> {
    output.lines().find_map(|line| {
        let mut fields = line.splitn(3, ':');
        let device_type = fields.next()?;
        let state = fields.next()?;
        let connection = fields.next()?.trim();

        (device_type == "wifi"
            && state.starts_with("connected")
            && !connection.is_empty()
            && connection != "--")
            .then(|| connection.to_owned())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_motor_temperatures_from_motor_states() {
        let motor_states = Joints::fill(MotorState {
            temperature: 42,
            ..Default::default()
        });

        assert_eq!(motor_temperatures(motor_states), Some(vec![42.0; 22]));
    }

    #[test]
    fn ignore_zero_motor_temperatures() {
        let motor_states = Joints::fill(MotorState::default());

        assert_eq!(motor_temperatures(motor_states), None);
    }
}
