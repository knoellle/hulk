use std::time::Duration;

use aliveness::{Battery, RobotIdentity};
use booster::LowState;
use booster_sdk::{
    client::BoosterClient,
    dds::{battery_state_topic, BatteryState, DdsConfig, DdsNode, RpcClientOptions},
};
use color_eyre::eyre::{eyre, Context, Result};
use configparser::ini::Ini;
use log::warn;
use tokio::{
    process::Command,
    sync::watch,
    task::JoinHandle,
    time::{sleep, timeout},
};

const OS_RELEASE_PATH: &str = "/etc/os-release";
const ROBOT_INFO_TIMEOUT: Duration = Duration::from_millis(300);
const TELEMETRY_RETRY_DELAY: Duration = Duration::from_secs(2);

pub struct RobotInfo {
    pub hulks_os_version: String,
    pub hostname: String,
    pub robot_identity: Option<RobotIdentity>,
    battery_receiver: watch::Receiver<Option<Battery>>,
    temperature_receiver: watch::Receiver<Option<Vec<f32>>>,
    tasks: Vec<JoinHandle<()>>,
}

impl RobotInfo {
    pub async fn initialize() -> Result<Self> {
        let hulks_os_version = get_hulks_os_version()
            .await
            .wrap_err("failed to load HULKs-OS version")?;
        let hostname = hostname::get()
            .wrap_err("failed to query hostname")?
            .into_string()
            .map_err(|hostname| eyre!("invalid utf8 in hostname: {hostname:?}"))?;

        let robot_identity = match get_robot_identity().await {
            Ok(robot_identity) => robot_identity,
            Err(error) => {
                warn!("failed to query booster robot identity: {error:#}");
                None
            }
        };

        let (battery_sender, battery_receiver) = watch::channel(None);
        let (temperature_sender, temperature_receiver) = watch::channel(None);

        let tasks = vec![
            tokio::spawn(watch_battery_state(battery_sender)),
            tokio::spawn(watch_low_state_temperatures(temperature_sender)),
        ];

        Ok(Self {
            hulks_os_version,
            hostname,
            robot_identity,
            battery_receiver,
            temperature_receiver,
            tasks,
        })
    }

    pub fn battery(&self) -> Option<Battery> {
        *self.battery_receiver.borrow()
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
    let mut os_release = Ini::new();
    os_release
        .load_async(OS_RELEASE_PATH)
        .await
        .map_err(|error| eyre!("{error}"))?;
    os_release
        .get("default", "VERSION_ID")
        .ok_or_else(|| eyre!("no VERSION_ID in {OS_RELEASE_PATH}"))
}

async fn get_robot_identity() -> Result<Option<RobotIdentity>> {
    let client = BoosterClient::with_options(RpcClientOptions::default().without_startup_wait())
        .wrap_err("failed to create BoosterClient")?;
    let info = timeout(ROBOT_INFO_TIMEOUT, client.get_robot_info())
        .await
        .wrap_err("timed out querying booster robot info")?
        .wrap_err("failed to query booster robot info")?;

    Ok(Some(RobotIdentity {
        name: info.name,
        nickname: info.nickname,
        version: info.version,
        model: info.model,
        serial_number: info.serial_number,
        edition: info.edition,
        region: info.region,
    }))
}

async fn watch_battery_state(sender: watch::Sender<Option<Battery>>) {
    loop {
        match watch_battery_state_until_disconnect(&sender).await {
            Ok(()) => warn!("booster battery state subscription ended"),
            Err(error) => warn!("failed to watch booster battery state: {error:#}"),
        }
        sleep(TELEMETRY_RETRY_DELAY).await;
    }
}

async fn watch_battery_state_until_disconnect(
    sender: &watch::Sender<Option<Battery>>,
) -> Result<()> {
    let node = DdsNode::new(DdsConfig::default()).wrap_err("failed to create DDS node")?;
    let mut subscription = node
        .subscribe::<BatteryState>(&battery_state_topic(), 4)
        .wrap_err("failed to subscribe to battery_state")?;

    while let Some(state) = subscription.recv().await {
        let _ = sender.send(Some(Battery {
            charge: normalize_state_of_charge(state.soc),
            current: state.current,
            temperature: state.temperature,
            voltage: state.voltage,
            health: state.health,
            status_code: state.status_code,
        }));
    }

    Ok(())
}

fn normalize_state_of_charge(soc: f32) -> f32 {
    if soc > 1.0 {
        soc / 100.0
    } else {
        soc
    }
}

async fn watch_low_state_temperatures(sender: watch::Sender<Option<Vec<f32>>>) {
    loop {
        match watch_low_state_temperatures_until_disconnect(&sender).await {
            Ok(()) => warn!("booster low_state subscription ended"),
            Err(error) => warn!("failed to watch booster motor temperatures: {error:#}"),
        }
        sleep(TELEMETRY_RETRY_DELAY).await;
    }
}

async fn watch_low_state_temperatures_until_disconnect(
    sender: &watch::Sender<Option<Vec<f32>>>,
) -> Result<()> {
    let session = zenoh::open(zenoh::Config::default())
        .await
        .map_err(|error| eyre!("failed to open Zenoh session: {error}"))?;
    let subscriber = session
        .declare_subscriber("rt/low_state")
        .await
        .map_err(|error| eyre!("failed to subscribe to rt/low_state: {error}"))?;

    loop {
        let sample = subscriber
            .recv_async()
            .await
            .map_err(|error| eyre!("failed to receive low_state: {error}"))?;
        let low_state: LowState = match cdr::deserialize(&sample.payload().to_bytes()) {
            Ok(low_state) => low_state,
            Err(error) => {
                warn!("failed to deserialize low_state: {error:#}");
                continue;
            }
        };
        let _ = sender.send(motor_temperatures(&low_state));
    }
}

fn motor_temperatures(low_state: &LowState) -> Option<Vec<f32>> {
    let temperatures = low_state
        .motor_state_serial
        .iter()
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
    fn normalize_percent_state_of_charge() {
        assert_eq!(normalize_state_of_charge(73.0), 0.73);
    }

    #[test]
    fn keep_fractional_state_of_charge() {
        assert_eq!(normalize_state_of_charge(0.73), 0.73);
    }

    #[test]
    fn extract_motor_temperatures_from_low_state() {
        let low_state = LowState {
            motor_state_serial: vec![booster::MotorState {
                temperature: 42,
                ..Default::default()
            }],
            ..Default::default()
        };

        assert_eq!(motor_temperatures(&low_state), Some(vec![42.0]));
    }
}
