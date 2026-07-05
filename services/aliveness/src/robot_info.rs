use std::time::Duration;

use aliveness::Battery;
use booster::MotorState;
use color_eyre::eyre::{eyre, Context, Result};
use kinematics::joints::Joints;
use log::{debug, warn};
use robot::extract_version_number;
use ros_z::context::ContextBuilder;
use serde::Deserialize;
use tokio::{fs, process::Command, sync::watch, task::JoinHandle, time::sleep};

const BOOSTER_VERSION_PATH: &str = "/opt/booster/version.txt";
const DEVICE_GATEWAY_TOPIC: &str = "rt/device_gateway";
const BATTERY_STATE_TOPIC: &str = "rt/battery_state";
const ROS_Z_ROUTER_ENDPOINT: &str = "tcp/127.0.0.1:7447";
const TELEMETRY_RETRY_DELAY: Duration = Duration::from_secs(2);

pub struct RobotInfo {
    pub os_version: String,
    pub hostname: String,
    serial_number: Option<String>,
    battery_receiver: watch::Receiver<Option<Battery>>,
    temperature_receiver: watch::Receiver<Option<Vec<f32>>>,
    tasks: Vec<JoinHandle<()>>,
}

impl RobotInfo {
    pub async fn initialize(ros_namespace: String) -> Result<Self> {
        let os_version = get_os_version()
            .await
            .wrap_err("failed to load Booster OS version")?;
        let hostname = hostname::get()
            .wrap_err("failed to query hostname")?
            .into_string()
            .map_err(|hostname| eyre!("invalid utf8 in hostname: {hostname:?}"))?;
        let serial_number = get_serial_number().await.ok().flatten();
        let (battery_sender, battery_receiver) = watch::channel(None);
        let (temperature_sender, temperature_receiver) = watch::channel(None);

        let tasks = vec![
            tokio::spawn(watch_battery(battery_sender)),
            tokio::spawn(watch_motor_temperatures(ros_namespace, temperature_sender)),
        ];

        Ok(Self {
            os_version,
            hostname,
            serial_number,
            battery_receiver,
            temperature_receiver,
            tasks,
        })
    }

    pub fn robot_name(&self) -> Option<String> {
        Some(self.hostname.clone())
    }

    pub fn serial_number(&self) -> Option<String> {
        self.serial_number.clone()
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

async fn watch_battery(sender: watch::Sender<Option<Battery>>) {
    loop {
        match watch_battery_until_disconnect(&sender).await {
            Ok(()) => warn!("Zenoh battery subscription ended"),
            Err(error) => warn!("failed to watch Zenoh battery state: {error:#}"),
        }
        sleep(TELEMETRY_RETRY_DELAY).await;
    }
}

async fn watch_battery_until_disconnect(sender: &watch::Sender<Option<Battery>>) -> Result<()> {
    let context = ContextBuilder::default()
        .with_mode("client")
        .with_connect_endpoints([ROS_Z_ROUTER_ENDPOINT])
        .build()
        .await
        .wrap_err("failed to create ROS-Z context for battery state")?;
    let zenoh_session = context.session();
    let device_gateway_sub = zenoh_session
        .declare_subscriber(DEVICE_GATEWAY_TOPIC)
        .await
        .map_err(|error| eyre!("failed to subscribe to {DEVICE_GATEWAY_TOPIC}: {error}"))?;
    let battery_state_sub = zenoh_session
        .declare_subscriber(BATTERY_STATE_TOPIC)
        .await
        .map_err(|error| eyre!("failed to subscribe to {BATTERY_STATE_TOPIC}: {error}"))?;

    loop {
        tokio::select! {
            sample = device_gateway_sub.recv_async() => {
                let sample = sample.map_err(|error| eyre!("failed to receive sample from {DEVICE_GATEWAY_TOPIC}: {error}"))?;
                match decode_device_gateway_battery_payload(&sample.payload().to_bytes()) {
                    Ok(battery) => {
                        let _ = sender.send(Some(battery));
                    }
                    Err(error) => debug!("failed to decode {DEVICE_GATEWAY_TOPIC} battery payload: {error}"),
                }
            }
            sample = battery_state_sub.recv_async() => {
                let sample = sample.map_err(|error| eyre!("failed to receive sample from {BATTERY_STATE_TOPIC}: {error}"))?;
                match decode_battery_state_payload(&sample.payload().to_bytes()) {
                    Ok(battery) => {
                        let _ = sender.send(Some(battery));
                    }
                    Err(error) => debug!("failed to decode {BATTERY_STATE_TOPIC} payload: {error}"),
                }
            }
        }
    }
}

fn normalize_charge(charge: f32) -> f32 {
    if charge > 1.0 {
        charge / 100.0
    } else {
        charge
    }
}

fn decode_device_gateway_battery_payload(input_bytes: &[u8]) -> Result<Battery> {
    let robot_status: RobotStatusDdsMsg = cdr::deserialize(input_bytes)
        .wrap_err("failed to deserialize rt/device_gateway payload")?;

    robot_status
        .battery_vec
        .into_iter()
        .map(RobotDdsBatteryStatus::into_battery)
        .find(is_valid_battery)
        .ok_or_else(|| eyre!("RobotStatusDdsMsg did not contain a valid battery status"))
}

fn decode_battery_state_payload(input_bytes: &[u8]) -> Result<Battery> {
    let battery_state: BatteryState =
        cdr::deserialize(input_bytes).wrap_err("failed to deserialize rt/battery_state payload")?;
    let battery = battery_state.into_battery();

    if is_valid_battery(&battery) {
        Ok(battery)
    } else {
        Err(eyre!("BatteryState did not contain a valid charge"))
    }
}

#[allow(dead_code)]
#[derive(Debug, Deserialize)]
struct RobotStatusDdsMsg {
    joint_vec: Vec<RobotDdsJointStatus>,
    imu_vec: Vec<RobotDdsImuStatus>,
    battery_vec: Vec<RobotDdsBatteryStatus>,
}

#[allow(dead_code)]
#[derive(Debug, Deserialize)]
struct RobotDdsJointStatus {
    name: String,
    mode: i32,
    is_enable: bool,
    status_code: i32,
    error_code: i32,
    status_level: i32,
    temperature: i32,
}

#[allow(dead_code)]
#[derive(Debug, Deserialize)]
struct RobotDdsImuStatus {
    name: String,
    status_code: i32,
    is_enable: bool,
    status_level: i32,
}

#[allow(dead_code)]
#[derive(Debug, Deserialize)]
struct RobotDdsBatteryStatus {
    name: String,
    temperature: f32,
    soc: f32,
    voltage: f32,
    status_code: i32,
    status_level: i32,
}

impl RobotDdsBatteryStatus {
    fn into_battery(self) -> Battery {
        debug!(
            "Zenoh device_gateway battery candidate: charge={}, status_code={}, status_level={}",
            self.soc, self.status_code, self.status_level,
        );

        Battery {
            charge: normalize_charge(self.soc),
            current: 0.0,
            temperature: 0.0,
            voltage: 0.0,
            health: self.status_level,
            status_code: self.status_code,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Deserialize)]
struct BatteryState {
    voltage: f32,
    current: f32,
    soc: f32,
    average_voltage: f32,
}

impl BatteryState {
    fn into_battery(self) -> Battery {
        debug!(
            "Zenoh battery_state candidate: charge={}, current={}, voltage={}, average_voltage={}",
            self.soc, self.current, self.voltage, self.average_voltage,
        );

        Battery {
            charge: normalize_charge(self.soc),
            current: 0.0,
            temperature: 0.0,
            voltage: 0.0,
            health: 0,
            status_code: 0,
        }
    }
}

fn is_valid_battery(battery: &Battery) -> bool {
    battery.charge.is_finite() && (0.0..=1.0).contains(&battery.charge)
}

async fn get_os_version() -> Result<String> {
    let contents = fs::read_to_string(BOOSTER_VERSION_PATH)
        .await
        .wrap_err_with(|| format!("failed to read {BOOSTER_VERSION_PATH}"))?;
    extract_version_number(&contents)
        .ok_or_else(|| eyre!("could not extract version number from {BOOSTER_VERSION_PATH}"))
}

async fn get_serial_number() -> Result<Option<String>> {
    let output = Command::new("jetson_release")
        .arg("-s")
        .output()
        .await
        .wrap_err("failed to execute jetson_release -s")?;

    if !output.status.success() {
        return Ok(None);
    }

    let output = String::from_utf8(output.stdout).wrap_err("failed to decode jetson_release")?;
    Ok(extract_serial_number(&output))
}

fn extract_serial_number(output: &str) -> Option<String> {
    output.lines().find_map(|line| {
        let line = strip_ansi_escape_sequences(line);
        let (key, value) = line.split_once(':')?;

        (key.trim() == "Serial Number")
            .then_some(value.trim())
            .filter(|value| !value.is_empty())
            .map(ToOwned::to_owned)
    })
}

fn strip_ansi_escape_sequences(input: &str) -> String {
    let mut output = String::with_capacity(input.len());
    let mut chars = input.chars();

    while let Some(character) = chars.next() {
        if character != '\u{1b}' {
            output.push(character);
            continue;
        }

        if chars.next() != Some('[') {
            continue;
        }

        for character in chars.by_ref() {
            if character.is_ascii_alphabetic() {
                break;
            }
        }
    }

    output
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
        let Some(motor_states) = subscriber
            .recv()
            .await
            .wrap_err("failed to receive ROS-Z inputs/parallel_motor_states")?
        else {
            continue;
        };
        let _ = sender.send(motor_temperatures(motor_states));
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

    let output = String::from_utf8(output.stdout).wrap_err("failed to decode nmcli output")?;
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
