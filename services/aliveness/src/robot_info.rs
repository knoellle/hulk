use std::{error::Error, fmt, time::Duration};

use aliveness::Battery;
use booster::MotorState;
use color_eyre::eyre::{eyre, Context, Result};
use kinematics::joints::Joints;
use log::{debug, warn};
use robot::extract_version_number;
use ros_z::context::ContextBuilder;
use rustdds::{
    no_key::{Decode, DefaultDecoder, DeserializerAdapter},
    policy, DomainParticipant, QosPolicyBuilder, ReadCondition, RepresentationIdentifier,
    TopicKind,
};
use tokio::{fs, process::Command, sync::watch, task::JoinHandle, time::sleep};

const BOOSTER_VERSION_PATH: &str = "/opt/booster/version.txt";
const BATTERY_TOPIC: &str = "rt/device_gateway";
const BATTERY_TYPE: &str = "booster_interface::msg::dds_::RobotStatusDdsMsg_";
const DDS_DOMAIN_ID: u16 = 0;
const ROS_Z_ROUTER_ENDPOINT: &str = "tcp/127.0.0.1:7447";
const TELEMETRY_RETRY_DELAY: Duration = Duration::from_secs(2);

const BATTERY_ENCODINGS: [RepresentationIdentifier; 10] = [
    RepresentationIdentifier::CDR_BE,
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::PL_CDR_BE,
    RepresentationIdentifier::PL_CDR_LE,
    RepresentationIdentifier::CDR2_BE,
    RepresentationIdentifier::CDR2_LE,
    RepresentationIdentifier::XCDR2_BE,
    RepresentationIdentifier::XCDR2_LE,
    RepresentationIdentifier::PL_XCDR2_BE,
    RepresentationIdentifier::PL_XCDR2_LE,
];

struct BatteryDeserializerAdapter;

#[derive(Clone, Copy)]
struct BatteryDecoder;

#[derive(Debug)]
struct BatteryDecodeError(String);

impl fmt::Display for BatteryDecodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl Error for BatteryDecodeError {}

impl DeserializerAdapter<Battery> for BatteryDeserializerAdapter {
    type Error = BatteryDecodeError;
    type Decoded = Battery;

    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &BATTERY_ENCODINGS
    }

    fn transform_decoded(decoded: Self::Decoded) -> Battery {
        decoded
    }
}

impl DefaultDecoder<Battery> for BatteryDeserializerAdapter {
    type Decoder = BatteryDecoder;

    const DECODER: Self::Decoder = BatteryDecoder;
}

impl Decode<Battery> for BatteryDecoder {
    type Error = BatteryDecodeError;

    fn decode_bytes(
        self,
        input_bytes: &[u8],
        encoding: RepresentationIdentifier,
    ) -> std::result::Result<Battery, Self::Error> {
        decode_battery_payload(input_bytes, encoding)
    }
}

pub struct RobotInfo {
    pub hulks_os_version: String,
    pub hostname: String,
    serial_number: Option<String>,
    battery_receiver: watch::Receiver<Option<Battery>>,
    temperature_receiver: watch::Receiver<Option<Vec<f32>>>,
    tasks: Vec<JoinHandle<()>>,
}

impl RobotInfo {
    pub async fn initialize(ros_namespace: String) -> Result<Self> {
        let hulks_os_version = get_hulks_os_version()
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
            hulks_os_version,
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
            Ok(()) => warn!("DDS battery subscription ended"),
            Err(error) => warn!("failed to watch DDS battery state: {error:#}"),
        }
        sleep(TELEMETRY_RETRY_DELAY).await;
    }
}

async fn watch_battery_until_disconnect(sender: &watch::Sender<Option<Battery>>) -> Result<()> {
    let participant = DomainParticipant::new(DDS_DOMAIN_ID)
        .wrap_err("failed to create DDS domain participant")?;
    let qos = QosPolicyBuilder::new()
        .reliability(policy::Reliability::BestEffort)
        .durability(policy::Durability::Volatile)
        .history(policy::History::KeepLast { depth: 1 })
        .build();
    let subscriber = participant
        .create_subscriber(&qos)
        .wrap_err("failed to create DDS subscriber")?;
    let topic = participant
        .create_topic(
            BATTERY_TOPIC.to_string(),
            BATTERY_TYPE.to_string(),
            &qos,
            TopicKind::NoKey,
        )
        .wrap_err("failed to create DDS battery topic")?;
    let mut reader = subscriber
        .create_datareader_no_key::<Battery, BatteryDeserializerAdapter>(&topic, Some(qos))
        .wrap_err("failed to create DDS battery reader")?;

    loop {
        let samples = reader
            .take(usize::MAX, ReadCondition::not_read())
            .wrap_err("failed to read DDS battery samples")?;
        for sample in samples {
            let _ = sender.send(Some(sample.into_value()));
        }

        sleep(Duration::from_millis(100)).await;
    }
}

fn normalize_charge(charge: f32) -> f32 {
    if charge > 1.0 {
        charge / 100.0
    } else {
        charge
    }
}

fn decode_battery_payload(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
) -> std::result::Result<Battery, BatteryDecodeError> {
    let little_endian = match encoding {
        RepresentationIdentifier::CDR_LE
        | RepresentationIdentifier::PL_CDR_LE
        | RepresentationIdentifier::CDR2_LE
        | RepresentationIdentifier::XCDR2_LE
        | RepresentationIdentifier::PL_XCDR2_LE => true,
        RepresentationIdentifier::CDR_BE
        | RepresentationIdentifier::PL_CDR_BE
        | RepresentationIdentifier::CDR2_BE
        | RepresentationIdentifier::XCDR2_BE
        | RepresentationIdentifier::PL_XCDR2_BE => false,
        _ => {
            return Err(BatteryDecodeError(format!(
                "unsupported battery representation {:?}",
                encoding.to_bytes()
            )))
        }
    };

    for offset in [0, 4, 8] {
        if let Ok(battery) = decode_robot_status_battery(input_bytes, offset, little_endian) {
            return Ok(battery);
        }
    }

    Err(BatteryDecodeError(format!(
        "could not decode battery payload with {} bytes and representation {:?}",
        input_bytes.len(),
        encoding.to_bytes()
    )))
}

fn decode_robot_status_battery(
    input_bytes: &[u8],
    offset: usize,
    little_endian: bool,
) -> std::result::Result<Battery, BatteryDecodeError> {
    let mut reader = CdrReader::new(input_bytes, offset, little_endian);

    let joint_count = reader.read_sequence_length("joint_vec")?;
    for _ in 0..joint_count {
        skip_joint_status(&mut reader)?;
    }

    let imu_count = reader.read_sequence_length("imu_vec")?;
    for _ in 0..imu_count {
        skip_imu_status(&mut reader)?;
    }

    let battery_count = reader.read_sequence_length("battery_vec")?;
    for _ in 0..battery_count {
        let battery = read_battery_status(&mut reader)?;
        if is_valid_battery(&battery) {
            return Ok(battery);
        }
    }

    Err(BatteryDecodeError(
        "RobotStatusDdsMsg did not contain a valid battery status".to_string(),
    ))
}

fn skip_joint_status(reader: &mut CdrReader<'_>) -> std::result::Result<(), BatteryDecodeError> {
    reader.skip_string()?;
    reader.skip_i32()?;
    reader.skip_bool()?;
    reader.skip_i32()?;
    reader.skip_i32()?;
    reader.skip_i32()?;
    reader.skip_i32()?;
    Ok(())
}

fn skip_imu_status(reader: &mut CdrReader<'_>) -> std::result::Result<(), BatteryDecodeError> {
    reader.skip_string()?;
    reader.skip_i32()?;
    reader.skip_bool()?;
    reader.skip_i32()?;
    Ok(())
}

fn read_battery_status(
    reader: &mut CdrReader<'_>,
) -> std::result::Result<Battery, BatteryDecodeError> {
    reader.skip_string()?;
    let _temperature = reader.read_f32()?;
    let charge = reader.read_f32()?;
    let _voltage = reader.read_f32()?;
    let status_code = reader.read_i32()?;
    let status_level = reader.read_i32()?;

    debug!(
        "DDS battery candidate: charge={charge}, status_code={status_code}, status_level={status_level}"
    );

    Ok(Battery {
        charge: normalize_charge(charge),
        current: 0.0,
        temperature: 0.0,
        voltage: 0.0,
        health: status_level,
        status_code,
    })
}

fn is_valid_battery(battery: &Battery) -> bool {
    battery.charge.is_finite() && (0.0..=1.0).contains(&battery.charge)
}

struct CdrReader<'a> {
    input_bytes: &'a [u8],
    position: usize,
    little_endian: bool,
}

impl<'a> CdrReader<'a> {
    fn new(input_bytes: &'a [u8], position: usize, little_endian: bool) -> Self {
        Self {
            input_bytes,
            position,
            little_endian,
        }
    }

    fn read_sequence_length(
        &mut self,
        field_name: &str,
    ) -> std::result::Result<usize, BatteryDecodeError> {
        let length = self.read_u32()? as usize;
        if length > 100 {
            return Err(BatteryDecodeError(format!(
                "implausible {field_name} length {length}"
            )));
        }
        Ok(length)
    }

    fn skip_string(&mut self) -> std::result::Result<(), BatteryDecodeError> {
        let length = self.read_u32()? as usize;
        if length == 0 {
            return Ok(());
        }
        self.read_bytes(length)?;
        Ok(())
    }

    fn skip_i32(&mut self) -> std::result::Result<(), BatteryDecodeError> {
        self.read_i32()?;
        Ok(())
    }

    fn skip_bool(&mut self) -> std::result::Result<(), BatteryDecodeError> {
        self.align(1)?;
        self.read_bytes(1)?;
        Ok(())
    }

    fn read_i32(&mut self) -> std::result::Result<i32, BatteryDecodeError> {
        let bytes = self.read_primitive_bytes()?;
        Ok(if self.little_endian {
            i32::from_le_bytes(bytes)
        } else {
            i32::from_be_bytes(bytes)
        })
    }

    fn read_u32(&mut self) -> std::result::Result<u32, BatteryDecodeError> {
        let bytes = self.read_primitive_bytes()?;
        Ok(if self.little_endian {
            u32::from_le_bytes(bytes)
        } else {
            u32::from_be_bytes(bytes)
        })
    }

    fn read_f32(&mut self) -> std::result::Result<f32, BatteryDecodeError> {
        let bytes = self.read_primitive_bytes()?;
        Ok(if self.little_endian {
            f32::from_le_bytes(bytes)
        } else {
            f32::from_be_bytes(bytes)
        })
    }

    fn read_primitive_bytes(&mut self) -> std::result::Result<[u8; 4], BatteryDecodeError> {
        self.align(4)?;
        Ok(self.read_bytes(4)?.try_into().expect("slice length is 4"))
    }

    fn read_bytes(&mut self, length: usize) -> std::result::Result<&'a [u8], BatteryDecodeError> {
        let end = self
            .position
            .checked_add(length)
            .ok_or_else(|| BatteryDecodeError("CDR cursor overflow".to_string()))?;
        let bytes = self.input_bytes.get(self.position..end).ok_or_else(|| {
            BatteryDecodeError(format!(
                "CDR payload ended at byte {} while reading {length} bytes from byte {}",
                self.input_bytes.len(),
                self.position,
            ))
        })?;
        self.position = end;
        Ok(bytes)
    }

    fn align(&mut self, alignment: usize) -> std::result::Result<(), BatteryDecodeError> {
        self.position = self
            .position
            .checked_add(alignment - 1)
            .map(|position| position & !(alignment - 1))
            .ok_or_else(|| BatteryDecodeError("CDR cursor overflow".to_string()))?;
        if self.position > self.input_bytes.len() {
            return Err(BatteryDecodeError(format!(
                "CDR payload ended at byte {} while aligning to byte {}",
                self.input_bytes.len(),
                self.position,
            )));
        }
        Ok(())
    }
}

async fn get_hulks_os_version() -> Result<String> {
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
