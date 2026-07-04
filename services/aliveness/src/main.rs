use std::{
    env::{args, var},
    net::{Ipv4Addr, SocketAddr, SocketAddrV4},
    time::Duration,
};

use aliveness::{
    service_manager::SystemServices, AlivenessState, BEACON_HEADER, BEACON_MULTICAST_GROUP,
    BEACON_PORT,
};
use color_eyre::eyre::{bail, Result, WrapErr};
use log::{error, info};
use tokio::{
    net::UdpSocket,
    process::Command,
    select, spawn,
    task::JoinHandle,
    time::{self, MissedTickBehavior},
};
use tokio_util::sync::CancellationToken;
use zbus::Connection;

use crate::robot_info::{get_network, RobotInfo};

mod robot_info;

#[derive(Clone, Debug, Eq, PartialEq)]
struct InterfaceAddress {
    name: String,
    ip: Ipv4Addr,
}

struct AlivenessService {
    token: CancellationToken,
    handle: JoinHandle<()>,
}

#[derive(Debug, Eq, PartialEq)]
struct Args {
    interface_name: Option<String>,
    ros_namespace: Option<String>,
}

impl AlivenessService {
    fn cancel(&self) {
        self.token.cancel();
    }

    async fn join(self) {
        self.handle.await.unwrap();
    }
}

async fn listen_for_network_change(
    configured_interface: Option<String>,
    configured_ros_namespace: Option<String>,
) -> Result<()> {
    let dbus_connection = Connection::system().await?;
    let mut active_service: Option<(InterfaceAddress, AlivenessService)> = None;
    let mut interval = time::interval(Duration::from_secs(2));
    interval.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        interval.tick().await;
        let address = get_interface_address(configured_interface.as_deref()).await?;

        if active_service.as_ref().map(|(address, _)| address) == address.as_ref() {
            continue;
        }

        if let Some((address, service)) = active_service.take() {
            info!(
                "IPv4 on {} changed from {}, leaving multicast",
                address.name, address.ip
            );
            service.cancel();
            service.join().await;
        }

        if let Some(address) = address {
            info!(
                "IPv4 on {} available as {}, joining multicast",
                address.name, address.ip
            );
            let service = join_multicast(
                address.clone(),
                dbus_connection.clone(),
                configured_ros_namespace.clone(),
            )
            .await?;
            active_service = Some((address, service));
        }
    }
}

async fn get_interface_address(interface_name: Option<&str>) -> Result<Option<InterfaceAddress>> {
    let mut command = Command::new("ip");
    command.args(["-j", "-4", "addr", "show"]);
    if let Some(interface_name) = interface_name {
        command.args(["dev", interface_name]);
    }

    let output = command
        .output()
        .await
        .wrap_err("failed to execute ip command")?;

    if !output.status.success() {
        return Ok(None);
    }

    let interfaces: serde_json::Value = serde_json::from_slice(&output.stdout)
        .wrap_err("failed to deserialize ip command output")?;
    let Some(interfaces) = interfaces.as_array() else {
        return Ok(None);
    };

    let mut fallback = None;
    for interface in interfaces {
        let Some(name) = interface["ifname"].as_str() else {
            continue;
        };
        if name == "lo" {
            continue;
        }

        let address = interface["addr_info"].as_array().and_then(|addresses| {
            addresses.iter().find_map(|address| {
                address["local"]
                    .as_str()
                    .and_then(|local| local.parse::<Ipv4Addr>().ok())
            })
        });

        let Some(ip) = address else {
            continue;
        };

        let address = InterfaceAddress {
            name: name.to_owned(),
            ip,
        };

        if interface_name.is_some() || is_team_network(ip) {
            return Ok(Some(address));
        }

        fallback.get_or_insert(address);
    }

    Ok(fallback)
}

fn is_team_network(ip: Ipv4Addr) -> bool {
    let octets = ip.octets();
    octets[0] == 10 && octets[1] == 1
}

async fn join_multicast(
    address: InterfaceAddress,
    dbus_connection: Connection,
    configured_ros_namespace: Option<String>,
) -> Result<AlivenessService> {
    let ros_namespace =
        configured_ros_namespace.unwrap_or_else(|| ros_namespace_from_ip(address.ip));
    let robot_info = RobotInfo::initialize(ros_namespace).await?;

    let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, BEACON_PORT))
        .await
        .wrap_err("failed to bind beacon socket")?;
    socket
        .join_multicast_v4(BEACON_MULTICAST_GROUP, address.ip)
        .wrap_err_with(|| format!("failed to join multicast group on {}", address.ip))?;

    info!("Joined multicast on {}", address.ip);

    let token = CancellationToken::new();
    let mut buffer = [0; 1024];

    let handle = {
        let token = token.clone();

        spawn(async move {
            loop {
                select! {
                    _ = token.cancelled() => {
                        break;
                    }
                    message = socket.recv_from(&mut buffer) => {
                        let (num_bytes, peer) = message.wrap_err("failed to read from beacon socket").unwrap();
                        handle_beacon(
                            &socket,
                            &dbus_connection,
                            &address.name,
                            &robot_info,
                            &buffer[0..num_bytes],
                            peer,
                        )
                        .await.unwrap_or_else(|err| {
                            error!("{err}");
                        });
                    }
                }
            }
        })
    };

    Ok(AlivenessService { token, handle })
}

async fn handle_beacon(
    socket: &UdpSocket,
    dbus_connection: &Connection,
    interface_name: &str,
    robot_info: &RobotInfo,
    message: &[u8],
    peer: SocketAddr,
) -> Result<()> {
    if message != BEACON_HEADER {
        bail!("invalid beacon header {message:?}");
    }
    info!("Received beacon from {peer}");
    let system_services = SystemServices::query(dbus_connection).await?;
    let response = AlivenessState {
        hostname: robot_info.hostname.to_owned(),
        interface_name: interface_name.to_owned(),
        system_services,
        hulks_os_version: robot_info.hulks_os_version.to_owned(),
        robot_identity: robot_info.robot_identity(),
        battery: robot_info.battery(),
        temperature: robot_info.temperature(),
        network: get_network().await.ok().flatten(),
    };
    let send_buffer = serde_json::to_vec(&response).wrap_err("failed to serialize response")?;
    socket
        .send_to(&send_buffer, peer)
        .await
        .wrap_err_with(|| format!("failed to send beacon response to peer at {peer}"))?;
    Ok(())
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<()> {
    env_logger::init();

    let args = parse_args(args().skip(1));

    listen_for_network_change(args.interface_name, args.ros_namespace).await
}

fn parse_args(arguments: impl IntoIterator<Item = String>) -> Args {
    parse_args_with_ros_namespace(arguments, var("ROS_Z_NAMESPACE").ok())
}

fn parse_args_with_ros_namespace(
    arguments: impl IntoIterator<Item = String>,
    env_ros_namespace: Option<String>,
) -> Args {
    let mut interface_name = None;
    let mut ros_namespace = env_ros_namespace.map(normalize_ros_namespace);
    let mut arguments = arguments.into_iter();

    while let Some(argument) = arguments.next() {
        if argument == "--ros-namespace" {
            if let Some(namespace) = arguments.next() {
                ros_namespace = Some(normalize_ros_namespace(namespace));
            }
            continue;
        }
        if let Some(namespace) = argument.strip_prefix("--ros-namespace=") {
            ros_namespace = Some(normalize_ros_namespace(namespace.to_owned()));
            continue;
        }
        interface_name = Some(argument);
    }

    Args {
        interface_name,
        ros_namespace,
    }
}

fn ros_namespace_from_ip(ip: Ipv4Addr) -> String {
    format!("/{}", ip.octets()[3])
}

fn normalize_ros_namespace(namespace: String) -> String {
    if namespace.starts_with('/') {
        namespace
    } else {
        format!("/{namespace}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_team_network() {
        assert!(is_team_network(Ipv4Addr::new(10, 1, 24, 22)));
        assert!(!is_team_network(Ipv4Addr::new(192, 168, 10, 102)));
    }

    #[test]
    fn parse_ros_namespace() {
        assert_eq!(
            parse_args_with_ros_namespace(["--ros-namespace".to_string(), "45".to_string()], None),
            Args {
                interface_name: None,
                ros_namespace: Some("/45".to_string()),
            }
        );
        assert_eq!(
            parse_args_with_ros_namespace(
                ["wlP1p1s0".to_string(), "--ros-namespace=/44".to_string()],
                None,
            ),
            Args {
                interface_name: Some("wlP1p1s0".to_string()),
                ros_namespace: Some("/44".to_string()),
            }
        );
    }

    #[test]
    fn derive_ros_namespace_from_ip() {
        assert_eq!(ros_namespace_from_ip(Ipv4Addr::new(10, 1, 24, 43)), "/43");
    }
}
