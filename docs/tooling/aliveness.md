# Aliveness

Aliveness is a system for querying status information from booster robots in the network. It consists of two parts: The service running on the robots and a client for sending aliveness requests to the network and processing answers.

## Information available via aliveness

The following information can be queried from booster robots connected via Ethernet:

- Hostname
- Current HULKs-OS version
- States of the systemd services for HULK, the HULK runtime, Zenoh and the DDS bridge
- Battery charge state, current, voltage and temperature
- Booster robot identity and serial number, when the SDK RPC is available
- Wireless network name
- Motor temperatures
- Name of the interface the beacon is received from

## Aliveness service

The aliveness service is built together with the HULKs-OS image and included in it. It joins the multicast group once an IPv4 address is available and listens for all messages sent to the multicast address `224.0.0.42` as well as its own IP address.

When receiving a UDP packet with content `BEACON`, it responds by sending the above described information encoded via JSON to the sender.

## Aliveness client

Pepsi includes a fully featured aliveness client with different verbosity levels and export options, see [here](./pepsi.md#aliveness) for further information.

Example usage:

```
./pepsi aliveness
./pepsi aliveness 27 32
./pepsi aliveness --json
./pepsi aliveness --timeout 500 -v
```

When executing any of the aliveness subcommands in pepsi, it will send the aforementioned beacon message to the multicast address or to a list of robot IP addresses. It then collects all responses within a timeout and filters their content according to the chosen verbosity level.

## Potential firewall issues

When no robot addresses are specified, the beacon is sent via multicast and the answers are received via unicast.
Since the answers are from a different IP addresses, most firewalls may block them.

In this case, the user has change their firewall settings to allow the incoming messages, e.g. for ufw by adding the following rule:

```
ufw allow proto udp from 10.1.24.0/24
```
