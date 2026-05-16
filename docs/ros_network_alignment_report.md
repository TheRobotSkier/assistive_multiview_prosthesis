# ROS Network Alignment Report

## Scope

Host and Jetson ROS 2 networking now use one address pair consistently:

- Host: `192.168.100.1`
- Jetson: `192.168.100.2`
- `ROS_DOMAIN_ID=0`
- CycloneDDS explicit peers on both addresses

## Fix

The host CycloneDDS config previously used the old `10.42.0.x` pair while the
Jetson config and connection script used `192.168.100.x`. That would prevent
host containers from discovering Jetson ROS topics even after SSH worked.

Updated:

- `config/cyclonedds_peer.xml`
- `scripts/robotlab_connect.sh`

## Diagnostics

`scripts/robotlab_connect.sh` now distinguishes:

- no USB ethernet adapter
- adapter present but no physical carrier
- carrier present but Jetson IP not answering

Current host observation while testing:

- Adapter found: `enp0s13f0u2u2`
- Host address: `192.168.100.1/24`
- Carrier: `0`
- Result: physical Jetson ethernet link not up

## Test

Passed locally:

```bash
bash scripts/test_ros_network_contracts.sh
```

After the cable/Jetson link has carrier:

```bash
make robotlab-connect
make jetson-list-cameras
```
