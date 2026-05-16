# Final OpenVINS Wiring Report

## Scope

Final dual-D435i OpenVINS wiring was checked statically and corrected where it
had drifted from the final rig.

## Fix

The head ArUco marker correction config now uses the final head D435i
calibration path:

- `head_d435i_336222071386/kalibr_imucam_chain.yaml`

Previously it still referenced the temporary D435 + external IMU calibration
directory.

## Runtime Interfaces

Expected OpenVINS outputs:

- `/ov_msckf_head/odomimu`
- `/ov_msckf_arm/odomimu`
- `/head/marker_pose/observation`
- `/arm/marker_pose/observation`
- TF: `marker_map -> head_imu`
- TF: `marker_map -> arm_imu`

Runtime check:

```bash
make check-final-openvins
```

## Test

Passed locally:

```bash
bash scripts/test_final_openvins_contracts.sh
```

Runtime validation is blocked until the Jetson ethernet link has carrier. The
host currently sees the adapter, but `/sys/class/net/enp0s13f0u2u2/carrier` is
`0`, so the Jetson is not reachable at `192.168.100.2`.
