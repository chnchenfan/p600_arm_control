# Static Calibration Workflow

## Summary

This document defines the staged static calibration workflow for experiment 2 geometry.

Execution split:
- Aircraft computer (Ubuntu 18.04 + ROS Melodic): run mocap startup, static sample collection, and rosbag recording
- Host workstation: copy the recorded bag back and run the offline fitting script

The stage-1 implementation in this repository fits:

- `arm_base_offset_x_m`
- `arm_base_offset_y_m`
- `arm_base_offset_z_m`
- `arm1_zero_offset_deg`
- `arm2_zero_offset_deg`

The UAV must remain landed or mechanically fixed. Only the arm is moved during collection.

## Stage 1: base xyz + arm1/arm2 zero offset

### Aircraft-side collection commands

Run the following on the aircraft computer.

1. Build `arm_control` on the aircraft computer if needed:

```bash
cd ~/p600_arm_control
catkin_make --pkg arm_control
```

2. Start mocap and PX4-side positioning for experiment 2:

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp2 <VRPN_SERVER_IP>
```

3. Start static calibration collection. This entrypoint launches the full arm-side chain required for calibration:
- arm parameter loading
- `motors_simulation` topic bridge (`/wjl/arm/guidefly/angle_d -> /wjl/arm/real/angle_d`)
- `serial_` real-arm driver (`/wjl/arm/real/angle_r`)
- `uam_static_calibration_collect`


```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_static_calibration_collect.sh
```

4. Start calibration recording:

```bash
cd ~/p600_arm_control
bash shell/Experiment/record_static_calibration_data.sh
```

### Copy bag from aircraft to host

After collection finishes on the aircraft computer, copy the calibration directory back to the host workstation. Example:

```bash
scp -r amov@<aircraft_ip>:/home/amov/p600_arm_control/data/calibration/static/<timestamp> /home/cf/Program/code/P600_uam/data/calibration/static/
```

### Host-side fitting command

Run the following on the host workstation:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/cf/Program/code/P600_uam/p600_arm_control/shell/Calibration/fit_static_geometry_stage1.py \
  --bag /home/cf/Program/code/P600_uam/data/calibration/static/<timestamp>/calibration_static.bag
```

### Data definition

The aircraft-side collector publishes 12 fixed static arm configurations. Each valid sample window lasts 2.5 s. The collector waits for `/wjl/arm/real/angle_r` from the real arm driver before opening each sample window. The host-side fitter uses `/wjl/calibration/sample_index` to segment the bag automatically.

### Output

The fitter writes:

- `sample_means.csv`
- `fit_result.json`

It also prints a launch snippet containing:

- `arm_base_offset_x_m`
- `arm_base_offset_y_m`
- `arm_base_offset_z_m`
- `arm1_zero_offset_deg`
- `arm2_zero_offset_deg`

Write these values back to:

- `src/arm_control/launch/simulation/desired_uam_fly_exp2.launch`

### Upgrade rule

If stage-1 `RMS > 8 mm`, move to stage 2.

## Stage 2: add arm_target xyz

Keep stage-1 parameters free, then additionally fit:

- `arm_target_offset_x_m`
- `arm_target_offset_y_m`
- `arm_target_offset_z_m`

Use the same static data collection process. Only upgrade to stage 2 if stage 1 cannot reduce residuals below the target.

### Upgrade rule

If stage-2 `RMS > 8 mm` or residuals still show systematic length error, move to stage 3.

## Stage 3: add L1/L2

Keep stage-2 parameters free, then additionally fit:

- `L1_m`
- `L2_m`

This stage should only be used after base/target rigid-body offsets and joint zero offsets are already reasonably aligned.

## Acceptance targets

Recommended acceptance targets:

- `RMS < 8 mm`
- max per-sample residual `< 20 mm`

## Notes

- Do not fly during calibration.
- The aircraft computer only needs to compile and run the calibration launch plus rosbag recording; it does not need to run the fitting script.
- The fitting script is intended to run on the host workstation with Python 3, `numpy`, `scipy`, and `rosbag` available.
- Do not release all parameters at once in the first iteration.
- The stage-1 script in this repository implements only the stage-1 parameter set.
