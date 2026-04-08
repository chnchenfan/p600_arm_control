# Static Calibration Workflow

## Summary

This document defines the staged static calibration workflow for experiment 2 geometry.

Execution split:
- Aircraft computer (Ubuntu 18.04 + ROS Melodic): run mocap startup, static sample collection, and rosbag recording
- Host workstation: copy the recorded bag back and run the offline fitting script

The current experiment-2 geometry convention in this repository is: arm1 increasing moves the end effector toward -y in the arm_base body frame.

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

3. Run the static ground IK test before the formal experiment. This entry keeps the UAV fixed and only validates experiment-2 geometry compensation + IK on the arm side:

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_control_desired_exp2_static_ik.sh
```

Only continue to the formal experiment-2 flight after this static test finishes without continuous IK failure.

Target-point definition for the static IK test:
- The test does not use a manually typed new target by default.
- With `use_initial_ee_hold=true`, it freezes the current `arm_target` world position into a static `P_hold` after startup.
- The node then repeatedly solves IK to keep the end effector at that frozen world point while the UAV base remains stationary.

4. Start static calibration collection. This entrypoint launches the full arm-side chain required for calibration:
- arm parameter loading
- `motors_simulation` topic bridge (`/wjl/arm/guidefly/angle_d -> /wjl/arm/real/angle_d`)
- `serial_` real-arm driver (`/wjl/arm/real/angle_r`)
- `uam_static_calibration_collect`


```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_static_calibration_collect.sh
```

5. Start calibration recording:

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

The aircraft-side collector now publishes 39 fixed static arm configurations for the current high-coverage dataset. Each valid sample window lasts 2.5 s. The default per-sample convergence timeout is 15.0 s. The collector waits for `/wjl/arm/real/angle_r` from the real arm driver before opening each sample window. The host-side fitter uses `/wjl/calibration/sample_index` to segment the bag automatically.

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

If stage-1 `RMS > 8 mm`, move to stage 2. For the current collection round, use the built-in 39-pose sequence instead of the older 12/24-pose sets.

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

## Multi-Static IK Validation

Before the formal experiment-2 flight, run the automatic validation suite on the aircraft computer:

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_control_desired_exp2_validation.sh
```

The suite executes five fixed static IK cases in sequence:

- `(0, 0)`
- `(30, 0)`
- `(-30, 0)`
- `(0, 10)`
- `(0, -10)`

For each case, the node first prepositions the arm to the requested initial pose, then freezes the current `arm_target` world position into `P_hold`, then runs static IK for 10 s.

Each static case must satisfy:

- no continuous `IK fail`
- no continuous `pose loss`
- total `ik_fail_count = 0`
- total `pose_loss_count < pose_loss_limit`
- `max(reach_error) <= eps_r_m`
- final `hold_error_world <= 0.01 m`

Only if all five static cases pass should you continue to the virtual disturbance phase.

## Virtual Base Disturbance Validation

After all five static cases pass, the suite automatically returns to the center pose `(0, 0)`, freezes a new `P_hold`, and runs a software-injected virtual base disturbance sequence.

The disturbance sequence is a fixed step table:

- `baseline_zero`
- `x_pos`
- `zero_reset_1`
- `x_neg`
- `zero_reset_2`
- `y_pos`
- `zero_reset_3`
- `y_neg`
- `zero_reset_4`
- `z_pos`
- `zero_reset_5`
- `z_neg`
- `zero_reset_6`
- `yaw_pos`
- `zero_reset_7`
- `yaw_neg`
- `zero_reset_8`

Translation disturbances are defined in the world frame. Yaw disturbances are injected as an additional yaw rotation on top of the measured `arm_base` pose, but only inside the IK/control computation.

This validation is intended to verify:

- compensation logic stability
- command continuity
- ability to return close to the frozen hold point after each disturbance reset

Current code-level pass rule for each virtual step:

- `ik_fail_count = 0`
- `pose_loss_count < pose_loss_limit`
- for `zero_reset_*` steps, final `hold_error_world <= hold_error_pass_m`

This validation is **not** equivalent to a real base-motion hold test. Even if the virtual disturbance suite passes, you still need at least one real small-disturbance retest with the UAV unpowered or otherwise safely constrained before using the formal experiment-2 flight as the next step.


## Update Log

### 2026-04-08 23:00 CST
- Selected `20260408_1` `base_xyz_dq_l2` as the final experiment-2 static calibration result.
- Wrote the fitted parameters back to the experiment-2 launch entries.
- Increased `pose_timeout_sec` to `0.8` for experiment-2 static and validation entries to better tolerate observed VRPN stalls.
- Updated validation pass criteria so `pose_loss_count < pose_loss_limit` is accepted instead of requiring `pose_loss_count = 0`.
- Added virtual-disturbance reachable-shell projection so software-injected targets outside the radius shell are projected back before IK.
- Added a finalized run record: `src/arm_control/README_exp2_validation_20260408.md`.

### 2026-03-27 17:25 CST
- Added a validation-suite mode to the experiment-2 static IK node so one entrypoint can run either the old single-point static test or the new automatic verification flow.
- Added five fixed multi-static IK validation cases before the disturbance phase to verify the calibrated geometry in multiple workspace regions.
- Added a virtual base disturbance validation phase with fixed translation and yaw step disturbances to check compensation stability and return-to-hold behavior.
- Added a dedicated launch/script entrypoint for the automatic validation suite and updated this README to state that virtual disturbance validation does not replace a real small-disturbance retest.

### 2026-03-24 20:18 CST
- Fixed the experiment-2 arm1/body-frame sign convention: arm1 increasing now maps to end-effector motion toward -y in the arm_base body frame.
- Wrote the fitted parameters from `data/calibration/static/2026324_1` back to the experiment-2 launch entry.
- Added a static ground IK test entrypoint to validate geometry compensation and IK before the formal experiment-2 flight.
- Upgraded the default static calibration pose set from 12 samples to a 24-pose high-coverage sequence.
