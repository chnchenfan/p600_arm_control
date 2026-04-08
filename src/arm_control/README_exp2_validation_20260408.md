# Experiment 2 Validation Record 2026-04-08

## Summary

This record summarizes the final usable experiment-2 geometry parameters, the end-to-end validation flow, and the measured indicators from the successful `validation_suite` run on 2026-04-08.

Final result:

- static calibration fitting completed
- multi-static IK validation passed: `5/5`
- virtual disturbance validation passed: `true`
- overall validation result: `true`

This record corresponds to the current experiment-2 launch configuration in:

- `src/arm_control/launch/simulation/desired_uam_fly_exp2.launch`
- `src/arm_control/launch/simulation/desired_uam_fly_exp2_static_ik.launch`
- `src/arm_control/launch/simulation/desired_uam_fly_exp2_validation.launch`

## Final Usable Parameters

Source dataset:

- `/home/cf/Program/code/P600_uam/data/calibration/static/20260408_1`

Selected fit mode:

- `base_xyz_dq_l2`

Final parameters written back to launch:

```xml
<param name="L1_m" value="0.154859" />
<param name="L2_m" value="0.223951" />
<param name="arm_base_offset_x_m" value="0.024759" />
<param name="arm_base_offset_y_m" value="0.006501" />
<param name="arm_base_offset_z_m" value="-0.005065" />
<param name="arm1_zero_offset_deg" value="-7.011313" />
<param name="arm2_zero_offset_deg" value="-5.434880" />
```

Validation-related runtime parameters:

```xml
<param name="pose_timeout_sec" value="0.8" />
<param name="hold_error_pass_m" value="0.01" />
<param name="ik_fail_limit" value="10" />
<param name="pose_loss_limit" value="10" />
<param name="eps_r_m" value="0.01" />
<param name="eps_xy_m" value="0.015" />
```

Current code-level validation behavior:

- static and virtual steps pass when `ik_fail_count == 0` and `pose_loss_count < pose_loss_limit`
- hard abort still occurs when `ik_fail_count >= ik_fail_limit` or `pose_loss_count >= pose_loss_limit`
- during virtual disturbance only, if the commanded target radius falls outside the reachable shell, the target is projected back to radius `L2` before solving IK

## Workflow

Recommended execution flow:

1. Calibration
2. Static test
3. Virtual disturbance test

### 1. Calibration

Aircraft-side:

- run mocap and arm chain
- run `uam_static_calibration_collect`
- record `calibration_static.bag`

Host-side:

- run `fit_static_geometry_stage1.py`
- choose the final fit mode
- write fitted parameters back to the experiment-2 launch files

This round used:

- dataset: `20260408_1`
- fit mode: `base_xyz_dq_l2`

### 2. Static Test

Run:

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_control_desired_exp2_validation.sh
```

The static validation phase runs five fixed initial poses:

- `(0, 0)`
- `(30, 0)`
- `(-30, 0)`
- `(0, 10)`
- `(0, -10)`

For each static case:

- preposition to the requested arm pose
- freeze the current `arm_target` world position as `P_hold`
- run closed-loop static IK for `10 s`
- record hold-point and IK diagnostics

### 3. Virtual Disturbance Test

After all static cases pass, the suite:

- returns to center pose `(0, 0)`
- freezes a new `P_hold`
- runs a fixed virtual disturbance step table

The disturbance sequence is:

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

Nominal disturbance amplitudes:

- `x`: `±0.020 m`
- `y`: `±0.020 m`
- `z`: `±0.015 m`
- `yaw`: `±8 deg`

## Test Indicators

### Calibration Indicators

From `/home/cf/Program/code/P600_uam/data/calibration/static/20260408_1/fit_base_xyz_dq_l2/fit_result.json`:

- sample count: `39`
- initial RMS: `0.02424 m`
- fitted RMS: `0.00456 m`
- max per-sample residual: `0.01462 m`
- optimizer status: `success=true`

Meaning of main calibration indicators:

- `initial_rms_m`: residual RMS before fitting
- `fitted_rms_m`: residual RMS after fitting
- `per_sample_residual_norm_m`: residual norm for each static sample

### Static Test Indicators

Main per-case indicators:

- `reach_error_max`
- `hold_error_final_m`
- `hold_error_max_m`
- `ik_fail_count`
- `pose_loss_count`
- `freeze_real`
- `freeze_ik`
- `freeze_delta`
- `freeze_fk_hold_error_m`

Static-case pass criteria used in the current code:

- `ik_fail_count == 0`
- `pose_loss_count < pose_loss_limit`
- `reach_error_max <= eps_r_m`
- `hold_error_final_m <= hold_error_pass_m`

Final successful run results:

| Case | Initial pose | reach_error_max (m) | hold_error_final_m (m) | hold_error_max_m (m) | ik_fail_count | pose_loss_count | freeze_delta (deg) | freeze_fk_hold_error_m (m) | pass |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| center_0_0 | `(0, 0)` | 0.0044 | 0.0050 | 0.0071 | 0 | 0 | `(-0.76, -1.57)` | 0.0079 | true |
| right_30_0 | `(30, 0)` | 0.0017 | 0.0031 | 0.0057 | 0 | 0 | `(-1.01, -0.19)` | 0.0043 | true |
| left_-30_0 | `(-30, 0)` | 0.0011 | 0.0057 | 0.0064 | 0 | 0 | `(0.77, -1.68)` | 0.0073 | true |
| up_0_10 | `(0, 10)` | 0.0045 | 0.0038 | 0.0056 | 0 | 0 | `(-1.13, -0.81)` | 0.0069 | true |
| down_0_-10 | `(0, -10)` | 0.0030 | 0.0057 | 0.0077 | 0 | 4 | `(-0.75, -1.78)` | 0.0079 | true |

Static-phase summary:

- `static_cases_passed = 5/5`

### Virtual Disturbance Indicators

Main per-step indicators:

- `peak_hold_error_m`
- `final_hold_error_m`
- `peak_reach_error_m`
- `ik_fail_count`
- `pose_loss_count`
- `pass`

Virtual-step pass criteria used in the current code:

- `ik_fail_count == 0`
- `pose_loss_count < pose_loss_limit`
- for `zero_reset_*` steps, additionally `final_hold_error_m <= hold_error_pass_m`

Final successful run results:

| Step | Disturbance | peak_hold_error_m (m) | final_hold_error_m (m) | peak_reach_error_m (m) | pass |
| --- | --- | ---: | ---: | ---: | --- |
| baseline_zero | `(0, 0, 0, yaw=0)` | 0.0078 | 0.0055 | 0.0043 | true |
| x_pos | `(0.020, 0, 0, yaw=0)` | 0.0071 | 0.0069 | 0.0000 | true |
| zero_reset_1 | `(0, 0, 0, yaw=0)` | 0.0069 | 0.0051 | 0.0042 | true |
| x_neg | `(-0.020, 0, 0, yaw=0)` | 0.0053 | 0.0039 | 0.0000 | true |
| zero_reset_2 | `(0, 0, 0, yaw=0)` | 0.0060 | 0.0056 | 0.0042 | true |
| y_pos | `(0, 0.020, 0, yaw=0)` | 0.0149 | 0.0146 | 0.0051 | true |
| zero_reset_3 | `(0, 0, 0, yaw=0)` | 0.0146 | 0.0054 | 0.0042 | true |
| y_neg | `(0, -0.020, 0, yaw=0)` | 0.0130 | 0.0122 | 0.0020 | true |
| zero_reset_4 | `(0, 0, 0, yaw=0)` | 0.0122 | 0.0055 | 0.0042 | true |
| z_pos | `(0, 0, 0.015, yaw=0)` | 0.0200 | 0.0172 | 0.0024 | true |
| zero_reset_5 | `(0, 0, 0, yaw=0)` | 0.0173 | 0.0054 | 0.0043 | true |
| z_neg | `(0, 0, -0.015, yaw=0)` | 0.0094 | 0.0077 | 0.0053 | true |
| zero_reset_6 | `(0, 0, 0, yaw=0)` | 0.0087 | 0.0051 | 0.0042 | true |
| yaw_pos | `(0, 0, 0, yaw=8)` | 0.0224 | 0.0190 | 0.0047 | true |
| zero_reset_7 | `(0, 0, 0, yaw=0)` | 0.0193 | 0.0058 | 0.0042 | true |
| yaw_neg | `(0, 0, 0, yaw=-8)` | 0.0205 | 0.0185 | 0.0041 | true |
| zero_reset_8 | `(0, 0, 0, yaw=0)` | 0.0191 | 0.0056 | 0.0042 | true |

Virtual-phase summary:

- `virtual_disturbance_passed = true`

## Notes

- The current successful result depends on `pose_timeout_sec = 0.8`.
- The `down_0_-10` static case still saw non-zero `pose_loss_count`, but stayed below `pose_loss_limit`, so the case passed under the current rule.
- `x_pos` and `x_neg` can push the raw target radius outside the reachable shell. The current code projects such virtual targets back to radius `L2` and logs:

```text
exp2 validation virtual projected target to reachable shell
```

- Virtual disturbance passing does not replace a real constrained small-disturbance retest before formal flight.
