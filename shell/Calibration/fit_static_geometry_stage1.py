#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Stage-1 静态几何标定拟合脚本。
#
# 这个脚本运行在主机电脑上，不参与真机控制。
# 它的输入是一包已经录好的静态标定 rosbag，核心任务是：
# 1. 根据 /wjl/calibration/sample_index 自动找出每个有效静态样本窗口；
# 2. 对每个窗口内的 arm_base / arm_target / real_angle 数据取均值；
# 3. 建立“动捕测得的几何关系”和“关节角正运动学几何关系”之间的残差；
# 4. 用最小二乘拟合几何参数。
#
# 当前支持两种模式：
# - base_xyz_dq：拟合 [arm_base_offset_x/y/z, arm1_zero_offset, arm2_zero_offset]
# - base_xyz_dq_l2：在 base_xyz_dq 基础上继续放开 L2_m
# - base_xyz_dq_l1_l2：在 base_xyz_dq_l2 基础上继续放开 L1_m
# - dx_only：只拟合 arm_base_offset_x_m，其余量固定为 0
# - dx_l2：拟合 [arm_base_offset_x_m, L2_m]，其余量固定为 0
#
# 信息流和文件之间的关系如下：
# - 飞机电脑上的 collector 节点发布 /wjl/calibration/sample_index
# - 录包脚本把 sample_index 和 pose / angle 一起录进 bag
# - 本脚本读 bag 后，先按 sample_index 切段，再在每一段内求均值，最后进入拟合器

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares

try:
    import rosbag
    import rospy
except ImportError as exc:
    raise SystemExit("rosbag/rospy import failed: %s" % exc)


TOPIC_BASE = "/vrpn_client_node/arm_base/pose"
TOPIC_TARGET = "/vrpn_client_node/arm_target/pose"
TOPIC_ARM_REAL = "/wjl/arm/real/angle_r"
TOPIC_SAMPLE_INDEX = "/wjl/calibration/sample_index"
FIT_MODE_FULL = "base_xyz_dq"
FIT_MODE_DX_ONLY = "dx_only"
FIT_MODE_DX_L2 = "dx_l2"
FIT_MODE_FULL_L2 = "base_xyz_dq_l2"
FIT_MODE_FULL_L1_L2 = "base_xyz_dq_l1_l2"


@dataclass
class Window:
    sample_index: int
    start_sec: float
    end_sec: float

    @property
    def duration_sec(self) -> float:
        return self.end_sec - self.start_sec


@dataclass
class SampleMean:
    sample_index: int
    start_sec: float
    end_sec: float
    duration_sec: float
    base_position: np.ndarray
    base_quaternion: np.ndarray
    target_position: np.ndarray
    arm1_deg: float
    arm2_deg: float
    arm1_std_deg: float
    arm2_std_deg: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit static calibration parameters from rosbag.")
    parser.add_argument("--bag", default=None, help="Input rosbag path")
    parser.add_argument("--output-dir", default=None, help="Output directory for fit_result.json and sample_means.csv")
    parser.add_argument("--l1", type=float, default=0.161, help="Current measured L1 in meters")
    parser.add_argument("--l2", type=float, default=0.262, help="Current measured L2 in meters")
    parser.add_argument("--min-window-sec", type=float, default=2.0, help="Minimum valid sample window duration")
    parser.add_argument("--base-topic", default=TOPIC_BASE)
    parser.add_argument("--target-topic", default=TOPIC_TARGET)
    parser.add_argument("--arm-real-topic", default=TOPIC_ARM_REAL)
    parser.add_argument("--sample-topic", default=TOPIC_SAMPLE_INDEX)
    parser.add_argument(
        "--fit-mode",
        default=FIT_MODE_FULL,
        choices=[FIT_MODE_FULL, FIT_MODE_DX_ONLY, FIT_MODE_DX_L2, FIT_MODE_FULL_L2, FIT_MODE_FULL_L1_L2],
        help="Parameter subset to fit",
    )
    parser.add_argument("--synthetic-test", action="store_true", help="Run a self-check on synthetic data instead of reading a bag")
    return parser.parse_args()


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def average_quaternions(quaternions: Sequence[np.ndarray]) -> np.ndarray:
    if not quaternions:
        raise ValueError("no quaternions to average")
    ref = normalize_quaternion(np.array(quaternions[0], dtype=float))
    aligned = []
    for q in quaternions:
        q_arr = normalize_quaternion(np.array(q, dtype=float))
        if np.dot(q_arr, ref) < 0.0:
            q_arr = -q_arr
        aligned.append(q_arr)
    mean_q = np.mean(np.vstack(aligned), axis=0)
    return normalize_quaternion(mean_q)


def extract_windows(bag: rosbag.Bag, sample_topic: str, min_window_sec: float) -> List[Window]:
    # 根据 sample_index 自动提取有效静态样本窗口。
    windows: List[Window] = []
    active_index = None
    active_start = None
    active_end = None

    for _, msg, t in bag.read_messages(topics=[sample_topic]):
        sample_idx = int(msg.data)
        sec = t.to_sec()
        if sample_idx >= 0:
            if active_index is None:
                active_index = sample_idx
                active_start = sec
                active_end = sec
            elif sample_idx == active_index:
                active_end = sec
            else:
                if active_end is not None and active_start is not None and active_end - active_start >= min_window_sec:
                    windows.append(Window(active_index, active_start, active_end))
                active_index = sample_idx
                active_start = sec
                active_end = sec
        else:
            if active_index is not None and active_end is not None and active_start is not None:
                if active_end - active_start >= min_window_sec:
                    windows.append(Window(active_index, active_start, active_end))
            active_index = None
            active_start = None
            active_end = None

    if active_index is not None and active_end is not None and active_start is not None:
        if active_end - active_start >= min_window_sec:
            windows.append(Window(active_index, active_start, active_end))

    return windows


def collect_window_messages(bag: rosbag.Bag, topic: str, window: Window) -> List[Tuple[float, object]]:
    start = rospy.Time.from_sec(window.start_sec)
    end = rospy.Time.from_sec(window.end_sec)
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic], start_time=start, end_time=end)]


def compute_sample_means(
    bag: rosbag.Bag,
    windows: Sequence[Window],
    base_topic: str,
    target_topic: str,
    arm_real_topic: str,
) -> List[SampleMean]:
    # 对每个窗口内的数据取均值，作为拟合输入样本。
    results: List[SampleMean] = []
    for window in windows:
        base_msgs = collect_window_messages(bag, base_topic, window)
        target_msgs = collect_window_messages(bag, target_topic, window)
        arm_msgs = collect_window_messages(bag, arm_real_topic, window)
        if not base_msgs or not target_msgs or not arm_msgs:
            continue

        base_positions = np.array(
            [[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in base_msgs], dtype=float
        )
        base_quaternions = [
            np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w], dtype=float)
            for _, m in base_msgs
        ]
        target_positions = np.array(
            [[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in target_msgs], dtype=float
        )
        arm1_values = np.array([m.arm1_angle for _, m in arm_msgs], dtype=float)
        arm2_values = np.array([m.arm2_angle for _, m in arm_msgs], dtype=float)

        results.append(
            SampleMean(
                sample_index=window.sample_index,
                start_sec=window.start_sec,
                end_sec=window.end_sec,
                duration_sec=window.duration_sec,
                base_position=np.mean(base_positions, axis=0),
                base_quaternion=average_quaternions(base_quaternions),
                target_position=np.mean(target_positions, axis=0),
                arm1_deg=float(np.mean(arm1_values)),
                arm2_deg=float(np.mean(arm2_values)),
                arm1_std_deg=float(np.std(arm1_values)),
                arm2_std_deg=float(np.std(arm2_values)),
            )
        )

    results.sort(key=lambda item: item.sample_index)
    return results


def forward_kinematics(l2_m: float, arm1_deg: float, arm2_deg: float, dq1_deg: float, dq2_deg: float) -> np.ndarray:
    # 实测约定：arm1 增大时，末端在 arm_base 体坐标系中朝 -y 方向运动。
    # 因此二维水平面的 FK 不再使用 +sin(q1)，而是使用 -sin(q1)。
    q1 = math.radians(arm1_deg + dq1_deg)
    q2 = math.radians(arm2_deg + dq2_deg)
    return np.array(
        [
            l2_m * math.cos(q2) * math.cos(q1),
            -l2_m * math.cos(q2) * math.sin(q1),
            l2_m * math.sin(q2),
        ],
        dtype=float,
    )


def expand_theta(free_theta: np.ndarray, fit_mode: str) -> np.ndarray:
    # 把“当前拟合模式下的自由参数”映射回统一的 5 维参数向量：
    # [dx, dy, dz, dq1, dq2]
    theta = np.zeros(5, dtype=float)
    if fit_mode == FIT_MODE_DX_ONLY:
        theta[0] = float(free_theta[0])
    elif fit_mode == FIT_MODE_DX_L2:
        theta[0] = float(free_theta[0])
    elif fit_mode == FIT_MODE_FULL:
        theta[:] = np.array(free_theta, dtype=float)
    elif fit_mode == FIT_MODE_FULL_L2:
        theta[:] = np.array(free_theta[:5], dtype=float)
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        theta[:] = np.array(free_theta[:5], dtype=float)
    else:
        raise ValueError("unsupported fit_mode: %s" % fit_mode)
    return theta


def residual_vector_from_full_theta(theta: np.ndarray, samples: Sequence[SampleMean], l1_m: float, l2_m: float) -> np.ndarray:
    dx, dy, dz, dq1_deg, dq2_deg = theta.tolist()
    offset_body = np.array([dx, dy, dz], dtype=float)
    residuals: List[float] = []
    for sample in samples:
        rot_wb = quaternion_to_rotation_matrix(sample.base_quaternion)
        p_b_world = sample.base_position - rot_wb.dot(offset_body)
        relative_world = sample.target_position - p_b_world
        relative_body = rot_wb.T.dot(relative_world)
        p_rel_meas = relative_body - np.array([0.0, 0.0, -l1_m], dtype=float)
        p_rel_fk = forward_kinematics(l2_m, sample.arm1_deg, sample.arm2_deg, dq1_deg, dq2_deg)
        residuals.extend((p_rel_meas - p_rel_fk).tolist())
    return np.array(residuals, dtype=float)


def compute_residual_vector(free_theta: np.ndarray, samples: Sequence[SampleMean], l1_m: float, l2_m: float, fit_mode: str) -> np.ndarray:
    # 构造拟合器使用的残差。不同模式下只放开不同子集的参数。
    theta = expand_theta(free_theta, fit_mode)
    effective_l1 = l1_m
    effective_l2 = l2_m
    if fit_mode == FIT_MODE_DX_L2:
        effective_l2 = float(free_theta[1])
    elif fit_mode == FIT_MODE_FULL_L2:
        effective_l2 = float(free_theta[5])
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        effective_l1 = float(free_theta[5])
        effective_l2 = float(free_theta[6])
    return residual_vector_from_full_theta(theta, samples, effective_l1, effective_l2)


def residual_norms(full_theta: np.ndarray, samples: Sequence[SampleMean], l1_m: float, l2_m: float) -> List[float]:
    vec = residual_vector_from_full_theta(full_theta, samples, l1_m, l2_m)
    reshaped = vec.reshape((-1, 3))
    return [float(np.linalg.norm(row)) for row in reshaped]


def rms_from_residual_vector(residual_vector: np.ndarray) -> float:
    if residual_vector.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(residual_vector))))


def fit_stage1(samples: Sequence[SampleMean], l1_m: float, l2_m: float, fit_mode: str):
    # 调用 least_squares。
    # dx_only：验证 arm_base x 偏移是否是主误差源。
    # dx_l2：验证 arm_base x 偏移 + 有效 L2 是否足以解释当前样本。
    if fit_mode == FIT_MODE_DX_ONLY:
        initial_free = np.zeros(1, dtype=float)
        lower = np.array([-0.08], dtype=float)
        upper = np.array([0.08], dtype=float)
    elif fit_mode == FIT_MODE_DX_L2:
        initial_free = np.array([0.0, l2_m], dtype=float)
        lower = np.array([-0.08, 0.15], dtype=float)
        upper = np.array([0.08, 0.35], dtype=float)
    elif fit_mode == FIT_MODE_FULL:
        initial_free = np.zeros(5, dtype=float)
        lower = np.array([-0.08, -0.08, -0.08, -20.0, -20.0], dtype=float)
        upper = np.array([0.08, 0.08, 0.08, 20.0, 20.0], dtype=float)
    elif fit_mode == FIT_MODE_FULL_L2:
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, l2_m], dtype=float)
        lower = np.array([-0.08, -0.08, -0.08, -20.0, -20.0, 0.15], dtype=float)
        upper = np.array([0.08, 0.08, 0.08, 20.0, 20.0, 0.35], dtype=float)
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        # L1/L2 与零偏、base 外参之间存在明显耦合，所以这里把范围收紧到
        # “围绕当前实测值的小范围修正”，避免优化器用极端长度去硬补模型残差。
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, l1_m, l2_m], dtype=float)
        lower = np.array([-0.08, -0.08, -0.08, -20.0, -20.0, max(0.05, l1_m - 0.08), max(0.10, l2_m - 0.10)], dtype=float)
        upper = np.array([0.08, 0.08, 0.08, 20.0, 20.0, l1_m + 0.08, l2_m + 0.10], dtype=float)
    else:
        raise ValueError("unsupported fit_mode: %s" % fit_mode)

    result = least_squares(
        compute_residual_vector,
        x0=initial_free,
        bounds=(lower, upper),
        args=(samples, l1_m, l2_m, fit_mode),
    )
    initial_full = expand_theta(initial_free, fit_mode)
    fitted_full = expand_theta(np.array(result.x, dtype=float), fit_mode)
    if fit_mode == FIT_MODE_DX_L2:
        fitted_l2 = float(result.x[1])
        initial_l2 = float(initial_free[1])
    elif fit_mode == FIT_MODE_FULL_L2:
        fitted_l2 = float(result.x[5])
        initial_l2 = float(initial_free[5])
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        fitted_l2 = float(result.x[6])
        initial_l2 = float(initial_free[6])
    else:
        fitted_l2 = float(l2_m)
        initial_l2 = float(l2_m)
    if fit_mode == FIT_MODE_FULL_L1_L2:
        fitted_l1 = float(result.x[5])
        initial_l1 = float(initial_free[5])
    else:
        fitted_l1 = float(l1_m)
        initial_l1 = float(l1_m)
    return result, initial_full, fitted_full, initial_l1, fitted_l1, initial_l2, fitted_l2


def write_sample_csv(path: str, samples: Sequence[SampleMean]) -> None:
    fieldnames = [
        "sample_index",
        "start_sec",
        "end_sec",
        "duration_sec",
        "base_x",
        "base_y",
        "base_z",
        "base_qx",
        "base_qy",
        "base_qz",
        "base_qw",
        "target_x",
        "target_y",
        "target_z",
        "arm1_deg",
        "arm2_deg",
        "arm1_std_deg",
        "arm2_std_deg",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "sample_index": sample.sample_index,
                    "start_sec": sample.start_sec,
                    "end_sec": sample.end_sec,
                    "duration_sec": sample.duration_sec,
                    "base_x": sample.base_position[0],
                    "base_y": sample.base_position[1],
                    "base_z": sample.base_position[2],
                    "base_qx": sample.base_quaternion[0],
                    "base_qy": sample.base_quaternion[1],
                    "base_qz": sample.base_quaternion[2],
                    "base_qw": sample.base_quaternion[3],
                    "target_x": sample.target_position[0],
                    "target_y": sample.target_position[1],
                    "target_z": sample.target_position[2],
                    "arm1_deg": sample.arm1_deg,
                    "arm2_deg": sample.arm2_deg,
                    "arm1_std_deg": sample.arm1_std_deg,
                    "arm2_std_deg": sample.arm2_std_deg,
                }
            )


def build_result_dict(
    fit_mode: str,
    result,
    initial_theta: np.ndarray,
    fitted_theta: np.ndarray,
    samples: Sequence[SampleMean],
    initial_l1_m: float,
    fitted_l1_m: float,
    initial_l2_m: float,
    fitted_l2_m: float,
) -> Dict[str, object]:
    initial_residual_vector = residual_vector_from_full_theta(initial_theta, samples, initial_l1_m, initial_l2_m)
    fitted_residual_vector = residual_vector_from_full_theta(fitted_theta, samples, fitted_l1_m, fitted_l2_m)
    return {
        "fit_mode": fit_mode,
        "initial_l1_m": initial_l1_m,
        "fitted_l1_m": fitted_l1_m,
        "initial_l2_m": initial_l2_m,
        "fitted_l2_m": fitted_l2_m,
        "sample_count": len(samples),
        "initial_theta": {
            "arm_base_offset_x_m": float(initial_theta[0]),
            "arm_base_offset_y_m": float(initial_theta[1]),
            "arm_base_offset_z_m": float(initial_theta[2]),
            "arm1_zero_offset_deg": float(initial_theta[3]),
            "arm2_zero_offset_deg": float(initial_theta[4]),
        },
        "fitted_theta": {
            "arm_base_offset_x_m": float(fitted_theta[0]),
            "arm_base_offset_y_m": float(fitted_theta[1]),
            "arm_base_offset_z_m": float(fitted_theta[2]),
            "arm1_zero_offset_deg": float(fitted_theta[3]),
            "arm2_zero_offset_deg": float(fitted_theta[4]),
        },
        "initial_rms_m": rms_from_residual_vector(initial_residual_vector),
        "fitted_rms_m": rms_from_residual_vector(fitted_residual_vector),
        "per_sample_residual_norm_m": residual_norms(fitted_theta, samples, fitted_l1_m, fitted_l2_m),
        "optimizer": {
            "success": bool(result.success),
            "status": int(result.status),
            "message": str(result.message),
            "cost": float(result.cost),
            "nfev": int(result.nfev),
        },
    }


def print_launch_snippet(result_dict: Dict[str, object]) -> None:
    theta = result_dict["fitted_theta"]
    print("\nSuggested launch params:")
    print('<param name="arm_base_offset_x_m" value="%.6f" />' % theta["arm_base_offset_x_m"])
    print('<param name="arm_base_offset_y_m" value="%.6f" />' % theta["arm_base_offset_y_m"])
    print('<param name="arm_base_offset_z_m" value="%.6f" />' % theta["arm_base_offset_z_m"])
    print('<param name="arm1_zero_offset_deg" value="%.6f" />' % theta["arm1_zero_offset_deg"])
    print('<param name="arm2_zero_offset_deg" value="%.6f" />' % theta["arm2_zero_offset_deg"])
    if result_dict.get("fit_mode") == FIT_MODE_FULL_L1_L2:
        print('<param name="L1_m" value="%.6f" />' % result_dict['fitted_l1_m'])
    if result_dict.get("fit_mode") in (FIT_MODE_DX_L2, FIT_MODE_FULL_L2, FIT_MODE_FULL_L1_L2):
        print('<param name="L2_m" value="%.6f" />' % result_dict['fitted_l2_m'])


def run_synthetic_test() -> int:
    true_theta = np.array([0.031, -0.006, 0.012, 2.0, -3.0], dtype=float)
    l1_m = 0.161
    l2_m = 0.262
    arm_angles = [(-60.0, 0.0), (-30.0, 0.0), (0.0, 0.0), (30.0, 15.0), (15.0, 30.0), (45.0, 15.0)]
    samples: List[SampleMean] = []
    for idx, (arm1_deg, arm2_deg) in enumerate(arm_angles):
        q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        rot = quaternion_to_rotation_matrix(q)
        base_position = np.array([0.2 + 0.01 * idx, -0.1 + 0.005 * idx, 0.9], dtype=float)
        p_rel_fk = forward_kinematics(l2_m, arm1_deg, arm2_deg, true_theta[3], true_theta[4])
        relative_body = p_rel_fk + np.array([0.0, 0.0, -l1_m], dtype=float)
        p_b_world = base_position - rot.dot(true_theta[:3])
        target_position = p_b_world + rot.dot(relative_body)
        samples.append(
            SampleMean(
                sample_index=idx,
                start_sec=float(idx),
                end_sec=float(idx) + 2.5,
                duration_sec=2.5,
                base_position=base_position,
                base_quaternion=q,
                target_position=target_position,
                arm1_deg=arm1_deg,
                arm2_deg=arm2_deg,
                arm1_std_deg=0.0,
                arm2_std_deg=0.0,
            )
        )

    result, _, fitted_full, _, _, _, fitted_l2 = fit_stage1(samples, l1_m, l2_m, FIT_MODE_FULL)
    error = np.abs(fitted_full - true_theta)
    print("synthetic_true_theta=", true_theta.tolist())
    print("synthetic_fitted_theta=", fitted_full.tolist())
    print("synthetic_abs_error=", error.tolist())
    if not np.all(error < np.array([5e-3, 5e-3, 5e-3, 0.5, 0.5])):
        raise SystemExit("synthetic test failed")
    print("synthetic_test_passed=true")
    return 0


def main() -> int:
    args = parse_args()

    if args.synthetic_test:
        return run_synthetic_test()

    if not args.bag:
        raise SystemExit("--bag is required unless --synthetic-test is used")
    if not os.path.isfile(args.bag):
        raise SystemExit("bag not found: %s" % args.bag)

    output_dir = args.output_dir or os.path.dirname(os.path.abspath(args.bag))
    os.makedirs(output_dir, exist_ok=True)

    with rosbag.Bag(args.bag, "r") as bag:
        windows = extract_windows(bag, args.sample_topic, args.min_window_sec)
        if not windows:
            raise SystemExit("no valid sample windows found in %s" % args.sample_topic)
        samples = compute_sample_means(bag, windows, args.base_topic, args.target_topic, args.arm_real_topic)

    if not samples:
        raise SystemExit("no valid sample means were generated")

    result, initial_full, fitted_full, initial_l1_m, fitted_l1_m, initial_l2_m, fitted_l2_m = fit_stage1(samples, args.l1, args.l2, args.fit_mode)
    result_dict = build_result_dict(args.fit_mode, result, initial_full, fitted_full, samples, initial_l1_m, fitted_l1_m, initial_l2_m, fitted_l2_m)

    csv_path = os.path.join(output_dir, "sample_means.csv")
    json_path = os.path.join(output_dir, "fit_result.json")
    write_sample_csv(csv_path, samples)
    with open(json_path, "w") as f:
        json.dump(result_dict, f, indent=2, sort_keys=True)

    print("fit_mode=%s" % args.fit_mode)
    print("sample_count=%d" % len(samples))
    print("sample_csv=%s" % csv_path)
    print("fit_json=%s" % json_path)
    print("initial_rms_m=%.6f" % result_dict["initial_rms_m"])
    print("fitted_rms_m=%.6f" % result_dict["fitted_rms_m"])
    if args.fit_mode == FIT_MODE_FULL_L1_L2:
        print("fitted_l1_m=%.6f" % result_dict["fitted_l1_m"])
    if args.fit_mode in (FIT_MODE_DX_L2, FIT_MODE_FULL_L2, FIT_MODE_FULL_L1_L2):
        print("fitted_l2_m=%.6f" % result_dict["fitted_l2_m"])
    print_launch_snippet(result_dict)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
