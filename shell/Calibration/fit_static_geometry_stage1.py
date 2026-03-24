#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    parser = argparse.ArgumentParser(description="Fit stage-1 static calibration parameters from rosbag.")
    parser.add_argument("--bag", default=None, help="Input rosbag path")
    parser.add_argument("--output-dir", default=None, help="Output directory for fit_result.json and sample_means.csv")
    parser.add_argument("--l1", type=float, default=0.161, help="Current measured L1 in meters")
    parser.add_argument("--l2", type=float, default=0.262, help="Current measured L2 in meters")
    parser.add_argument("--min-window-sec", type=float, default=2.0, help="Minimum valid sample window duration")
    parser.add_argument("--base-topic", default=TOPIC_BASE)
    parser.add_argument("--target-topic", default=TOPIC_TARGET)
    parser.add_argument("--arm-real-topic", default=TOPIC_ARM_REAL)
    parser.add_argument("--sample-topic", default=TOPIC_SAMPLE_INDEX)
    parser.add_argument("--synthetic-test", action="store_true", help="Run a self-check on synthetic data instead of reading a bag")
    return parser.parse_args()


def stamp_to_sec(stamp) -> float:
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


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
    q1 = math.radians(arm1_deg + dq1_deg)
    q2 = math.radians(arm2_deg + dq2_deg)
    return np.array(
        [
            l2_m * math.cos(q2) * math.cos(q1),
            l2_m * math.cos(q2) * math.sin(q1),
            l2_m * math.sin(q2),
        ],
        dtype=float,
    )


def compute_residual_vector(theta: np.ndarray, samples: Sequence[SampleMean], l1_m: float, l2_m: float) -> np.ndarray:
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


def residual_norms(theta: np.ndarray, samples: Sequence[SampleMean], l1_m: float, l2_m: float) -> List[float]:
    vec = compute_residual_vector(theta, samples, l1_m, l2_m)
    reshaped = vec.reshape((-1, 3))
    return [float(np.linalg.norm(row)) for row in reshaped]


def rms_from_residual_vector(residual_vector: np.ndarray) -> float:
    if residual_vector.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(residual_vector))))


def fit_stage1(samples: Sequence[SampleMean], l1_m: float, l2_m: float):
    initial_theta = np.zeros(5, dtype=float)
    lower = np.array([-0.08, -0.08, -0.08, -20.0, -20.0], dtype=float)
    upper = np.array([0.08, 0.08, 0.08, 20.0, 20.0], dtype=float)
    result = least_squares(
        compute_residual_vector,
        x0=initial_theta,
        bounds=(lower, upper),
        args=(samples, l1_m, l2_m),
    )
    return result, initial_theta


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
    with open(path, "w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "sample_index": sample.sample_index,
                    "start_sec": f"{sample.start_sec:.6f}",
                    "end_sec": f"{sample.end_sec:.6f}",
                    "duration_sec": f"{sample.duration_sec:.6f}",
                    "base_x": f"{sample.base_position[0]:.9f}",
                    "base_y": f"{sample.base_position[1]:.9f}",
                    "base_z": f"{sample.base_position[2]:.9f}",
                    "base_qx": f"{sample.base_quaternion[0]:.9f}",
                    "base_qy": f"{sample.base_quaternion[1]:.9f}",
                    "base_qz": f"{sample.base_quaternion[2]:.9f}",
                    "base_qw": f"{sample.base_quaternion[3]:.9f}",
                    "target_x": f"{sample.target_position[0]:.9f}",
                    "target_y": f"{sample.target_position[1]:.9f}",
                    "target_z": f"{sample.target_position[2]:.9f}",
                    "arm1_deg": f"{sample.arm1_deg:.6f}",
                    "arm2_deg": f"{sample.arm2_deg:.6f}",
                    "arm1_std_deg": f"{sample.arm1_std_deg:.6f}",
                    "arm2_std_deg": f"{sample.arm2_std_deg:.6f}",
                }
            )


def write_fit_json(
    path: str,
    args: argparse.Namespace,
    samples: Sequence[SampleMean],
    initial_theta: np.ndarray,
    result,
) -> None:
    initial_residuals = compute_residual_vector(initial_theta, samples, args.l1, args.l2)
    optimized_residuals = compute_residual_vector(result.x, samples, args.l1, args.l2)
    initial_norms = residual_norms(initial_theta, samples, args.l1, args.l2)
    optimized_norms = residual_norms(result.x, samples, args.l1, args.l2)

    payload = {
        "bag": os.path.abspath(args.bag),
        "sample_count": len(samples),
        "model": {
            "l1_m": args.l1,
            "l2_m": args.l2,
            "parameter_vector": [
                "arm_base_offset_x_m",
                "arm_base_offset_y_m",
                "arm_base_offset_z_m",
                "arm1_zero_offset_deg",
                "arm2_zero_offset_deg",
            ],
        },
        "initial_guess": {
            "arm_base_offset_x_m": float(initial_theta[0]),
            "arm_base_offset_y_m": float(initial_theta[1]),
            "arm_base_offset_z_m": float(initial_theta[2]),
            "arm1_zero_offset_deg": float(initial_theta[3]),
            "arm2_zero_offset_deg": float(initial_theta[4]),
        },
        "best_fit": {
            "arm_base_offset_x_m": float(result.x[0]),
            "arm_base_offset_y_m": float(result.x[1]),
            "arm_base_offset_z_m": float(result.x[2]),
            "arm1_zero_offset_deg": float(result.x[3]),
            "arm2_zero_offset_deg": float(result.x[4]),
        },
        "rms_before_m": rms_from_residual_vector(initial_residuals),
        "rms_after_m": rms_from_residual_vector(optimized_residuals),
        "max_sample_residual_before_m": max(initial_norms) if initial_norms else 0.0,
        "max_sample_residual_after_m": max(optimized_norms) if optimized_norms else 0.0,
        "solver": {
            "success": bool(result.success),
            "status": int(result.status),
            "message": str(result.message),
            "nfev": int(result.nfev),
            "cost": float(result.cost),
        },
        "per_sample_residual_m": [
            {
                "sample_index": sample.sample_index,
                "before_m": initial_norms[idx],
                "after_m": optimized_norms[idx],
            }
            for idx, sample in enumerate(samples)
        ],
    }
    with open(path, "w") as fp:
        json.dump(payload, fp, indent=2, sort_keys=True)


def print_launch_snippet(theta: np.ndarray) -> None:
    print("\nSuggested launch params:")
    print(f'<param name="arm_base_offset_x_m" value="{theta[0]:.6f}" />')
    print(f'<param name="arm_base_offset_y_m" value="{theta[1]:.6f}" />')
    print(f'<param name="arm_base_offset_z_m" value="{theta[2]:.6f}" />')
    print(f'<param name="arm1_zero_offset_deg" value="{theta[3]:.6f}" />')
    print(f'<param name="arm2_zero_offset_deg" value="{theta[4]:.6f}" />')


def run_synthetic_test() -> int:
    synthetic_samples: List[SampleMean] = []
    true_theta = np.array([0.037, -0.006, 0.004, 2.0, -1.5], dtype=float)
    l1 = 0.161
    l2 = 0.262
    sequence = [(-60.0, 0.0), (-30.0, 0.0), (0.0, 0.0), (30.0, 15.0), (0.0, 30.0), (45.0, 15.0)]
    noise_rng = np.random.default_rng(7)

    for idx, (arm1_deg, arm2_deg) in enumerate(sequence):
        yaw = math.radians(-30.0 + idx * 12.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        q = np.array([0.0, 0.0, sy, cy], dtype=float)
        rot = quaternion_to_rotation_matrix(q)
        base_rb = np.array([0.4 + idx * 0.02, -0.2 + idx * 0.01, 0.9], dtype=float)
        p_rel_fk = forward_kinematics(l2, arm1_deg, arm2_deg, true_theta[3], true_theta[4])
        relative_body = p_rel_fk + np.array([0.0, 0.0, -l1], dtype=float)
        base_true_world = base_rb - rot.dot(true_theta[:3])
        target_world = base_true_world + rot.dot(relative_body)
        base_rb = base_rb + noise_rng.normal(0.0, 0.0005, size=3)
        target_world = target_world + noise_rng.normal(0.0, 0.0005, size=3)
        synthetic_samples.append(
            SampleMean(
                sample_index=idx,
                start_sec=float(idx),
                end_sec=float(idx) + 2.5,
                duration_sec=2.5,
                base_position=base_rb,
                base_quaternion=q,
                target_position=target_world,
                arm1_deg=arm1_deg,
                arm2_deg=arm2_deg,
                arm1_std_deg=0.0,
                arm2_std_deg=0.0,
            )
        )

    result, _ = fit_stage1(synthetic_samples, l1, l2)
    error = np.abs(result.x - true_theta)
    print("synthetic true theta:", true_theta)
    print("synthetic fitted theta:", result.x)
    print("synthetic abs error:", error)
    passed = np.all(error[:3] < 0.005) and np.all(error[3:] < 0.8)
    print("synthetic_test_passed=%s" % ("true" if passed else "false"))
    return 0 if passed else 1


def main() -> int:
    args = parse_args()
    if args.synthetic_test:
        return run_synthetic_test()

    if not args.bag:
        raise SystemExit("--bag is required unless --synthetic-test is used")

    bag_path = os.path.abspath(args.bag)
    if args.output_dir is None:
        output_dir = os.path.join(os.path.dirname(bag_path), "fit_static_geometry_stage1")
    else:
        output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    with rosbag.Bag(bag_path, "r") as bag:
        windows = extract_windows(bag, args.sample_topic, args.min_window_sec)
        if not windows:
            raise SystemExit("No valid sample windows found in %s" % args.sample_topic)
        samples = compute_sample_means(bag, windows, args.base_topic, args.target_topic, args.arm_real_topic)

    if not samples:
        raise SystemExit("No complete samples found after averaging")

    result, initial_theta = fit_stage1(samples, args.l1, args.l2)
    csv_path = os.path.join(output_dir, "sample_means.csv")
    json_path = os.path.join(output_dir, "fit_result.json")
    write_sample_csv(csv_path, samples)
    write_fit_json(json_path, args, samples, initial_theta, result)

    print("sample_count=%d" % len(samples))
    print("rms_before_m=%.6f" % rms_from_residual_vector(compute_residual_vector(initial_theta, samples, args.l1, args.l2)))
    print("rms_after_m=%.6f" % rms_from_residual_vector(compute_residual_vector(result.x, samples, args.l1, args.l2)))
    print("sample_means_csv=%s" % csv_path)
    print("fit_result_json=%s" % json_path)
    print_launch_snippet(result.x)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
