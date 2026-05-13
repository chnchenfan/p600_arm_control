#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from plot_experiment1 import (
    TOPIC_ARM_DESIRED,
    TOPIC_ARM_ERROR,
    TOPIC_ARM_REAL,
    TOPIC_GUIDE_POSE,
    TOPIC_LOCAL_POSE,
    TOPIC_SETPOINT,
    TOPIC_VRPN_POSE,
    add_sample,
    build_common_time,
    compute_error,
    detect_landing_cut_index,
    deserialize_dynamic_message,
    ensure_rosbag_available,
    interp_series,
    normalize_series_times,
    safe_stamp_to_sec,
    samples_to_arrays,
    series_stats,
    style_axis,
)


TOPIC_ARM_BASE_POSE = "/vrpn_client_node/arm_base/pose"
TOPIC_ARM_LIMITED_DESIRED = "/wjl/arm/real/angle_d"


def parse_args():
    parser = argparse.ArgumentParser(description="Plot experiment 3 figures from rosbag.")
    parser.add_argument("input", nargs="?", default="", help="Record directory or input rosbag path")
    parser.add_argument("--bag", default="", help="Input rosbag path")
    parser.add_argument("--output-dir", default="", help="Directory for PNG outputs")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    return parser.parse_args()


def compute_rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


def quaternion_to_rpy_deg(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def load_experiment3_bag_data(bag_path):
    ensure_rosbag_available()
    topic_store = {}
    class_cache = {}
    topics = [
        TOPIC_ARM_BASE_POSE,
        TOPIC_VRPN_POSE,
        TOPIC_LOCAL_POSE,
        TOPIC_SETPOINT,
        TOPIC_GUIDE_POSE,
        TOPIC_ARM_DESIRED,
        TOPIC_ARM_LIMITED_DESIRED,
        TOPIC_ARM_REAL,
        TOPIC_ARM_ERROR,
    ]
    with __import__("rosbag").Bag(bag_path, "r") as bag:
        for topic, raw_msg, bag_time, connection_header in bag.read_messages(
            topics=topics,
            raw=True,
            return_connection_header=True,
        ):
            msg = deserialize_dynamic_message(raw_msg, connection_header, class_cache)
            stamp = safe_stamp_to_sec(msg, bag_time.to_sec())
            if topic in (TOPIC_ARM_BASE_POSE, TOPIC_VRPN_POSE, TOPIC_LOCAL_POSE):
                rpy = quaternion_to_rpy_deg(msg.pose.orientation)
                values = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] + rpy
            elif topic == TOPIC_SETPOINT:
                values = [msg.position.x, msg.position.y, msg.position.z, np.nan, np.nan, math.degrees(msg.yaw)]
            elif topic == TOPIC_GUIDE_POSE:
                values = [msg.x_d, msg.y_d, msg.z_d, 0.0, 0.0, msg.yaw_d]
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_LIMITED_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR):
                values = [msg.arm1_angle, msg.arm2_angle, msg.hand_angle]
            else:
                continue
            add_sample(topic_store, topic, stamp, values)
    return {topic: samples_to_arrays(samples) for topic, samples in topic_store.items()}


def pick(row, *keys):
    for key in keys:
        if key in row and row[key] != "":
            return row[key]
    return ""


def to_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def load_monitor_csv(path, kind):
    rows = []
    if not path or not os.path.isfile(path) or os.path.getsize(path) == 0:
        return None
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            stamp = to_float(pick(raw, "%time", "time"))
            if stamp is None:
                continue
            if kind == "guide":
                values = [
                    to_float(pick(raw, "field.x_d", "x_d")),
                    to_float(pick(raw, "field.y_d", "y_d")),
                    to_float(pick(raw, "field.z_d", "z_d")),
                    0.0,
                    0.0,
                    to_float(pick(raw, "field.yaw_d", "yaw_d")),
                ]
            else:
                values = [
                    to_float(pick(raw, "field.pose.position.x", "pose.position.x")),
                    to_float(pick(raw, "field.pose.position.y", "pose.position.y")),
                    to_float(pick(raw, "field.pose.position.z", "pose.position.z")),
                    0.0,
                    0.0,
                    0.0,
                ]
            if any(value is None for value in values):
                continue
            rows.append((stamp, values))
    return samples_to_arrays(rows)


def resolve_paths(args):
    input_path = os.path.expanduser(args.bag or args.input)
    if not input_path:
        raise RuntimeError("Pass a record directory or --bag path.")
    if os.path.isdir(input_path):
        record_dir = input_path
        bag_path = os.path.join(record_dir, "experiment3_px4.bag")
        if not os.path.isfile(bag_path):
            candidates = [name for name in os.listdir(record_dir) if name.endswith(".bag")]
            if not candidates:
                raise RuntimeError("No rosbag found in directory: %s" % record_dir)
            bag_path = os.path.join(record_dir, candidates[0])
    else:
        bag_path = input_path
        record_dir = os.path.dirname(bag_path)
    output_dir = os.path.expanduser(args.output_dir) if args.output_dir else os.path.join(record_dir, "plots")
    return bag_path, record_dir, output_dir


def apply_csv_fallbacks(bag_data, record_dir):
    monitor_dir = os.path.join(record_dir, "monitor_tmp")
    if bag_data.get(TOPIC_GUIDE_POSE) is None:
        bag_data[TOPIC_GUIDE_POSE] = load_monitor_csv(os.path.join(monitor_dir, "pose_d.csv"), "guide")
    if bag_data.get(TOPIC_ARM_BASE_POSE) is None:
        bag_data[TOPIC_ARM_BASE_POSE] = load_monitor_csv(os.path.join(monitor_dir, "vrpn_pose.csv"), "pose")
    return bag_data


def first_available(series_map, topics):
    for topic in topics:
        if series_map.get(topic) is not None:
            return topic, series_map[topic]
    return "", None


def save_uav_3d_tracking(output_dir, actual_base, ref_base):
    figure = plt.figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")

    if ref_base is not None:
        axis.plot(ref_base[:, 0], ref_base[:, 1], ref_base[:, 2], color="#d62728", linestyle="--", linewidth=1.5, label="Desired")
    axis.plot(actual_base[:, 0], actual_base[:, 1], actual_base[:, 2], color="#1f77b4", linewidth=1.8, label="Actual")
    axis.set_title("UAV 3D trajectory tracking")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.legend(loc="best", frameon=False)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_uav_3d_tracking.png"), dpi=300)
    plt.close(figure)


def save_uav_axis_tracking(output_dir, common_time, actual_base, ref_base):
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    if ref_base is not None:
        _, error_norm = compute_error(actual_base, ref_base)
    else:
        error_norm = None

    for axis_index, axis_name in enumerate(axis_names):
        if ref_base is not None:
            axes[axis_index].plot(
                common_time,
                ref_base[:, axis_index],
                color=axis_colors[axis_index],
                linewidth=1.2,
                linestyle=":",
                label="desired %s" % axis_name,
            )
        axes[axis_index].plot(
            common_time,
            actual_base[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    if error_norm is not None:
        axes[3].plot(common_time, error_norm, color="#d62728", linewidth=2.0, label="||e-e_d||")
    else:
        axes[3].text(
            0.5,
            0.5,
            "Missing /wjl/guidefly/pose_d: position error cannot be computed",
            transform=axes[3].transAxes,
            ha="center",
            va="center",
            color="0.4",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.75", alpha=0.9),
        )
    axes[3].set_ylabel("norm (m)")
    axes[3].set_xlabel("Time (s)")
    if error_norm is not None:
        axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_uav_axis_tracking.png"), dpi=300)
    plt.close(figure)


def save_uav_attitude_tracking(output_dir, common_time, actual_pose, ref_pose, setpoint_pose):
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    names = ["roll", "pitch", "yaw"]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]

    for axis_index, axis_name in enumerate(names):
        value_index = axis_index + 3
        if ref_pose is not None:
            axes[axis_index].plot(
                common_time,
                ref_pose[:, value_index],
                color=colors[axis_index],
                linewidth=1.2,
                linestyle=":",
                label="desired %s" % axis_name,
            )
        if setpoint_pose is not None and np.any(np.isfinite(setpoint_pose[:, value_index])):
            axes[axis_index].plot(
                common_time,
                setpoint_pose[:, value_index],
                color="#9467bd",
                linewidth=1.3,
                linestyle="--",
                label="setpoint %s" % axis_name,
            )
        axes[axis_index].plot(
            common_time,
            actual_pose[:, value_index],
            color=colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (deg)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    axes[2].set_xlabel("Time (s)")
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_uav_attitude_tracking.png"), dpi=300)
    plt.close(figure)


def save_arm_tracking(output_dir, common_time, desired_arm, limited_arm, real_arm, error_arm):
    figure, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axes[0].plot(common_time, desired_arm[:, 0], color="#111111", linewidth=2.0, label="arm1 desired")
    if limited_arm is not None:
        axes[0].plot(common_time, limited_arm[:, 0], color="#9467bd", linewidth=1.4, linestyle="--", label="arm1 limited desired")
    axes[0].plot(common_time, real_arm[:, 0], color="#d62728", linewidth=1.8, label="arm1 actual")
    axes[0].set_ylabel("arm1 (deg)")
    axes[0].set_title("Arm tracking")
    axes[0].legend(loc="best", frameon=False)
    style_axis(axes[0])

    axes[1].plot(common_time, desired_arm[:, 1], color="#111111", linewidth=2.0, label="arm2 desired")
    if limited_arm is not None:
        axes[1].plot(common_time, limited_arm[:, 1], color="#9467bd", linewidth=1.4, linestyle="--", label="arm2 limited desired")
    axes[1].plot(common_time, real_arm[:, 1], color="#1f77b4", linewidth=1.8, label="arm2 actual")
    axes[1].set_ylabel("arm2 (deg)")
    axes[1].legend(loc="best", frameon=False)
    style_axis(axes[1])

    axes[2].plot(common_time, desired_arm[:, 2], color="#111111", linewidth=2.0, label="hand desired")
    if limited_arm is not None:
        axes[2].plot(common_time, limited_arm[:, 2], color="#9467bd", linewidth=1.4, linestyle="--", label="hand limited desired")
    axes[2].plot(common_time, real_arm[:, 2], color="#2ca02c", linewidth=1.8, label="hand actual")
    axes[2].set_ylabel("hand (deg)")
    axes[2].legend(loc="best", frameon=False)
    style_axis(axes[2])

    axes[3].plot(common_time, error_arm[:, 0], color="#d62728", linewidth=1.5, label="arm1 error")
    axes[3].plot(common_time, error_arm[:, 1], color="#1f77b4", linewidth=1.5, label="arm2 error")
    axes[3].plot(common_time, error_arm[:, 2], color="#2ca02c", linewidth=1.2, label="hand error")
    axes[3].axhline(0.0, color="0.35", linewidth=1.0, linestyle=":")
    axes[3].set_ylabel("error")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_arm_angle_tracking.png"), dpi=300)
    plt.close(figure)


def save_uav_error_summary(output_dir, common_time, actual_base, ref_base):
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    diff, error_norm = compute_error(actual_base, ref_base)

    rmse_x = compute_rmse(diff[:, 0])
    rmse_y = compute_rmse(diff[:, 1])
    rmse_z = compute_rmse(diff[:, 2])
    rmse_norm = compute_rmse(error_norm)

    for axis_index, axis_name in enumerate(axis_names):
        axes[axis_index].plot(
            common_time,
            diff[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="e_%s" % axis_name,
        )
        axes[axis_index].axhline(0.0, color="0.35", linewidth=1.0, linestyle=":")
        axes[axis_index].set_ylabel("%s err (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    mean_norm, max_norm = series_stats(error_norm)
    axes[3].plot(common_time, error_norm, color="#d62728", linewidth=2.0, label="||e-e_d||")
    axes[3].text(
        0.02,
        0.98,
        "rmse_x = %.4f m\nrmse_y = %.4f m\nrmse_z = %.4f m\nrmse_norm = %.4f m\nmean_norm = %.4f m\nmax_norm = %.4f m"
        % (rmse_x, rmse_y, rmse_z, rmse_norm, mean_norm, max_norm),
        transform=axes[3].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.8", alpha=0.9),
    )
    axes[3].set_ylabel("norm (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_uav_error_summary.png"), dpi=300)
    plt.close(figure)


def main():
    args = parse_args()
    bag_path, record_dir, output_dir = resolve_paths(args)
    os.makedirs(output_dir, exist_ok=True)

    bag_data = load_experiment3_bag_data(bag_path)
    bag_data = apply_csv_fallbacks(bag_data, record_dir)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    actual_pose_topic, actual_pose_series = first_available(
        bag_data,
        [TOPIC_ARM_BASE_POSE, TOPIC_VRPN_POSE, TOPIC_LOCAL_POSE],
    )
    required_topics = [TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)
    if actual_pose_series is None:
        raise RuntimeError("Required pose topic missing: expected arm_base, Tracker0, or mavros local pose")

    pose_time_sources = [actual_pose_series]
    if bag_data.get(TOPIC_GUIDE_POSE) is not None:
        pose_time_sources.append(bag_data[TOPIC_GUIDE_POSE])
    common_time = build_common_time(pose_time_sources)

    actual_base = interp_series(actual_pose_series, common_time)
    ref_base = interp_series(bag_data.get(TOPIC_GUIDE_POSE), common_time) if bag_data.get(TOPIC_GUIDE_POSE) is not None else None
    setpoint_base = interp_series(bag_data.get(TOPIC_SETPOINT), common_time) if bag_data.get(TOPIC_SETPOINT) is not None else None

    common_arm_time = build_common_time(
        [
            bag_data[TOPIC_ARM_DESIRED],
            bag_data[TOPIC_ARM_REAL],
            bag_data[TOPIC_ARM_ERROR],
        ]
        + ([bag_data[TOPIC_ARM_LIMITED_DESIRED]] if bag_data.get(TOPIC_ARM_LIMITED_DESIRED) is not None else [])
    )
    desired_arm = interp_series(bag_data[TOPIC_ARM_DESIRED], common_arm_time)
    real_arm = interp_series(bag_data[TOPIC_ARM_REAL], common_arm_time)
    error_arm = interp_series(bag_data[TOPIC_ARM_ERROR], common_arm_time)
    limited_arm = interp_series(bag_data.get(TOPIC_ARM_LIMITED_DESIRED), common_arm_time) if bag_data.get(TOPIC_ARM_LIMITED_DESIRED) is not None else None

    if ref_base is not None:
        landing_cut_index = detect_landing_cut_index(ref_base[:, :3])
    else:
        landing_cut_index = common_time.shape[0]
    if 0 < landing_cut_index < common_time.shape[0]:
        common_time = common_time[:landing_cut_index]
        actual_base = actual_base[:landing_cut_index]
        if ref_base is not None:
            ref_base = ref_base[:landing_cut_index]
        if setpoint_base is not None:
            setpoint_base = setpoint_base[:landing_cut_index]

    save_uav_3d_tracking(output_dir, actual_base[:, :3], ref_base[:, :3] if ref_base is not None else None)
    save_uav_axis_tracking(output_dir, common_time, actual_base[:, :3], ref_base[:, :3] if ref_base is not None else None)
    if ref_base is not None:
        save_uav_error_summary(output_dir, common_time, actual_base[:, :3], ref_base[:, :3])
    save_uav_attitude_tracking(output_dir, common_time, actual_base, ref_base, setpoint_base)
    save_arm_tracking(output_dir, common_arm_time, desired_arm, limited_arm, real_arm, error_arm)

    with open(os.path.join(output_dir, "exp3_plot_summary.txt"), "w") as handle:
        handle.write("bag=%s\n" % bag_path)
        handle.write("actual_pose_topic=%s\n" % actual_pose_topic)
        for topic in [
            TOPIC_ARM_BASE_POSE,
            TOPIC_VRPN_POSE,
            TOPIC_LOCAL_POSE,
            TOPIC_SETPOINT,
            TOPIC_GUIDE_POSE,
            TOPIC_ARM_DESIRED,
            TOPIC_ARM_LIMITED_DESIRED,
            TOPIC_ARM_REAL,
            TOPIC_ARM_ERROR,
        ]:
            series = bag_data.get(topic)
            handle.write("%s=%s\n" % (topic, "missing" if series is None else "%d samples" % series["time"].shape[0]))

    print("Saved experiment 3 figures to %s" % output_dir)


if __name__ == "__main__":
    main()
