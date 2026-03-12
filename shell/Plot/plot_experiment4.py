#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from plot_experiment1 import (
    TOPIC_ARM_DESIRED,
    TOPIC_ARM_REAL,
    TOPIC_GUIDE_POSE,
    TOPIC_VRPN_POSE,
    add_sample,
    build_common_time,
    compute_error,
    deserialize_dynamic_message,
    detect_landing_cut_index,
    ensure_rosbag_available,
    forward_kinematics,
    interp_series,
    normalize_series_times,
    safe_stamp_to_sec,
    samples_to_arrays,
    series_stats,
    style_axis,
)

try:
    import rosbag
except ImportError:
    rosbag = None


TOPIC_VRPN_POSE_FULL = TOPIC_VRPN_POSE + "__full_pose"
TOPIC_GUIDE_POSE_FULL = TOPIC_GUIDE_POSE + "__full_pose"


def parse_args():
    parser = argparse.ArgumentParser(description="Plot experiment 4 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--ring-names", default="ring1,ring2,ring3,ring4", help="Comma-separated ring tracker names")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    return parser.parse_args()


def parse_ring_names(raw_value):
    names = [name.strip() for name in raw_value.split(",") if name.strip()]
    return names if names else ["ring1", "ring2", "ring3", "ring4"]


def load_bag_data_exp4(bag_path, ring_names):
    ensure_rosbag_available()
    topic_store = {}
    class_cache = {}
    ring_topics = ["/vrpn_client_node/%s/pose" % name for name in ring_names]
    topics = [
        TOPIC_VRPN_POSE,
        TOPIC_GUIDE_POSE,
        TOPIC_ARM_DESIRED,
        TOPIC_ARM_REAL,
    ] + ring_topics

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, raw_msg, bag_time, connection_header in bag.read_messages(
            topics=topics,
            raw=True,
            return_connection_header=True,
        ):
            msg = deserialize_dynamic_message(raw_msg, connection_header, class_cache)
            stamp = safe_stamp_to_sec(msg, bag_time.to_sec())

            if topic == TOPIC_VRPN_POSE:
                add_sample(topic_store, topic, stamp, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                add_sample(
                    topic_store,
                    TOPIC_VRPN_POSE_FULL,
                    stamp,
                    [
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z,
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ],
                )
            elif topic == TOPIC_GUIDE_POSE:
                add_sample(topic_store, topic, stamp, [msg.x_d, msg.y_d, msg.z_d])
                add_sample(topic_store, TOPIC_GUIDE_POSE_FULL, stamp, [msg.x_d, msg.y_d, msg.z_d, msg.yaw_d])
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_REAL):
                add_sample(topic_store, topic, stamp, [msg.arm1_angle, msg.arm2_angle, msg.hand_angle])
            elif topic in ring_topics:
                add_sample(topic_store, topic, stamp, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    output = {}
    for topic, samples in topic_store.items():
        output[topic] = samples_to_arrays(samples)
    return output


def quaternion_to_yaw(quaternion_array):
    qx = quaternion_array[:, 0]
    qy = quaternion_array[:, 1]
    qz = quaternion_array[:, 2]
    qw = quaternion_array[:, 3]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def rotate_body_to_world(body_points, yaw_array):
    cos_yaw = np.cos(yaw_array)
    sin_yaw = np.sin(yaw_array)
    world_points = np.zeros_like(body_points)
    world_points[:, 0] = cos_yaw * body_points[:, 0] - sin_yaw * body_points[:, 1]
    world_points[:, 1] = sin_yaw * body_points[:, 0] + cos_yaw * body_points[:, 1]
    world_points[:, 2] = body_points[:, 2]
    return world_points


def compute_end_effector_body(series_values):
    positions = np.zeros((series_values.shape[0], 3), dtype=float)
    for index in range(series_values.shape[0]):
        positions[index, :] = forward_kinematics(series_values[index, 0], series_values[index, 1])
    return positions


def compute_end_effector_world(base_position, yaw_array, arm_series):
    ee_body = compute_end_effector_body(arm_series)
    return base_position + rotate_body_to_world(ee_body, yaw_array)


def compute_rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


def apply_landing_cut(cut_index, common_time, *arrays):
    trimmed = [common_time[:cut_index]]
    for array in arrays:
        trimmed.append(array[:cut_index] if array is not None else None)
    return trimmed


def save_overview(output_dir, actual_base, actual_ee, ref_ee, ring_series_map):
    figure = plt.figure(figsize=(11, 8))
    axis = figure.add_subplot(111, projection="3d")

    axis.plot(actual_base[:, 0], actual_base[:, 1], actual_base[:, 2], color="#1f77b4", linewidth=1.4, label="Base actual")
    axis.plot(actual_ee[:, 0], actual_ee[:, 1], actual_ee[:, 2], color="#2ca02c", linewidth=1.8, label="EE actual")
    axis.plot(ref_ee[:, 0], ref_ee[:, 1], ref_ee[:, 2], color="#d62728", linewidth=1.6, linestyle="--", label="EE reference")

    for ring_name, values in ring_series_map.items():
        ring_center = np.median(values, axis=0)
        axis.scatter([ring_center[0]], [ring_center[1]], [ring_center[2]], s=45, label=ring_name)

    axis.set_title("Experiment 4 overview")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.legend(loc="best", frameon=False)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp4_overview.png"), dpi=300)
    plt.close(figure)


def save_tracking_figure(output_path, common_time, actual_values, reference_values, title_prefix, axis_unit):
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_names = ["x", "y", "z"]
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    diff, error_norm = compute_error(actual_values, reference_values)

    for axis_index, axis_name in enumerate(axis_names):
        axes[axis_index].plot(
            common_time,
            reference_values[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.2,
            linestyle=":",
            label="reference %s" % axis_name,
        )
        axes[axis_index].plot(
            common_time,
            actual_values[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (%s)" % (axis_name, axis_unit))
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    mean_norm, max_norm = series_stats(error_norm)
    rmse_norm = compute_rmse(error_norm)
    axes[3].plot(common_time, error_norm, color="#d62728", linewidth=2.0, label="norm error")
    axes[3].text(
        0.02,
        0.98,
        "mean = %.4f %s\nmax = %.4f %s\nrmse = %.4f %s"
        % (mean_norm, axis_unit, max_norm, axis_unit, rmse_norm, axis_unit),
        transform=axes[3].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.8", alpha=0.9),
    )
    axes[3].set_ylabel("norm (%s)" % axis_unit)
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.suptitle(title_prefix)
    figure.tight_layout()
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def save_ring_passage(output_dir, common_time, actual_ee, ref_ee, ring_series_map):
    figure = plt.figure(figsize=(12, 10))
    ring_items = list(ring_series_map.items())

    for plot_index, (ring_name, ring_values) in enumerate(ring_items, start=1):
        axis = figure.add_subplot(2, 2, plot_index, projection="3d")
        desired_distance = np.linalg.norm(ref_ee - ring_values, axis=1)
        center_index = int(np.argmin(desired_distance))
        start_index = max(0, center_index - 50)
        end_index = min(common_time.shape[0], center_index + 51)

        local_center = np.median(ring_values[start_index:end_index], axis=0)
        actual_local = actual_ee[start_index:end_index] - local_center
        ref_local = ref_ee[start_index:end_index] - local_center
        min_distance = float(np.min(np.linalg.norm(actual_ee - ring_values, axis=1)))

        axis.plot(ref_local[:, 0], ref_local[:, 1], ref_local[:, 2], color="#d62728", linestyle="--", linewidth=1.5, label="EE ref")
        axis.plot(actual_local[:, 0], actual_local[:, 1], actual_local[:, 2], color="#1f77b4", linewidth=1.8, label="EE actual")
        axis.scatter([0.0], [0.0], [0.0], color="#111111", s=35, label="Ring center")
        axis.set_title("%s  min_dist=%.3f m" % (ring_name, min_distance))
        axis.set_xlabel("dx (m)")
        axis.set_ylabel("dy (m)")
        axis.set_zlabel("dz (m)")
        axis.legend(loc="best", frameon=False, fontsize=8)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp4_ring_passage.png"), dpi=300)
    plt.close(figure)


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    ring_names = parse_ring_names(args.ring_names)
    bag_data = load_bag_data_exp4(args.bag, ring_names)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [TOPIC_VRPN_POSE, TOPIC_VRPN_POSE_FULL, TOPIC_GUIDE_POSE, TOPIC_GUIDE_POSE_FULL, TOPIC_ARM_DESIRED, TOPIC_ARM_REAL]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    ring_topics = ["/vrpn_client_node/%s/pose" % name for name in ring_names]
    for topic in ring_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required ring topic missing or empty in bag: %s" % topic)

    common_time = build_common_time(
        [
            bag_data[TOPIC_VRPN_POSE_FULL],
            bag_data[TOPIC_GUIDE_POSE_FULL],
            bag_data[TOPIC_ARM_DESIRED],
            bag_data[TOPIC_ARM_REAL],
        ]
        + [bag_data[topic] for topic in ring_topics]
    )

    actual_base_full = interp_series(bag_data[TOPIC_VRPN_POSE_FULL], common_time)
    ref_base_full = interp_series(bag_data[TOPIC_GUIDE_POSE_FULL], common_time)
    desired_arm = interp_series(bag_data[TOPIC_ARM_DESIRED], common_time)
    real_arm = interp_series(bag_data[TOPIC_ARM_REAL], common_time)

    ring_series_map = {}
    for ring_name, topic in zip(ring_names, ring_topics):
        ring_series_map[ring_name] = interp_series(bag_data[topic], common_time)

    actual_base = actual_base_full[:, :3]
    ref_base = ref_base_full[:, :3]
    actual_yaw = quaternion_to_yaw(actual_base_full[:, 3:])
    ref_yaw = ref_base_full[:, 3]
    actual_ee = compute_end_effector_world(actual_base, actual_yaw, real_arm)
    ref_ee = compute_end_effector_world(ref_base, ref_yaw, desired_arm)

    cut_index = detect_landing_cut_index(ref_base)
    if 0 < cut_index < common_time.shape[0]:
        trimmed = apply_landing_cut(
            cut_index,
            common_time,
            actual_base,
            ref_base,
            actual_ee,
            ref_ee,
            desired_arm,
            real_arm,
        )
        common_time, actual_base, ref_base, actual_ee, ref_ee, desired_arm, real_arm = trimmed
        for ring_name in ring_names:
            ring_series_map[ring_name] = ring_series_map[ring_name][:cut_index]

    save_overview(args.output_dir, actual_base, actual_ee, ref_ee, ring_series_map)
    save_tracking_figure(
        os.path.join(args.output_dir, "exp4_ee_tracking.png"),
        common_time,
        actual_ee,
        ref_ee,
        "Experiment 4 end-effector tracking",
        "m",
    )
    save_tracking_figure(
        os.path.join(args.output_dir, "exp4_base_tracking.png"),
        common_time,
        actual_base,
        ref_base,
        "Experiment 4 base tracking",
        "m",
    )
    save_ring_passage(args.output_dir, common_time, actual_ee, ref_ee, ring_series_map)

    print("Saved experiment 4 figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
