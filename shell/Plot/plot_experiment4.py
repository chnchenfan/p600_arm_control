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
    add_sample,
    build_common_time,
    compute_error,
    deserialize_dynamic_message,
    ensure_rosbag_available,
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


TOPIC_LOCAL_POSE = "/mavros/local_position/pose"
TOPIC_ARM_BASE_WORLD = "/vrpn_client_node/arm_base/pose"


def parse_args():
    parser = argparse.ArgumentParser(description="Plot experiment 4 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--target-names", default="target0,target1,target2", help="Comma-separated target tracker names")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    return parser.parse_args()


def parse_target_names(raw_value):
    names = [name.strip() for name in raw_value.split(",") if name.strip()]
    return names if names else ["target0", "target1", "target2"]


def load_bag_data_exp4(bag_path, target_names):
    ensure_rosbag_available()
    topic_store = {}
    class_cache = {}
    target_topics = ["/vrpn_client_node/%s/pose" % name for name in target_names]
    topics = [
        TOPIC_LOCAL_POSE,
        TOPIC_ARM_BASE_WORLD,
        TOPIC_GUIDE_POSE,
        TOPIC_ARM_DESIRED,
        TOPIC_ARM_REAL,
    ] + target_topics

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, raw_msg, bag_time, connection_header in bag.read_messages(
            topics=topics,
            raw=True,
            return_connection_header=True,
        ):
            msg = deserialize_dynamic_message(raw_msg, connection_header, class_cache)
            stamp = safe_stamp_to_sec(msg, bag_time.to_sec())

            if topic in (TOPIC_LOCAL_POSE, TOPIC_ARM_BASE_WORLD) or topic in target_topics:
                add_sample(topic_store, topic, stamp, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            elif topic == TOPIC_GUIDE_POSE:
                add_sample(topic_store, topic, stamp, [msg.x_d, msg.y_d, msg.z_d, msg.yaw_d])
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_REAL):
                add_sample(topic_store, topic, stamp, [msg.arm1_angle, msg.arm2_angle, msg.hand_angle])

    output = {}
    for topic, samples in topic_store.items():
        output[topic] = samples_to_arrays(samples)
    return output


def compute_rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


def save_overview(output_dir, actual_work, ref_work, target_series_map):
    figure = plt.figure(figsize=(11, 8))
    axis = figure.add_subplot(111, projection="3d")

    axis.plot(actual_work[:, 0], actual_work[:, 1], actual_work[:, 2], color="#1f77b4", linewidth=1.8, label="arm_base actual")
    axis.plot(ref_work[:, 0], ref_work[:, 1], ref_work[:, 2], color="#d62728", linewidth=1.4, linestyle="--", label="arm_base ref")

    for target_name, values in target_series_map.items():
        center = np.median(values, axis=0)
        axis.scatter([center[0]], [center[1]], [center[2]], s=45, label=target_name)

    axis.set_title("Experiment 4 weaving overview")
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


def save_target_passage(output_dir, actual_work, ref_work, target_series_map):
    figure = plt.figure(figsize=(12, 4 * max(1, len(target_series_map))))
    target_items = list(target_series_map.items())

    for plot_index, (target_name, target_values) in enumerate(target_items, start=1):
        axis = figure.add_subplot(len(target_items), 1, plot_index, projection="3d")
        target_center = np.median(target_values, axis=0)
        actual_local = actual_work - target_center
        ref_local = ref_work - target_center
        min_distance = float(np.min(np.linalg.norm(actual_work - target_values, axis=1)))

        axis.plot(ref_local[:, 0], ref_local[:, 1], ref_local[:, 2], color="#d62728", linestyle="--", linewidth=1.5, label="ref")
        axis.plot(actual_local[:, 0], actual_local[:, 1], actual_local[:, 2], color="#1f77b4", linewidth=1.8, label="actual")
        axis.scatter([0.0], [0.0], [0.0], color="#111111", s=35, label="target rigid")
        axis.set_title("%s  min_dist=%.3f m" % (target_name, min_distance))
        axis.set_xlabel("dx (m)")
        axis.set_ylabel("dy (m)")
        axis.set_zlabel("dz (m)")
        axis.legend(loc="best", frameon=False, fontsize=8)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp4_target_passage.png"), dpi=300)
    plt.close(figure)


def save_first_target1_axis_entry(output_dir, common_time, actual_work, ref_work):
    initial_x = ref_work[0, 0]
    initial_y = ref_work[0, 1]
    eps = 0.01
    y_start_index = int(np.argmax(np.abs(ref_work[:, 1] - initial_y) > eps))
    x_start_index = int(np.argmax(np.abs(ref_work[:, 0] - initial_x) > eps))
    if y_start_index == 0 and not (np.abs(ref_work[0, 1] - initial_y) > eps):
        y_start_index = 0
    if x_start_index == 0 and not (np.abs(ref_work[0, 0] - initial_x) > eps):
        x_start_index = common_time.shape[0] - 1

    end_index = min(common_time.shape[0], x_start_index + 180)

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ["x", "y", "z"]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for axis_index in range(3):
        axes[axis_index].plot(common_time[:end_index], ref_work[:end_index, axis_index], linestyle=":", linewidth=1.2,
                              color=colors[axis_index], label="ref %s" % labels[axis_index])
        axes[axis_index].plot(common_time[:end_index], actual_work[:end_index, axis_index], linewidth=1.8,
                              color=colors[axis_index], label="actual %s" % labels[axis_index])
        axes[axis_index].axvline(common_time[y_start_index], color="#666666", linestyle="--", linewidth=1.0)
        axes[axis_index].axvline(common_time[min(x_start_index, end_index - 1)], color="#aa0000", linestyle="--", linewidth=1.0)
        axes[axis_index].legend(loc="best", frameon=False)
        axes[axis_index].set_ylabel("%s (m)" % labels[axis_index])
        style_axis(axes[axis_index])

    axes[0].set_title("First target1 entry: phase_y then phase_x")
    axes[-1].set_xlabel("Time (s)")
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp4_first_target1_axis_entry.png"), dpi=300)
    plt.close(figure)


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    target_names = parse_target_names(args.target_names)
    bag_data = load_bag_data_exp4(args.bag, target_names)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [TOPIC_LOCAL_POSE, TOPIC_ARM_BASE_WORLD, TOPIC_GUIDE_POSE, TOPIC_ARM_DESIRED, TOPIC_ARM_REAL]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    target_topics = ["/vrpn_client_node/%s/pose" % name for name in target_names]
    for topic in target_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required target topic missing or empty in bag: %s" % topic)

    common_time = build_common_time(
        [bag_data[TOPIC_LOCAL_POSE], bag_data[TOPIC_ARM_BASE_WORLD], bag_data[TOPIC_GUIDE_POSE], bag_data[TOPIC_ARM_DESIRED], bag_data[TOPIC_ARM_REAL]]
        + [bag_data[topic] for topic in target_topics]
    )

    actual_work = interp_series(bag_data[TOPIC_LOCAL_POSE], common_time)
    arm_base_world = interp_series(bag_data[TOPIC_ARM_BASE_WORLD], common_time)
    ref_work_full = interp_series(bag_data[TOPIC_GUIDE_POSE], common_time)
    ref_work = ref_work_full[:, :3]

    local_shift = np.median(actual_work - arm_base_world, axis=0)
    target_series_map = {}
    for target_name, topic in zip(target_names, target_topics):
        target_world = interp_series(bag_data[topic], common_time)
        target_series_map[target_name] = target_world + local_shift.reshape(1, 3)

    save_overview(args.output_dir, actual_work, ref_work, target_series_map)
    save_tracking_figure(
        os.path.join(args.output_dir, "exp4_arm_base_tracking.png"),
        common_time,
        actual_work,
        ref_work,
        "Experiment 4 arm_base tracking",
        "m",
    )
    save_target_passage(args.output_dir, actual_work, ref_work, target_series_map)
    save_first_target1_axis_entry(args.output_dir, common_time, actual_work, ref_work)

    print("Saved experiment 4 figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
