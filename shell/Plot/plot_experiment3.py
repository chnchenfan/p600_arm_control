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
    TOPIC_ARM_ERROR,
    TOPIC_ARM_REAL,
    TOPIC_GUIDE_POSE,
    TOPIC_VRPN_POSE,
    build_common_time,
    compute_error,
    detect_landing_cut_index,
    interp_series,
    load_bag_data,
    normalize_series_times,
    series_stats,
    style_axis,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Plot experiment 3 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    return parser.parse_args()


def compute_rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


def save_uav_3d_tracking(output_dir, actual_base, ref_base):
    figure = plt.figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")

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

    _, error_norm = compute_error(actual_base, ref_base)

    for axis_index, axis_name in enumerate(axis_names):
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

    axes[3].plot(common_time, error_norm, color="#d62728", linewidth=2.0, label="||e-e_d||")
    axes[3].set_ylabel("norm (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_uav_axis_tracking.png"), dpi=300)
    plt.close(figure)


def save_arm_tracking(output_dir, common_time, desired_arm, real_arm, error_arm):
    figure, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(common_time, desired_arm[:, 0], color="#111111", linewidth=2.0, label="arm1 desired")
    axes[0].plot(common_time, real_arm[:, 0], color="#d62728", linewidth=1.8, label="arm1 actual")
    axes[0].set_ylabel("arm1 (deg)")
    axes[0].set_title("Arm tracking")
    axes[0].legend(loc="best", frameon=False)
    style_axis(axes[0])

    axes[1].plot(common_time, desired_arm[:, 1], color="#111111", linewidth=2.0, label="arm2 desired")
    axes[1].plot(common_time, real_arm[:, 1], color="#1f77b4", linewidth=1.8, label="arm2 actual")
    axes[1].set_ylabel("arm2 (deg)")
    axes[1].legend(loc="best", frameon=False)
    style_axis(axes[1])

    axes[2].plot(common_time, error_arm[:, 0], color="#d62728", linewidth=1.5, label="arm1 error")
    axes[2].plot(common_time, error_arm[:, 1], color="#1f77b4", linewidth=1.5, label="arm2 error")
    axes[2].axhline(0.0, color="0.35", linewidth=1.0, linestyle=":")
    axes[2].set_ylabel("error")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best", frameon=False)
    style_axis(axes[2])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp3_arm_tracking.png"), dpi=300)
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
    os.makedirs(args.output_dir, exist_ok=True)

    bag_data = load_bag_data(args.bag)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [TOPIC_VRPN_POSE, TOPIC_GUIDE_POSE, TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    common_time = build_common_time(
        [
            bag_data[TOPIC_VRPN_POSE],
            bag_data[TOPIC_GUIDE_POSE],
            bag_data[TOPIC_ARM_DESIRED],
            bag_data[TOPIC_ARM_REAL],
        ]
    )

    actual_base = interp_series(bag_data[TOPIC_VRPN_POSE], common_time)
    ref_base = interp_series(bag_data[TOPIC_GUIDE_POSE], common_time)
    desired_arm = interp_series(bag_data[TOPIC_ARM_DESIRED], common_time)
    real_arm = interp_series(bag_data[TOPIC_ARM_REAL], common_time)
    error_arm = interp_series(bag_data[TOPIC_ARM_ERROR], common_time)

    landing_cut_index = detect_landing_cut_index(ref_base)
    if 0 < landing_cut_index < common_time.shape[0]:
        common_time = common_time[:landing_cut_index]
        actual_base = actual_base[:landing_cut_index]
        ref_base = ref_base[:landing_cut_index]
        desired_arm = desired_arm[:landing_cut_index]
        real_arm = real_arm[:landing_cut_index]
        error_arm = error_arm[:landing_cut_index]

    save_uav_3d_tracking(args.output_dir, actual_base, ref_base)
    save_uav_axis_tracking(args.output_dir, common_time, actual_base, ref_base)
    save_arm_tracking(args.output_dir, common_time, desired_arm, real_arm, error_arm)
    save_uav_error_summary(args.output_dir, common_time, actual_base, ref_base)

    print("Saved experiment 3 figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
