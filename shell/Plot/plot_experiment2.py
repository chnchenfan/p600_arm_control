#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 该脚本用于离线读取实验二 rosbag，并生成对齐论文 Fig. 8 风格的结果图。
# 当前实验二的工程目标是：
# 1. 基座在世界系内按小圆轨迹运动；
# 2. 机械臂主动补偿，使末端尽量稳定在一个固定世界点；
# 3. 用离线图像把“末端稳不稳”“基座圆轨迹画没画出来”“补偿结果如何”直接表达出来。
#
# 代码设计原则：
# 1. 不改实验一绘图脚本入口，实验二单独用 plot_experiment2.py；
# 2. 底层读包/插值/正运动学直接复用 plot_experiment1.py，减少两套逻辑漂移；
# 3. 与 uam_desired_exp2.cpp 保持同一简化假设：世界系末端重建只使用基座位置 + yaw，
#    暂时不补偿 roll/pitch。

import argparse
import math
import os

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from plot_experiment1 import (
    TOPIC_ARM_DESIRED,
    TOPIC_ARM_ERROR,
    TOPIC_ARM_REAL,
    TOPIC_GUIDE_POSE,
    TOPIC_LOCAL_POSE,
    TOPIC_SETPOINT,
    TOPIC_VRPN_POSE,
    add_placeholder,
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
TOPIC_LOCAL_POSE_FULL = TOPIC_LOCAL_POSE + "__full_pose"


def parse_args():
    # 实验二的 CLI 比实验一多几个参数，主要是为了控制
    # “末端固定点如何确定” 以及 “主段从哪里开始裁剪”。
    parser = argparse.ArgumentParser(description="Plot experiment 2 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    parser.add_argument("--settle-time", type=float, default=3.0, help="Nominal settle time before circle stage")
    parser.add_argument("--hover-z", type=float, default=1.0, help="Hover z reference used in experiment 2")
    parser.add_argument("--use-initial-ee-hold", dest="use_initial_ee_hold", action="store_true")
    parser.add_argument("--no-use-initial-ee-hold", dest="use_initial_ee_hold", action="store_false")
    parser.add_argument("--ee-hold-x", type=float, default=0.0, help="Explicit EE hold x in world frame")
    parser.add_argument("--ee-hold-y", type=float, default=0.0, help="Explicit EE hold y in world frame")
    parser.add_argument("--ee-hold-z", type=float, default=0.0, help="Explicit EE hold z in world frame")
    parser.set_defaults(use_initial_ee_hold=True)
    return parser.parse_args()


def load_bag_data_exp2(bag_path):
    # 实验二除了位置，还需要基座姿态中的 yaw。
    # 因此这里对 pose 话题额外保留 [x, y, z, qx, qy, qz, qw] 的“全量版”数组。
    ensure_rosbag_available()
    topic_store = {}
    class_cache = {}

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, raw_msg, bag_time, connection_header in bag.read_messages(
            topics=[
                TOPIC_VRPN_POSE,
                TOPIC_LOCAL_POSE,
                TOPIC_SETPOINT,
                TOPIC_GUIDE_POSE,
                TOPIC_ARM_DESIRED,
                TOPIC_ARM_REAL,
                TOPIC_ARM_ERROR,
            ],
            raw=True,
            return_connection_header=True,
        ):
            msg = deserialize_dynamic_message(raw_msg, connection_header, class_cache)
            stamp = safe_stamp_to_sec(msg, bag_time.to_sec())

            if topic in (TOPIC_VRPN_POSE, TOPIC_LOCAL_POSE):
                xyz_values = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                pose_values = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
                add_sample(topic_store, topic, stamp, xyz_values)
                full_key = TOPIC_VRPN_POSE_FULL if topic == TOPIC_VRPN_POSE else TOPIC_LOCAL_POSE_FULL
                add_sample(topic_store, full_key, stamp, pose_values)
            elif topic == TOPIC_SETPOINT:
                add_sample(topic_store, topic, stamp, [msg.position.x, msg.position.y, msg.position.z])
            elif topic == TOPIC_GUIDE_POSE:
                add_sample(topic_store, topic, stamp, [msg.x_d, msg.y_d, msg.z_d])
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR):
                add_sample(topic_store, topic, stamp, [msg.arm1_angle, msg.arm2_angle, msg.hand_angle])

    output = {}
    for topic, samples in topic_store.items():
        output[topic] = samples_to_arrays(samples)
    return output


def quaternion_to_yaw(quaternion_array):
    # 将 [qx, qy, qz, qw] 转成 yaw。
    # 与控制节点保持同一提取方式，避免离线重建和在线控制不一致。
    qx = quaternion_array[:, 0]
    qy = quaternion_array[:, 1]
    qz = quaternion_array[:, 2]
    qw = quaternion_array[:, 3]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def rotate_body_to_world(body_points, yaw_array):
    # 只按 yaw 旋转机体系末端点到世界系。
    # 这是实验二首版离线重建的核心假设，与 uam_desired_exp2.cpp 一致。
    cos_yaw = np.cos(yaw_array)
    sin_yaw = np.sin(yaw_array)
    world_points = np.zeros_like(body_points)
    world_points[:, 0] = cos_yaw * body_points[:, 0] - sin_yaw * body_points[:, 1]
    world_points[:, 1] = sin_yaw * body_points[:, 0] + cos_yaw * body_points[:, 1]
    world_points[:, 2] = body_points[:, 2]
    return world_points


def compute_end_effector_body(series_values):
    # 批量正运动学：从关节角恢复末端在机体系的位置。
    positions = np.zeros((series_values.shape[0], 3), dtype=float)
    for index in range(series_values.shape[0]):
        positions[index, :] = forward_kinematics(series_values[index, 0], series_values[index, 1])
    return positions


def compute_end_effector_world(base_pose_full, arm_real):
    # 将机体系末端位置叠加到基座世界位姿上，得到世界系末端轨迹。
    base_position = base_pose_full[:, :3]
    base_yaw = quaternion_to_yaw(base_pose_full[:, 3:])
    ee_body = compute_end_effector_body(arm_real)
    ee_world_offset = rotate_body_to_world(ee_body, base_yaw)
    return base_position + ee_world_offset


def detect_circle_start_index(guide_pose, settle_time, time_array, min_consecutive=5):
    # 自动识别实验二主段开始时刻。
    # 思路：
    # 1. 先看 guide pose 的 x/y 是否开始持续偏离初始悬停点；
    # 2. 若噪声太小或检测失败，则回退到用户给的 settle_time。
    xy_ref = guide_pose[:, :2]
    head_count = min(max(10, min_consecutive + 1), xy_ref.shape[0])
    initial_center = np.median(xy_ref[:head_count, :], axis=0)
    distance = np.linalg.norm(xy_ref - initial_center, axis=1)

    max_distance = float(np.max(distance))
    threshold = max(0.005, 0.2 * max_distance)

    consecutive = 0
    detected_index = None
    for index, value in enumerate(distance):
        if value >= threshold:
            consecutive += 1
            if consecutive >= min_consecutive:
                detected_index = index - min_consecutive + 1
                break
        else:
            consecutive = 0

    settle_index = int(np.searchsorted(time_array, settle_time))
    if detected_index is None:
        return min(settle_index, time_array.shape[0] - 1)
    return max(detected_index, settle_index)


def trim_from_start(start_index, *arrays):
    trimmed = []
    for array in arrays:
        if array is None:
            trimmed.append(None)
        else:
            trimmed.append(array[start_index:])
    return trimmed


def save_main_figure(output_dir, common_time, ee_world, ee_target, base_actual, base_ref):
    # 主图对齐论文 Fig. 8 的表达重点：
    # (a) 末端稳定结果
    # (b) 基座圆轨迹
    # (c) 补偿后末端坐标时序
    figure = plt.figure(figsize=(12, 11))
    grid = gridspec.GridSpec(3, 2, figure=figure, height_ratios=[1.15, 1.0, 1.0])

    axis_a_left = figure.add_subplot(grid[0, 0], projection="3d")
    axis_a_right = figure.add_subplot(grid[0, 1])
    axis_b = figure.add_subplot(grid[1, :])
    axis_c = figure.add_subplot(grid[2, :])

    ee_error = np.linalg.norm(ee_world - ee_target, axis=1)
    ee_mean = float(np.mean(ee_error))
    ee_std = float(np.std(ee_error))

    axis_a_left.plot(ee_world[:, 0], ee_world[:, 1], ee_world[:, 2], color="#1f77b4", linewidth=1.8)
    axis_a_left.scatter(
        [ee_target[0, 0]], [ee_target[0, 1]], [ee_target[0, 2]], color="#d62728", s=50, label="Desired EE"
    )
    axis_a_left.set_title("(a) End-effector stabilization")
    axis_a_left.set_xlabel("x (m)")
    axis_a_left.set_ylabel("y (m)")
    axis_a_left.set_zlabel("z (m)")
    axis_a_left.legend(loc="best", frameon=False)

    axis_a_right.plot(common_time, ee_error, color="#1f77b4", linewidth=1.8)
    axis_a_right.set_title("EE position error norm")
    axis_a_right.set_xlabel("Time (s)")
    axis_a_right.set_ylabel("||pE - pEd|| (m)")
    axis_a_right.text(
        0.02,
        0.98,
        "mean = %.4f m\nstd = %.4f m" % (ee_mean, ee_std),
        transform=axis_a_right.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.8", alpha=0.9),
    )
    style_axis(axis_a_right)

    axis_b.plot(base_ref[:, 0], base_ref[:, 1], color="#d62728", linewidth=1.5, linestyle="--", label="Desired circle")
    axis_b.plot(base_actual[:, 0], base_actual[:, 1], color="#1f77b4", linewidth=1.8, label="Actual base")
    axis_b.set_title("(b) Base circle tracking")
    axis_b.set_xlabel("x (m)")
    axis_b.set_ylabel("y (m)")
    axis_b.legend(loc="best", frameon=False)
    axis_b.set_aspect("equal", adjustable="box")
    style_axis(axis_b)

    axis_c.plot(common_time, ee_target[:, 0], color="#1f77b4", linewidth=1.0, linestyle=":", label="x desired")
    axis_c.plot(common_time, ee_world[:, 0], color="#1f77b4", linewidth=1.8, label="x actual")
    axis_c.plot(common_time, ee_target[:, 1], color="#ff7f0e", linewidth=1.0, linestyle=":", label="y desired")
    axis_c.plot(common_time, ee_world[:, 1], color="#ff7f0e", linewidth=1.8, label="y actual")
    axis_c.plot(common_time, ee_target[:, 2], color="#2ca02c", linewidth=1.0, linestyle=":", label="z desired")
    axis_c.plot(common_time, ee_world[:, 2], color="#2ca02c", linewidth=1.8, label="z actual")
    axis_c.set_title("(c) Compensation result in world frame")
    axis_c.set_xlabel("Time (s)")
    axis_c.set_ylabel("Position (m)")
    axis_c.legend(loc="upper right", ncol=3, frameon=False, fontsize=8)
    style_axis(axis_c)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp2_main_figure.png"), dpi=300)
    plt.close(figure)


def save_end_effector_summary(output_dir, common_time, ee_world, ee_target):
    # 末端世界系跟踪汇总图，更适合做数值分析和参数微调。
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    for axis_index, axis_name in enumerate(axis_names):
        axes[axis_index].plot(
            common_time,
            ee_target[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.2,
            linestyle=":",
            label="desired %s" % axis_name,
        )
        axes[axis_index].plot(
            common_time,
            ee_world[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    _, ee_norm = compute_error(ee_world, ee_target)
    ee_mean = float(np.mean(ee_norm))
    ee_std = float(np.std(ee_norm))
    ee_max = float(np.max(ee_norm))

    axes[3].plot(common_time, ee_norm, color="#d62728", linewidth=2.0, label="EE error norm")
    axes[3].text(
        0.02,
        0.98,
        "mean = %.4f m\nstd = %.4f m\nmax = %.4f m" % (ee_mean, ee_std, ee_max),
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
    figure.savefig(os.path.join(output_dir, "exp2_end_effector_summary.png"), dpi=300)
    plt.close(figure)


def save_base_summary(output_dir, common_time, base_actual, base_ref):
    # 基座实际/期望跟踪图，与实验一新的 base summary 语义保持一致。
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    for axis_index, axis_name in enumerate(axis_names):
        axes[axis_index].plot(
            common_time,
            base_ref[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.2,
            linestyle=":",
            label="desired %s" % axis_name,
        )
        axes[axis_index].plot(
            common_time,
            base_actual[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    _, base_norm = compute_error(base_actual, base_ref)
    base_mean, base_max = series_stats(base_norm)
    axes[3].plot(common_time, base_norm, color="#d62728", linewidth=2.0, label="Base error norm")
    axes[3].text(
        0.02,
        0.98,
        "mean = %.4f m\nmax = %.4f m" % (base_mean, base_max),
        transform=axes[3].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.8", alpha=0.9),
    )
    axes[3].set_ylabel("error norm (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best", frameon=False)
    style_axis(axes[3])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp2_base_summary.png"), dpi=300)
    plt.close(figure)


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    bag_data = load_bag_data_exp2(args.bag)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [
        TOPIC_VRPN_POSE,
        TOPIC_VRPN_POSE_FULL,
        TOPIC_GUIDE_POSE,
        TOPIC_ARM_REAL,
    ]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    common_time = build_common_time(
        [
            bag_data[TOPIC_VRPN_POSE_FULL],
            bag_data[TOPIC_GUIDE_POSE],
            bag_data[TOPIC_ARM_REAL],
        ]
    )

    vrpn_pose = interp_series(bag_data[TOPIC_VRPN_POSE], common_time)
    vrpn_pose_full = interp_series(bag_data[TOPIC_VRPN_POSE_FULL], common_time)
    guide_pose = interp_series(bag_data[TOPIC_GUIDE_POSE], common_time)
    arm_real = interp_series(bag_data[TOPIC_ARM_REAL], common_time)

    landing_cut_index = detect_landing_cut_index(guide_pose)
    if 0 < landing_cut_index < common_time.shape[0]:
        common_time = common_time[:landing_cut_index]
        vrpn_pose, vrpn_pose_full, guide_pose, arm_real = [
            array[:landing_cut_index] for array in (vrpn_pose, vrpn_pose_full, guide_pose, arm_real)
        ]

    start_index = detect_circle_start_index(guide_pose, args.settle_time, common_time)
    common_time, vrpn_pose, vrpn_pose_full, guide_pose, arm_real = trim_from_start(
        start_index, common_time, vrpn_pose, vrpn_pose_full, guide_pose, arm_real
    )
    common_time = common_time - common_time[0]

    if common_time.shape[0] < 5:
        raise RuntimeError("Experiment 2 valid segment is too short after trimming")

    ee_world = compute_end_effector_world(vrpn_pose_full, arm_real)
    if args.use_initial_ee_hold:
        ee_hold_point = ee_world[0, :].copy()
    else:
        ee_hold_point = np.array([args.ee_hold_x, args.ee_hold_y, args.ee_hold_z], dtype=float)
    ee_target = np.repeat(ee_hold_point.reshape(1, 3), common_time.shape[0], axis=0)

    save_main_figure(args.output_dir, common_time, ee_world, ee_target, vrpn_pose, guide_pose)
    save_end_effector_summary(args.output_dir, common_time, ee_world, ee_target)
    save_base_summary(args.output_dir, common_time, vrpn_pose, guide_pose)

    print("Saved experiment 2 figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
