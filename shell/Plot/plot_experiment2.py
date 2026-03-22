#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""实验二绘图脚本。

输入：实验二 rosbag。
输出：末端定点效果、基座 x-z 圆弧跟踪、关节跟踪三类图。

与旧版的区别：
1. 直接使用 arm_base 作为基座真实位姿；
2. 直接使用 arm_target 作为末端真实位姿；
3. 不再使用 yaw-only 正运动学去重建末端。
"""

import argparse
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
    add_sample,
    build_common_time,
    compute_error,
    deserialize_dynamic_message,
    detect_landing_cut_index,
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


# 实验二核心动捕输入：
# - arm_base: 机械臂基座刚体位姿
# - arm_target: 末端工作点刚体位姿
TOPIC_BASE_POSE = "/vrpn_client_node/arm_base/pose"
TOPIC_EE_POSE = "/vrpn_client_node/arm_target/pose"
TOPIC_BASE_POSE_FULL = TOPIC_BASE_POSE + "__full_pose"
TOPIC_EE_POSE_FULL = TOPIC_EE_POSE + "__full_pose"
TOPIC_LOCAL_POSE_FULL = TOPIC_LOCAL_POSE + "__full_pose"
TOPIC_ONLINE_OFFSET = "/wjl/arm/guidefly/online_offset"


def parse_args():
    """解析命令行参数。

    主要控制：
    - 读取哪个 bag；
    - 图片输出目录；
    - 是否使用主段起始时刻的末端位置作为固定点目标。
    """
    parser = argparse.ArgumentParser(description="Plot experiment 2 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    parser.add_argument("--settle-time", type=float, default=3.0, help="Nominal settle time before arc stage")
    parser.add_argument("--use-initial-ee-hold", dest="use_initial_ee_hold", action="store_true")
    parser.add_argument("--no-use-initial-ee-hold", dest="use_initial_ee_hold", action="store_false")
    parser.add_argument("--ee-hold-x", type=float, default=0.0, help="Explicit EE hold x in world frame")
    parser.add_argument("--ee-hold-y", type=float, default=0.0, help="Explicit EE hold y in world frame")
    parser.add_argument("--ee-hold-z", type=float, default=0.0, help="Explicit EE hold z in world frame")
    parser.set_defaults(use_initial_ee_hold=True)
    return parser.parse_args()


def load_bag_data_exp2(bag_path):
    """读取实验二 rosbag，并整理成统一的时间序列字典。

    PoseStamped 话题同时保留：
    - xyz 序列：用于常规插值和误差计算；
    - full pose 序列：预留给需要姿态的后续分析。
    """
    ensure_rosbag_available()
    topic_store = {}
    class_cache = {}

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, raw_msg, bag_time, connection_header in bag.read_messages(
            topics=[
                TOPIC_BASE_POSE,
                TOPIC_EE_POSE,
                TOPIC_LOCAL_POSE,
                TOPIC_SETPOINT,
                TOPIC_GUIDE_POSE,
                TOPIC_ARM_DESIRED,
                TOPIC_ARM_REAL,
                TOPIC_ARM_ERROR,
                TOPIC_ONLINE_OFFSET,
            ],
            raw=True,
            return_connection_header=True,
        ):
            msg = deserialize_dynamic_message(raw_msg, connection_header, class_cache)
            stamp = safe_stamp_to_sec(msg, bag_time.to_sec())

            if topic in (TOPIC_BASE_POSE, TOPIC_EE_POSE, TOPIC_LOCAL_POSE):
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
                if topic == TOPIC_BASE_POSE:
                    add_sample(topic_store, TOPIC_BASE_POSE_FULL, stamp, pose_values)
                elif topic == TOPIC_EE_POSE:
                    add_sample(topic_store, TOPIC_EE_POSE_FULL, stamp, pose_values)
                else:
                    add_sample(topic_store, TOPIC_LOCAL_POSE_FULL, stamp, pose_values)
            elif topic == TOPIC_SETPOINT:
                add_sample(topic_store, topic, stamp, [msg.position.x, msg.position.y, msg.position.z])
            elif topic == TOPIC_GUIDE_POSE:
                add_sample(topic_store, topic, stamp, [msg.x_d, msg.y_d, msg.z_d])
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR, TOPIC_ONLINE_OFFSET):
                add_sample(topic_store, topic, stamp, [msg.arm1_angle, msg.arm2_angle, msg.hand_angle])

    output = {}
    for topic, samples in topic_store.items():
        output[topic] = samples_to_arrays(samples)
    return output


def detect_arc_start_index(guide_pose, settle_time, time_array, min_consecutive=5):
    """检测主段圆弧真正开始的索引。

    先看 guide pose 在 x-z 平面相对初始静止点的偏移，
    再结合 settle_time，避免把稳定段误判为主段。
    """
    xz_ref = guide_pose[:, [0, 2]]
    head_count = min(max(10, min_consecutive + 1), xz_ref.shape[0])
    initial_center = np.median(xz_ref[:head_count, :], axis=0)
    distance = np.linalg.norm(xz_ref - initial_center, axis=1)
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
    """按统一起点裁剪多组时间序列。"""
    trimmed = []
    for array in arrays:
        if array is None:
            trimmed.append(None)
        else:
            trimmed.append(array[start_index:])
    return trimmed


def save_main_figure(output_dir, common_time, ee_actual, ee_target, base_actual, base_ref):
    """主图：同时展示末端定点效果、误差和基座圆弧跟踪。"""
    figure = plt.figure(figsize=(12, 11))
    grid = gridspec.GridSpec(3, 2, figure=figure, height_ratios=[1.15, 1.0, 1.0])

    axis_a_left = figure.add_subplot(grid[0, 0], projection="3d")
    axis_a_right = figure.add_subplot(grid[0, 1])
    axis_b = figure.add_subplot(grid[1, :])
    axis_c = figure.add_subplot(grid[2, :])

    ee_error = np.linalg.norm(ee_actual - ee_target, axis=1)
    ee_mean = float(np.mean(ee_error))
    ee_std = float(np.std(ee_error))

    axis_a_left.plot(ee_actual[:, 0], ee_actual[:, 1], ee_actual[:, 2], color="#1f77b4", linewidth=1.8)
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

    axis_b.plot(base_ref[:, 0], base_ref[:, 2], color="#d62728", linewidth=1.5, linestyle="--", label="Desired arc")
    axis_b.plot(base_actual[:, 0], base_actual[:, 2], color="#1f77b4", linewidth=1.8, label="Actual base")
    axis_b.set_title("(b) Base x-z arc tracking")
    axis_b.set_xlabel("x (m)")
    axis_b.set_ylabel("z (m)")
    axis_b.legend(loc="best", frameon=False)
    axis_b.set_aspect("equal", adjustable="box")
    style_axis(axis_b)

    axis_c.plot(common_time, ee_target[:, 0], color="#1f77b4", linewidth=1.0, linestyle=":", label="x desired")
    axis_c.plot(common_time, ee_actual[:, 0], color="#1f77b4", linewidth=1.8, label="x actual")
    axis_c.plot(common_time, ee_target[:, 1], color="#ff7f0e", linewidth=1.0, linestyle=":", label="y desired")
    axis_c.plot(common_time, ee_actual[:, 1], color="#ff7f0e", linewidth=1.8, label="y actual")
    axis_c.plot(common_time, ee_target[:, 2], color="#2ca02c", linewidth=1.0, linestyle=":", label="z desired")
    axis_c.plot(common_time, ee_actual[:, 2], color="#2ca02c", linewidth=1.8, label="z actual")
    axis_c.set_title("(c) EE compensation result in world frame")
    axis_c.set_xlabel("Time (s)")
    axis_c.set_ylabel("Position (m)")
    axis_c.legend(loc="upper right", ncol=3, frameon=False, fontsize=8)
    style_axis(axis_c)

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp2_main_figure.png"), dpi=300)
    plt.close(figure)


def save_end_effector_summary(output_dir, common_time, ee_actual, ee_target):
    """末端位置专题图：三轴对比 + 误差范数统计。"""
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
            ee_actual[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="actual %s" % axis_name,
        )
        axes[axis_index].set_ylabel("%s (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    _, ee_norm = compute_error(ee_actual, ee_target)
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
    """基座轨迹专题图：检查 UAV 在 x-z 圆弧上的跟踪质量。"""
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


def save_joint_summary(output_dir, common_time, arm_desired, arm_real, online_offset=None):
    """关节跟踪专题图：对照 arm1/arm2 的期望与实测，并可选显示在线标定偏置。"""
    row_count = 3 if online_offset is not None else 2
    figure, axes = plt.subplots(row_count, 1, figsize=(10, 9 if online_offset is not None else 7), sharex=True)
    joint_names = ["arm1", "arm2"]
    joint_colors = ["#1f77b4", "#ff7f0e"]

    for index, name in enumerate(joint_names):
        axes[index].plot(common_time, arm_desired[:, index], linestyle=":", linewidth=1.2,
                         color=joint_colors[index], label="desired %s" % name)
        axes[index].plot(common_time, arm_real[:, index], linewidth=1.8,
                         color=joint_colors[index], label="real %s" % name)
        axes[index].set_ylabel("deg")
        axes[index].legend(loc="best", frameon=False)
        style_axis(axes[index])
    if online_offset is not None:
        axes[2].plot(common_time, online_offset[:, 0], color="#1f77b4", linewidth=1.6, label="arm1 offset")
        axes[2].plot(common_time, online_offset[:, 1], color="#ff7f0e", linewidth=1.6, label="arm2 offset")
        axes[2].set_ylabel("offset (deg)")
        axes[2].legend(loc="best", frameon=False)
        style_axis(axes[2])
        axes[2].set_xlabel("Time (s)")
    else:
        axes[1].set_xlabel("Time (s)")

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp2_joint_summary.png"), dpi=300)
    plt.close(figure)


def save_online_offset_summary(output_dir, common_time, online_offset):
    """在线零位标定专题图：观察 arm1/arm2 偏置是否稳定。"""
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(common_time, online_offset[:, 0], color="#1f77b4", linewidth=1.8, label="arm1 offset")
    axes[0].set_ylabel("deg")
    axes[0].legend(loc="best", frameon=False)
    style_axis(axes[0])

    axes[1].plot(common_time, online_offset[:, 1], color="#ff7f0e", linewidth=1.8, label="arm2 offset")
    axes[1].set_ylabel("deg")
    axes[1].legend(loc="best", frameon=False)
    style_axis(axes[1])

    axes[2].plot(common_time, online_offset[:, 2], color="#2ca02c", linewidth=1.8, label="offset initialized flag")
    axes[2].set_ylabel("flag")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best", frameon=False)
    style_axis(axes[2])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp2_online_offset_summary.png"), dpi=300)
    plt.close(figure)


def main():
    """实验二绘图总流程。

    1. 读 bag 并标准化时间轴；
    2. 检测降落段并裁掉；
    3. 检测主段起点并裁掉 settle 段；
    4. 生成固定点目标；
    5. 输出主图、末端图、基座图、关节图和在线标定图。
    """
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    # 先把 bag 中各 topic 转成统一结构，再按用户给的时间窗口裁剪。
    bag_data = load_bag_data_exp2(args.bag)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [
        TOPIC_BASE_POSE,
        TOPIC_EE_POSE,
        TOPIC_GUIDE_POSE,
        TOPIC_ARM_REAL,
    ]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    # 建立统一时间轴，保证基座、末端、参考轨迹和关节数据可直接对齐。
    common_time = build_common_time([
        bag_data[TOPIC_BASE_POSE],
        bag_data[TOPIC_EE_POSE],
        bag_data[TOPIC_GUIDE_POSE],
        bag_data[TOPIC_ARM_REAL],
    ])

    base_pose = interp_series(bag_data[TOPIC_BASE_POSE], common_time)
    ee_pose = interp_series(bag_data[TOPIC_EE_POSE], common_time)
    guide_pose = interp_series(bag_data[TOPIC_GUIDE_POSE], common_time)
    arm_real = interp_series(bag_data[TOPIC_ARM_REAL], common_time)
    arm_desired = interp_series(bag_data[TOPIC_ARM_DESIRED], common_time) if bag_data.get(TOPIC_ARM_DESIRED) else arm_real
    online_offset = interp_series(bag_data[TOPIC_ONLINE_OFFSET], common_time) if bag_data.get(TOPIC_ONLINE_OFFSET) else None

    # 降落段不属于实验二主评估区间，先裁掉。
    landing_cut_index = detect_landing_cut_index(guide_pose)
    if 0 < landing_cut_index < common_time.shape[0]:
        common_time = common_time[:landing_cut_index]
        base_pose, ee_pose, guide_pose, arm_real, arm_desired = [
            array[:landing_cut_index] for array in (base_pose, ee_pose, guide_pose, arm_real, arm_desired)
        ]
        if online_offset is not None:
            online_offset = online_offset[:landing_cut_index]

    # 再找出主段圆弧真正开始的位置，去掉稳定段。
    start_index = detect_arc_start_index(guide_pose, args.settle_time, common_time)
    common_time, base_pose, ee_pose, guide_pose, arm_real, arm_desired, online_offset = trim_from_start(
        start_index, common_time, base_pose, ee_pose, guide_pose, arm_real, arm_desired, online_offset
    )
    common_time = common_time - common_time[0]

    if common_time.shape[0] < 5:
        raise RuntimeError("Experiment 2 valid segment is too short after trimming")

    # 固定点目标默认取主段起始时刻的末端位置；也支持手工指定。
    if args.use_initial_ee_hold:
        ee_hold_point = ee_pose[0, :].copy()
    else:
        ee_hold_point = np.array([args.ee_hold_x, args.ee_hold_y, args.ee_hold_z], dtype=float)
    ee_target = np.repeat(ee_hold_point.reshape(1, 3), common_time.shape[0], axis=0)

    save_main_figure(args.output_dir, common_time, ee_pose, ee_target, base_pose, guide_pose)
    save_end_effector_summary(args.output_dir, common_time, ee_pose, ee_target)
    save_base_summary(args.output_dir, common_time, base_pose, guide_pose)
    save_joint_summary(args.output_dir, common_time, arm_desired, arm_real, online_offset)
    if online_offset is not None:
        save_online_offset_summary(args.output_dir, common_time, online_offset)

    print("Saved experiment 2 figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
