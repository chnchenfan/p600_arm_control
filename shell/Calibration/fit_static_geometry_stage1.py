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
# - base_xyz_target_xyz_dq_l1vec_l2：扩展模型，拟合
#   [arm_target_offset_x/y/z, arm1_zero_offset, arm2_zero_offset, l1_vec_x/y/z, L2_m]
#   注意：静态位置数据无法同时区分 arm_base_offset 和 l1_vec，所以这个模式固定 arm_base_offset=0，
#   直接拟合 arm_base 刚体原点到第二关节圆心的三维向量。
# - base_xyz_target_xyz_dq_l1vec_l2_reg：在扩展模型基础上增加软正则，
#   继续拟合 L2，但通过“dq 尽量小 / L2 尽量靠近当前量测 / arm_target_offset 不要过大”
#   这些先验抑制错误参数之间的相互代偿。
# - base_xyz_target_xyz_dq_l1vec_l2_reg_rpy：在正则扩展模型基础上继续放开
#   arm_base 刚体到机械臂模型坐标系的固定旋转外参 roll/pitch/yaw。
# - base_xyz_target_xyz_dq_l1vec_l2_reg_rpy_axes：在 reg_rpy 基础上继续放开
#   两个关节轴的小倾斜，用来描述维修后关节轴轻微非正交/安装偏斜。
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
FIT_MODE_EXTENDED = "base_xyz_target_xyz_dq_l1vec_l2"
FIT_MODE_EXTENDED_REG = "base_xyz_target_xyz_dq_l1vec_l2_reg"
FIT_MODE_EXTENDED_REG_RPY = "base_xyz_target_xyz_dq_l1vec_l2_reg_rpy"
FIT_MODE_EXTENDED_REG_RPY_AXES = "base_xyz_target_xyz_dq_l1vec_l2_reg_rpy_axes"


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
    target_quaternion: np.ndarray
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
        choices=[
            FIT_MODE_FULL,
            FIT_MODE_DX_ONLY,
            FIT_MODE_DX_L2,
            FIT_MODE_FULL_L2,
            FIT_MODE_FULL_L1_L2,
            FIT_MODE_EXTENDED,
            FIT_MODE_EXTENDED_REG,
            FIT_MODE_EXTENDED_REG_RPY,
            FIT_MODE_EXTENDED_REG_RPY_AXES,
        ],
        help="Parameter subset to fit",
    )
    parser.add_argument("--dq-bound-deg", type=float, default=8.0, help="Bound for dq1/dq2 in regularized extended mode")
    parser.add_argument("--dq-prior-sigma-deg", type=float, default=3.0, help="Soft prior sigma for dq1/dq2 in regularized extended mode")
    parser.add_argument("--l2-prior-sigma-m", type=float, default=0.03, help="Soft prior sigma for L2 in regularized extended mode")
    parser.add_argument("--target-offset-prior-sigma-m", type=float, default=0.04, help="Soft prior sigma for arm_target offset in regularized extended mode")
    parser.add_argument("--l1-xy-prior-sigma-m", type=float, default=0.03, help="Soft prior sigma for l1_vec x/y in regularized extended mode")
    parser.add_argument("--base-rpy-bound-deg", type=float, default=12.0, help="Bound for base fixed roll/pitch/yaw extrinsic in rpy mode")
    parser.add_argument("--base-rpy-prior-sigma-deg", type=float, default=4.0, help="Soft prior sigma for base fixed roll/pitch/yaw extrinsic in rpy mode")
    parser.add_argument("--joint-axis-bound-deg", type=float, default=8.0, help="Bound for small joint-axis tilt angles in axes mode")
    parser.add_argument("--joint-axis-prior-sigma-deg", type=float, default=3.0, help="Soft prior sigma for joint-axis tilt angles in axes mode")
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


def quaternion_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return normalize_quaternion(
        np.array(
            [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
            dtype=float,
        )
    )


def rotation_matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return normalize_quaternion(np.array([
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
            0.25 * s,
        ], dtype=float))
    if rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        return normalize_quaternion(np.array([
            0.25 * s,
            (rot[0, 1] + rot[1, 0]) / s,
            (rot[0, 2] + rot[2, 0]) / s,
            (rot[2, 1] - rot[1, 2]) / s,
        ], dtype=float))
    if rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        return normalize_quaternion(np.array([
            (rot[0, 1] + rot[1, 0]) / s,
            0.25 * s,
            (rot[1, 2] + rot[2, 1]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
        ], dtype=float))
    s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
    return normalize_quaternion(np.array([
        (rot[0, 2] + rot[2, 0]) / s,
        (rot[1, 2] + rot[2, 1]) / s,
        0.25 * s,
        (rot[1, 0] - rot[0, 1]) / s,
    ], dtype=float))


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
        target_positions = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in target_msgs], dtype=float)
        target_quaternions = [
            np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w], dtype=float)
            for _, m in target_msgs
        ]
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
                target_quaternion=average_quaternions(target_quaternions),
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


def default_l1_vector(l1_m: float) -> np.ndarray:
    # 旧模型里 L1 只是一段纯 z 偏移，这里保留它作为默认几何假设。
    return np.array([0.0, 0.0, -l1_m], dtype=float)


def l1_scalar_from_vector(l1_vector: np.ndarray) -> float:
    return float(np.linalg.norm(l1_vector))


def arm_tip_rotation_matrix(arm1_deg: float, arm2_deg: float, dq1_deg: float, dq2_deg: float) -> np.ndarray:
    # 一个与当前 FK 一致的二关节姿态近似：
    # 先绕 z 轴转 -q1，再绕 y 轴转 q2，使 link-2 的 x 轴沿末端指向。
    q1 = math.radians(arm1_deg + dq1_deg)
    q2 = math.radians(arm2_deg + dq2_deg)
    cz = math.cos(q1)
    sz = math.sin(q1)
    cy = math.cos(q2)
    sy = math.sin(q2)
    rot_z = np.array([[cz, sz, 0.0], [-sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=float)
    return rot_z.dot(rot_y)


def rotation_matrix_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3, dtype=float)
    axis = axis / norm
    x, y, z = axis.tolist()
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    v = 1.0 - c
    return np.array(
        [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ],
        dtype=float,
    )


def joint_axes_from_theta(theta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # joint1 名义轴是 +z；joint2 名义轴是 +y。
    # 这里只允许小角度倾斜，用来表达维修后的轻微非正交/装配误差。
    j1_tilt_x_deg = float(theta[14])
    j1_tilt_y_deg = float(theta[15])
    j2_tilt_x_deg = float(theta[16])
    j2_tilt_z_deg = float(theta[17])
    joint1_axis = quaternion_to_rotation_matrix(
        quaternion_from_rpy(math.radians(j1_tilt_x_deg), math.radians(j1_tilt_y_deg), 0.0)
    ).dot(np.array([0.0, 0.0, 1.0], dtype=float))
    joint2_axis_local = quaternion_to_rotation_matrix(
        quaternion_from_rpy(math.radians(j2_tilt_x_deg), 0.0, math.radians(j2_tilt_z_deg))
    ).dot(np.array([0.0, 1.0, 0.0], dtype=float))
    return joint1_axis, joint2_axis_local


def forward_kinematics_from_theta(theta: np.ndarray, l2_m: float, arm1_deg: float, arm2_deg: float) -> Tuple[np.ndarray, np.ndarray]:
    dq1_deg = float(theta[6])
    dq2_deg = float(theta[7])
    q1_rad = math.radians(arm1_deg + dq1_deg)
    q2_rad = math.radians(arm2_deg + dq2_deg)
    joint1_axis, joint2_axis_local = joint_axes_from_theta(theta)
    rot1 = rotation_matrix_from_axis_angle(joint1_axis, -q1_rad)
    rot2_local = rotation_matrix_from_axis_angle(joint2_axis_local, q2_rad)
    tip_rot_model = rot1.dot(rot2_local)
    tip_vector_model = tip_rot_model.dot(np.array([l2_m, 0.0, 0.0], dtype=float))
    return tip_vector_model, tip_rot_model


def expand_theta(free_theta: np.ndarray, fit_mode: str, l1_m: float) -> np.ndarray:
    # 把“当前拟合模式下的自由参数”映射回统一的 18 维模型参数：
    # [base_dx, base_dy, base_dz, target_dx, target_dy, target_dz, dq1, dq2,
    #  l1_vec_x, l1_vec_y, l1_vec_z, base_roll_deg, base_pitch_deg, base_yaw_deg,
    #  joint1_tilt_x_deg, joint1_tilt_y_deg, joint2_tilt_x_deg, joint2_tilt_z_deg]
    #
    # 旧模式下保持 arm_target_offset=0、L1 向量=[0,0,-L1]；
    # 扩展模式下固定 arm_base_offset=0，只放开 arm_target_offset 和 L1 三维向量。
    theta = np.zeros(18, dtype=float)
    theta[8:11] = default_l1_vector(l1_m)
    if fit_mode == FIT_MODE_DX_ONLY:
        theta[0] = float(free_theta[0])
    elif fit_mode == FIT_MODE_DX_L2:
        theta[0] = float(free_theta[0])
    elif fit_mode == FIT_MODE_FULL:
        theta[0:3] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
    elif fit_mode == FIT_MODE_FULL_L2:
        theta[0:3] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        theta[0:3] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
        theta[10] = -float(free_theta[5])
    elif fit_mode in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG):
        theta[3:6] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
        theta[8:11] = np.array(free_theta[5:8], dtype=float)
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY:
        theta[3:6] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
        theta[8:11] = np.array(free_theta[5:8], dtype=float)
        theta[11:14] = np.array(free_theta[8:11], dtype=float)
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY_AXES:
        theta[3:6] = np.array(free_theta[0:3], dtype=float)
        theta[6:8] = np.array(free_theta[3:5], dtype=float)
        theta[8:11] = np.array(free_theta[5:8], dtype=float)
        theta[11:14] = np.array(free_theta[8:11], dtype=float)
        theta[14:18] = np.array(free_theta[11:15], dtype=float)
    else:
        raise ValueError("unsupported fit_mode: %s" % fit_mode)
    return theta


def residual_vector_from_full_theta(theta: np.ndarray, samples: Sequence[SampleMean], l2_m: float) -> np.ndarray:
    base_offset_body = theta[0:3]
    target_offset_target = theta[3:6]
    dq1_deg = float(theta[6])
    dq2_deg = float(theta[7])
    l1_vector_body = theta[8:11]
    base_roll_deg = float(theta[11])
    base_pitch_deg = float(theta[12])
    base_yaw_deg = float(theta[13])
    rot_bm = quaternion_to_rotation_matrix(
        quaternion_from_rpy(math.radians(base_roll_deg), math.radians(base_pitch_deg), math.radians(base_yaw_deg))
    )
    residuals: List[float] = []
    for sample in samples:
        rot_wb = quaternion_to_rotation_matrix(sample.base_quaternion)
        rot_wm = rot_wb.dot(rot_bm)
        p_b_world = sample.base_position - rot_wb.dot(base_offset_body)
        # 扩展模型下允许两类额外几何补偿：
        # 1. arm_target_offset_target：arm_target 刚体参考点到真实末端点的常值偏移，
        #    这个偏移定义在 arm_target 自身坐标系里，会随 target 姿态一起旋转；
        # 2. l1_vector_body：从 arm_base 参考原点指向第二关节圆心的三维向量。
        #
        # 旧模型是它的特例：target_offset=0，l1_vector=[0,0,-L1]。
        rot_wt = quaternion_to_rotation_matrix(sample.target_quaternion)
        true_target_world = sample.target_position + rot_wt.dot(target_offset_target)
        relative_world = true_target_world - p_b_world
        relative_model = rot_wm.T.dot(relative_world)
        p_rel_meas = relative_model - l1_vector_body
        p_rel_fk, _ = forward_kinematics_from_theta(theta, l2_m, sample.arm1_deg, sample.arm2_deg)
        residuals.extend((p_rel_meas - p_rel_fk).tolist())
    return np.array(residuals, dtype=float)


def build_regularization_residuals(theta: np.ndarray, l2_m: float, fit_options: Dict[str, float]) -> np.ndarray:
    # 软正则不是拿来“拍脑袋修正结果”的，而是把你已经知道的机械事实写进优化问题：
    # 1. dq 零偏应该很小；
    # 2. L2 虽然不确定，但不该离当前量测值太远；
    # 3. arm_target 刚体偏移不应该无限大；
    # 4. l1_vec 的 x/y 通常只允许是小修正，主分量仍应以 z 为主。
    dq1_deg = float(theta[6])
    dq2_deg = float(theta[7])
    target_offset = theta[3:6]
    l1_vector = theta[8:11]
    base_rpy_deg = theta[11:14]
    joint_axis_tilt_deg = theta[14:18]
    return np.array(
        [
            dq1_deg / fit_options["dq_prior_sigma_deg"],
            dq2_deg / fit_options["dq_prior_sigma_deg"],
            (l2_m - fit_options["l2_prior_m"]) / fit_options["l2_prior_sigma_m"],
            target_offset[0] / fit_options["target_offset_prior_sigma_m"],
            target_offset[1] / fit_options["target_offset_prior_sigma_m"],
            target_offset[2] / fit_options["target_offset_prior_sigma_m"],
            l1_vector[0] / fit_options["l1_xy_prior_sigma_m"],
            l1_vector[1] / fit_options["l1_xy_prior_sigma_m"],
            base_rpy_deg[0] / fit_options["base_rpy_prior_sigma_deg"],
            base_rpy_deg[1] / fit_options["base_rpy_prior_sigma_deg"],
            base_rpy_deg[2] / fit_options["base_rpy_prior_sigma_deg"],
            joint_axis_tilt_deg[0] / fit_options["joint_axis_prior_sigma_deg"],
            joint_axis_tilt_deg[1] / fit_options["joint_axis_prior_sigma_deg"],
            joint_axis_tilt_deg[2] / fit_options["joint_axis_prior_sigma_deg"],
            joint_axis_tilt_deg[3] / fit_options["joint_axis_prior_sigma_deg"],
        ],
        dtype=float,
    )


def compute_residual_vector(
    free_theta: np.ndarray,
    samples: Sequence[SampleMean],
    l1_m: float,
    l2_m: float,
    fit_mode: str,
    fit_options: Dict[str, float] = None,
) -> np.ndarray:
    # 构造拟合器使用的残差。不同模式下只放开不同子集的参数。
    theta = expand_theta(free_theta, fit_mode, l1_m)
    effective_l2 = l2_m
    if fit_mode == FIT_MODE_DX_L2:
        effective_l2 = float(free_theta[1])
    elif fit_mode == FIT_MODE_FULL_L2:
        effective_l2 = float(free_theta[5])
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        effective_l1 = float(free_theta[5])
        theta[10] = -effective_l1
        effective_l2 = float(free_theta[6])
    elif fit_mode == FIT_MODE_EXTENDED:
        effective_l2 = float(free_theta[8])
    elif fit_mode == FIT_MODE_EXTENDED_REG:
        effective_l2 = float(free_theta[8])
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY:
        effective_l2 = float(free_theta[11])
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY_AXES:
        effective_l2 = float(free_theta[11])
    residual = residual_vector_from_full_theta(theta, samples, effective_l2)
    if fit_mode in (FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        if fit_options is None:
            raise ValueError("fit_options is required for regularized extended mode")
        residual = np.concatenate([residual, build_regularization_residuals(theta, effective_l2, fit_options)])
    return residual


def residual_norms(full_theta: np.ndarray, samples: Sequence[SampleMean], l2_m: float) -> List[float]:
    vec = residual_vector_from_full_theta(full_theta, samples, l2_m)
    reshaped = vec.reshape((-1, 3))
    return [float(np.linalg.norm(row)) for row in reshaped]


def rms_from_residual_vector(residual_vector: np.ndarray) -> float:
    if residual_vector.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(residual_vector))))


def fit_stage1(samples: Sequence[SampleMean], l1_m: float, l2_m: float, fit_mode: str, fit_options: Dict[str, float] = None):
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
    elif fit_mode == FIT_MODE_EXTENDED:
        # 扩展模型用于处理“维修后几何定义发生变化”的情况：
        # - arm_target 刚体参考点可能不再落在真正的末端点上；
        # - “L1”不再假设是纯 z 方向，而是直接拟合 arm_base 刚体原点到第二关节圆心的三维向量；
        # - arm_base_offset 与 l1_vec 在静态位置数据里不可分，所以这里固定 arm_base_offset=0。
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -l1_m, l2_m], dtype=float)
        lower = np.array(
            [
                -0.12, -0.12, -0.12,
                -20.0, -20.0,
                -0.12, -0.12, -(l1_m + 0.12),
                max(0.10, l2_m - 0.12),
            ],
            dtype=float,
        )
        upper = np.array(
            [
                0.12, 0.12, 0.12,
                20.0, 20.0,
                0.12, 0.12, -max(0.02, l1_m - 0.12),
                l2_m + 0.12,
            ],
            dtype=float,
        )
    elif fit_mode == FIT_MODE_EXTENDED_REG:
        if fit_options is None:
            raise ValueError("fit_options is required for regularized extended mode")
        # 这一版保留扩展几何模型，但把“现场已知事实”作为软约束写进优化问题：
        # - dq 零偏应该很小；
        # - L2 不固定，但应围绕当前粗量测值收敛；
        # - arm_target 刚体偏移和 l1_vec 的横向分量不应无界增大。
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -l1_m, l2_m], dtype=float)
        dq_bound_deg = fit_options["dq_bound_deg"]
        lower = np.array(
            [
                -0.10, -0.10, -0.10,
                -dq_bound_deg, -dq_bound_deg,
                -0.10, -0.10, -(l1_m + 0.12),
                max(0.10, l2_m - 0.10),
            ],
            dtype=float,
        )
        upper = np.array(
            [
                0.10, 0.10, 0.10,
                dq_bound_deg, dq_bound_deg,
                0.10, 0.10, -max(0.02, l1_m - 0.12),
                l2_m + 0.10,
            ],
            dtype=float,
        )
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY:
        if fit_options is None:
            raise ValueError("fit_options is required for regularized extended rpy mode")
        # 在正则扩展模型的基础上继续放开 arm_base 固定旋转外参。
        # 这用来解释“arm_base 刚体安装姿态”和机械臂模型坐标系之间的固定夹角。
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -l1_m, l2_m, 0.0, 0.0, 0.0], dtype=float)
        dq_bound_deg = fit_options["dq_bound_deg"]
        base_rpy_bound_deg = fit_options["base_rpy_bound_deg"]
        lower = np.array(
            [
                -0.10, -0.10, -0.10,
                -dq_bound_deg, -dq_bound_deg,
                -0.10, -0.10, -(l1_m + 0.12),
                max(0.10, l2_m - 0.10),
                -base_rpy_bound_deg, -base_rpy_bound_deg, -base_rpy_bound_deg,
            ],
            dtype=float,
        )
        upper = np.array(
            [
                0.10, 0.10, 0.10,
                dq_bound_deg, dq_bound_deg,
                0.10, 0.10, -max(0.02, l1_m - 0.12),
                l2_m + 0.10,
                base_rpy_bound_deg, base_rpy_bound_deg, base_rpy_bound_deg,
            ],
            dtype=float,
        )
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY_AXES:
        if fit_options is None:
            raise ValueError("fit_options is required for regularized extended rpy+axes mode")
        # 这一版在 reg_rpy 的基础上继续放开两个关节轴的小倾斜角。
        # 只允许小范围拟合，并通过正则强约束，避免优化器把一切误差都推给轴倾斜。
        initial_free = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -l1_m, l2_m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        dq_bound_deg = fit_options["dq_bound_deg"]
        base_rpy_bound_deg = fit_options["base_rpy_bound_deg"]
        joint_axis_bound_deg = fit_options["joint_axis_bound_deg"]
        lower = np.array(
            [
                -0.10, -0.10, -0.10,
                -dq_bound_deg, -dq_bound_deg,
                -0.10, -0.10, -(l1_m + 0.12),
                max(0.10, l2_m - 0.10),
                -base_rpy_bound_deg, -base_rpy_bound_deg, -base_rpy_bound_deg,
                -joint_axis_bound_deg, -joint_axis_bound_deg, -joint_axis_bound_deg, -joint_axis_bound_deg,
            ],
            dtype=float,
        )
        upper = np.array(
            [
                0.10, 0.10, 0.10,
                dq_bound_deg, dq_bound_deg,
                0.10, 0.10, -max(0.02, l1_m - 0.12),
                l2_m + 0.10,
                base_rpy_bound_deg, base_rpy_bound_deg, base_rpy_bound_deg,
                joint_axis_bound_deg, joint_axis_bound_deg, joint_axis_bound_deg, joint_axis_bound_deg,
            ],
            dtype=float,
        )
    else:
        raise ValueError("unsupported fit_mode: %s" % fit_mode)

    result = least_squares(
        compute_residual_vector,
        x0=initial_free,
        bounds=(lower, upper),
        args=(samples, l1_m, l2_m, fit_mode, fit_options),
    )
    initial_full = expand_theta(initial_free, fit_mode, l1_m)
    fitted_full = expand_theta(np.array(result.x, dtype=float), fit_mode, l1_m)
    if fit_mode == FIT_MODE_DX_L2:
        fitted_l2 = float(result.x[1])
        initial_l2 = float(initial_free[1])
    elif fit_mode == FIT_MODE_FULL_L2:
        fitted_l2 = float(result.x[5])
        initial_l2 = float(initial_free[5])
    elif fit_mode == FIT_MODE_FULL_L1_L2:
        fitted_l2 = float(result.x[6])
        initial_l2 = float(initial_free[6])
    elif fit_mode in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG):
        fitted_l2 = float(result.x[8])
        initial_l2 = float(initial_free[8])
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY:
        fitted_l2 = float(result.x[8])
        initial_l2 = float(initial_free[8])
    elif fit_mode == FIT_MODE_EXTENDED_REG_RPY_AXES:
        fitted_l2 = float(result.x[8])
        initial_l2 = float(initial_free[8])
    else:
        fitted_l2 = float(l2_m)
        initial_l2 = float(l2_m)
    if fit_mode == FIT_MODE_FULL_L1_L2:
        fitted_l1 = float(result.x[5])
        initial_l1 = float(initial_free[5])
    elif fit_mode in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        fitted_l1 = l1_scalar_from_vector(fitted_full[8:11])
        initial_l1 = l1_scalar_from_vector(initial_full[8:11])
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
        "target_qx",
        "target_qy",
        "target_qz",
        "target_qw",
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
                    "target_qx": sample.target_quaternion[0],
                    "target_qy": sample.target_quaternion[1],
                    "target_qz": sample.target_quaternion[2],
                    "target_qw": sample.target_quaternion[3],
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
    initial_residual_vector = residual_vector_from_full_theta(initial_theta, samples, initial_l2_m)
    fitted_residual_vector = residual_vector_from_full_theta(fitted_theta, samples, fitted_l2_m)
    return {
        "fit_mode": fit_mode,
        "initial_l1_m": initial_l1_m,
        "fitted_l1_m": fitted_l1_m,
        "initial_l1_vector_m": {
            "x": float(initial_theta[8]),
            "y": float(initial_theta[9]),
            "z": float(initial_theta[10]),
        },
        "fitted_l1_vector_m": {
            "x": float(fitted_theta[8]),
            "y": float(fitted_theta[9]),
            "z": float(fitted_theta[10]),
        },
        "initial_l2_m": initial_l2_m,
        "fitted_l2_m": fitted_l2_m,
        "sample_count": len(samples),
        "initial_theta": {
            "arm_base_offset_x_m": float(initial_theta[0]),
            "arm_base_offset_y_m": float(initial_theta[1]),
            "arm_base_offset_z_m": float(initial_theta[2]),
            "arm_target_offset_x_m": float(initial_theta[3]),
            "arm_target_offset_y_m": float(initial_theta[4]),
            "arm_target_offset_z_m": float(initial_theta[5]),
            "arm1_zero_offset_deg": float(initial_theta[6]),
            "arm2_zero_offset_deg": float(initial_theta[7]),
            "arm_base_rot_roll_deg": float(initial_theta[11]),
            "arm_base_rot_pitch_deg": float(initial_theta[12]),
            "arm_base_rot_yaw_deg": float(initial_theta[13]),
            "joint1_axis_tilt_x_deg": float(initial_theta[14]),
            "joint1_axis_tilt_y_deg": float(initial_theta[15]),
            "joint2_axis_tilt_x_deg": float(initial_theta[16]),
            "joint2_axis_tilt_z_deg": float(initial_theta[17]),
        },
        "fitted_theta": {
            "arm_base_offset_x_m": float(fitted_theta[0]),
            "arm_base_offset_y_m": float(fitted_theta[1]),
            "arm_base_offset_z_m": float(fitted_theta[2]),
            "arm_target_offset_x_m": float(fitted_theta[3]),
            "arm_target_offset_y_m": float(fitted_theta[4]),
            "arm_target_offset_z_m": float(fitted_theta[5]),
            "arm1_zero_offset_deg": float(fitted_theta[6]),
            "arm2_zero_offset_deg": float(fitted_theta[7]),
            "arm_base_rot_roll_deg": float(fitted_theta[11]),
            "arm_base_rot_pitch_deg": float(fitted_theta[12]),
            "arm_base_rot_yaw_deg": float(fitted_theta[13]),
            "joint1_axis_tilt_x_deg": float(fitted_theta[14]),
            "joint1_axis_tilt_y_deg": float(fitted_theta[15]),
            "joint2_axis_tilt_x_deg": float(fitted_theta[16]),
            "joint2_axis_tilt_z_deg": float(fitted_theta[17]),
        },
        "initial_rms_m": rms_from_residual_vector(initial_residual_vector),
        "fitted_rms_m": rms_from_residual_vector(fitted_residual_vector),
        "per_sample_residual_norm_m": residual_norms(fitted_theta, samples, fitted_l2_m),
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
    print("<!-- arm_target_offset_* is expressed in arm_target rigid-body frame -->")
    print('<param name="arm_target_offset_x_m" value="%.6f" />' % theta["arm_target_offset_x_m"])
    print('<param name="arm_target_offset_y_m" value="%.6f" />' % theta["arm_target_offset_y_m"])
    print('<param name="arm_target_offset_z_m" value="%.6f" />' % theta["arm_target_offset_z_m"])
    print('<param name="arm1_zero_offset_deg" value="%.6f" />' % theta["arm1_zero_offset_deg"])
    print('<param name="arm2_zero_offset_deg" value="%.6f" />' % theta["arm2_zero_offset_deg"])
    if result_dict.get("fit_mode") == FIT_MODE_EXTENDED_REG_RPY:
        print('<param name="arm_base_rot_roll_deg" value="%.6f" />' % theta["arm_base_rot_roll_deg"])
        print('<param name="arm_base_rot_pitch_deg" value="%.6f" />' % theta["arm_base_rot_pitch_deg"])
        print('<param name="arm_base_rot_yaw_deg" value="%.6f" />' % theta["arm_base_rot_yaw_deg"])
    if result_dict.get("fit_mode") == FIT_MODE_EXTENDED_REG_RPY_AXES:
        print('<param name="arm_base_rot_roll_deg" value="%.6f" />' % theta["arm_base_rot_roll_deg"])
        print('<param name="arm_base_rot_pitch_deg" value="%.6f" />' % theta["arm_base_rot_pitch_deg"])
        print('<param name="arm_base_rot_yaw_deg" value="%.6f" />' % theta["arm_base_rot_yaw_deg"])
        print('<param name="joint1_axis_tilt_x_deg" value="%.6f" />' % theta["joint1_axis_tilt_x_deg"])
        print('<param name="joint1_axis_tilt_y_deg" value="%.6f" />' % theta["joint1_axis_tilt_y_deg"])
        print('<param name="joint2_axis_tilt_x_deg" value="%.6f" />' % theta["joint2_axis_tilt_x_deg"])
        print('<param name="joint2_axis_tilt_z_deg" value="%.6f" />' % theta["joint2_axis_tilt_z_deg"])
    if result_dict.get("fit_mode") == FIT_MODE_FULL_L1_L2:
        print('<param name="L1_m" value="%.6f" />' % result_dict['fitted_l1_m'])
    if result_dict.get("fit_mode") in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print('<param name="l1_vec_x_m" value="%.6f" />' % result_dict['fitted_l1_vector_m']['x'])
        print('<param name="l1_vec_y_m" value="%.6f" />' % result_dict['fitted_l1_vector_m']['y'])
        print('<param name="l1_vec_z_m" value="%.6f" />' % result_dict['fitted_l1_vector_m']['z'])
    if result_dict.get("fit_mode") in (FIT_MODE_DX_L2, FIT_MODE_FULL_L2, FIT_MODE_FULL_L1_L2):
        print('<param name="L2_m" value="%.6f" />' % result_dict['fitted_l2_m'])
    if result_dict.get("fit_mode") in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print('<param name="L2_m" value="%.6f" />' % result_dict['fitted_l2_m'])


def run_synthetic_test() -> int:
    true_theta = np.array([0.031, -0.006, 0.012, 0.0, 0.0, 0.0, 2.0, -3.0, 0.0, 0.0, -0.161], dtype=float)
    l1_m = 0.161
    l2_m = 0.262
    arm_angles = [(-60.0, 0.0), (-30.0, 0.0), (0.0, 0.0), (30.0, 15.0), (15.0, 30.0), (45.0, 15.0)]
    base_rpys_deg = [(0.0, 0.0, 0.0), (2.0, -3.0, 5.0), (-4.0, 3.0, 10.0), (5.0, 2.0, -8.0), (-3.0, -4.0, 12.0), (4.0, -2.0, -6.0)]
    samples: List[SampleMean] = []
    for idx, ((arm1_deg, arm2_deg), (roll_deg, pitch_deg, yaw_deg)) in enumerate(zip(arm_angles, base_rpys_deg)):
        q = quaternion_from_rpy(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        rot = quaternion_to_rotation_matrix(q)
        base_position = np.array([0.2 + 0.01 * idx, -0.1 + 0.005 * idx, 0.9], dtype=float)
        p_rel_fk = forward_kinematics(l2_m, arm1_deg, arm2_deg, true_theta[6], true_theta[7])
        tip_rot_body = arm_tip_rotation_matrix(arm1_deg, arm2_deg, true_theta[6], true_theta[7])
        target_rot_world = rot.dot(tip_rot_body)
        relative_body = p_rel_fk - tip_rot_body.dot(true_theta[3:6]) + true_theta[8:11]
        p_b_world = base_position - rot.dot(true_theta[0:3])
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
                target_quaternion=rotation_matrix_to_quaternion(target_rot_world),
                arm1_deg=arm1_deg,
                arm2_deg=arm2_deg,
                arm1_std_deg=0.0,
                arm2_std_deg=0.0,
            )
        )

    result_basic, _, fitted_basic, _, _, _, _ = fit_stage1(samples, l1_m, l2_m, FIT_MODE_FULL)
    basic_error = np.abs(fitted_basic[[0, 1, 2, 6, 7]] - true_theta[[0, 1, 2, 6, 7]])
    print("synthetic_basic_true_theta=", true_theta[[0, 1, 2, 6, 7]].tolist())
    print("synthetic_basic_fitted_theta=", fitted_basic[[0, 1, 2, 6, 7]].tolist())
    print("synthetic_basic_abs_error=", basic_error.tolist())
    if not np.all(np.isfinite(fitted_basic)):
        raise SystemExit("synthetic basic smoke test failed")

    extended_theta = np.array([0.0, 0.0, 0.0, 0.012, -0.007, 0.018, 1.5, -2.5, 0.011, -0.006, -0.229], dtype=float)
    extended_l2_m = 0.281
    extended_samples: List[SampleMean] = []
    for idx, ((arm1_deg, arm2_deg), (roll_deg, pitch_deg, yaw_deg)) in enumerate(zip(arm_angles, base_rpys_deg)):
        q = quaternion_from_rpy(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        rot = quaternion_to_rotation_matrix(q)
        base_position = np.array([0.1 + 0.01 * idx, 0.05 - 0.004 * idx, 1.0], dtype=float)
        p_rel_fk = forward_kinematics(extended_l2_m, arm1_deg, arm2_deg, extended_theta[6], extended_theta[7])
        tip_rot_body = arm_tip_rotation_matrix(arm1_deg, arm2_deg, extended_theta[6], extended_theta[7])
        target_rot_world = rot.dot(tip_rot_body)
        relative_body = p_rel_fk - tip_rot_body.dot(extended_theta[3:6]) + extended_theta[8:11]
        p_b_world = base_position
        target_position = p_b_world + rot.dot(relative_body)
        extended_samples.append(
            SampleMean(
                sample_index=idx,
                start_sec=float(idx),
                end_sec=float(idx) + 2.5,
                duration_sec=2.5,
                base_position=base_position,
                base_quaternion=q,
                target_position=target_position,
                target_quaternion=rotation_matrix_to_quaternion(target_rot_world),
                arm1_deg=arm1_deg,
                arm2_deg=arm2_deg,
                arm1_std_deg=0.0,
                arm2_std_deg=0.0,
            )
        )

    result_extended, _, fitted_extended, _, _, _, fitted_extended_l2 = fit_stage1(
        extended_samples,
        l1_m=0.23,
        l2_m=0.28,
        fit_mode=FIT_MODE_EXTENDED,
    )
    extended_error = np.abs(fitted_extended[0:11] - extended_theta)
    print("synthetic_extended_true_theta=", extended_theta.tolist())
    print("synthetic_extended_fitted_theta=", fitted_extended[0:11].tolist())
    print("synthetic_extended_abs_error=", extended_error.tolist())
    print("synthetic_extended_fitted_l2=", fitted_extended_l2)
    if not np.all(np.isfinite(fitted_extended)):
        raise SystemExit("synthetic extended smoke test failed")
    if not math.isfinite(fitted_extended_l2):
        raise SystemExit("synthetic extended l2 smoke test failed")

    extended_rpy_theta = np.array([0.0, 0.0, 0.0, 0.012, -0.007, 0.018, 1.5, -2.5, 0.011, -0.006, -0.229, 3.0, -2.0, 5.0], dtype=float)
    extended_rpy_l2_m = 0.281
    extended_rpy_samples: List[SampleMean] = []
    for idx, ((arm1_deg, arm2_deg), (roll_deg, pitch_deg, yaw_deg)) in enumerate(zip(arm_angles, base_rpys_deg)):
        q = quaternion_from_rpy(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        rot_wb = quaternion_to_rotation_matrix(q)
        rot_bm = quaternion_to_rotation_matrix(
            quaternion_from_rpy(
                math.radians(extended_rpy_theta[11]),
                math.radians(extended_rpy_theta[12]),
                math.radians(extended_rpy_theta[13]),
            )
        )
        rot_wm = rot_wb.dot(rot_bm)
        base_position = np.array([0.12 + 0.01 * idx, -0.02 - 0.004 * idx, 1.05], dtype=float)
        p_rel_fk = forward_kinematics(extended_rpy_l2_m, arm1_deg, arm2_deg, extended_rpy_theta[6], extended_rpy_theta[7])
        tip_rot_body = arm_tip_rotation_matrix(arm1_deg, arm2_deg, extended_rpy_theta[6], extended_rpy_theta[7])
        target_rot_world = rot_wm.dot(tip_rot_body)
        relative_model = p_rel_fk - tip_rot_body.dot(extended_rpy_theta[3:6]) + extended_rpy_theta[8:11]
        target_position = base_position + rot_wm.dot(relative_model)
        extended_rpy_samples.append(
            SampleMean(
                sample_index=idx,
                start_sec=float(idx),
                end_sec=float(idx) + 2.5,
                duration_sec=2.5,
                base_position=base_position,
                base_quaternion=q,
                target_position=target_position,
                target_quaternion=rotation_matrix_to_quaternion(target_rot_world),
                arm1_deg=arm1_deg,
                arm2_deg=arm2_deg,
                arm1_std_deg=0.0,
                arm2_std_deg=0.0,
            )
        )
    reg_rpy_options = {
        "dq_bound_deg": 8.0,
        "dq_prior_sigma_deg": 3.0,
        "l2_prior_m": 0.28,
        "l2_prior_sigma_m": 0.03,
        "target_offset_prior_sigma_m": 0.04,
        "l1_xy_prior_sigma_m": 0.03,
        "base_rpy_bound_deg": 12.0,
        "base_rpy_prior_sigma_deg": 4.0,
        "joint_axis_bound_deg": 8.0,
        "joint_axis_prior_sigma_deg": 3.0,
    }
    _, _, fitted_rpy, _, _, _, fitted_rpy_l2 = fit_stage1(
        extended_rpy_samples,
        l1_m=0.23,
        l2_m=0.28,
        fit_mode=FIT_MODE_EXTENDED_REG_RPY,
        fit_options=reg_rpy_options,
    )
    rpy_error = np.abs(fitted_rpy - extended_rpy_theta)
    print("synthetic_rpy_true_theta=", extended_rpy_theta.tolist())
    print("synthetic_rpy_fitted_theta=", fitted_rpy.tolist())
    print("synthetic_rpy_abs_error=", rpy_error.tolist())
    print("synthetic_rpy_fitted_l2=", fitted_rpy_l2)
    if not np.all(np.isfinite(fitted_rpy)):
        raise SystemExit("synthetic rpy smoke test failed")
    if not math.isfinite(fitted_rpy_l2):
        raise SystemExit("synthetic rpy l2 smoke test failed")
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

    fit_options = {
        "dq_bound_deg": float(args.dq_bound_deg),
        "dq_prior_sigma_deg": float(args.dq_prior_sigma_deg),
        "l2_prior_m": float(args.l2),
        "l2_prior_sigma_m": float(args.l2_prior_sigma_m),
        "target_offset_prior_sigma_m": float(args.target_offset_prior_sigma_m),
        "l1_xy_prior_sigma_m": float(args.l1_xy_prior_sigma_m),
        "base_rpy_bound_deg": float(args.base_rpy_bound_deg),
        "base_rpy_prior_sigma_deg": float(args.base_rpy_prior_sigma_deg),
        "joint_axis_bound_deg": float(args.joint_axis_bound_deg),
        "joint_axis_prior_sigma_deg": float(args.joint_axis_prior_sigma_deg),
    }
    result, initial_full, fitted_full, initial_l1_m, fitted_l1_m, initial_l2_m, fitted_l2_m = fit_stage1(
        samples,
        args.l1,
        args.l2,
        args.fit_mode,
        fit_options=fit_options,
    )
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
    if args.fit_mode in (FIT_MODE_FULL_L1_L2, FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print("fitted_l1_m=%.6f" % result_dict["fitted_l1_m"])
    if args.fit_mode in (FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print(
            "fitted_l1_vector_m=(%.6f, %.6f, %.6f)"
            % (
                result_dict["fitted_l1_vector_m"]["x"],
                result_dict["fitted_l1_vector_m"]["y"],
                result_dict["fitted_l1_vector_m"]["z"],
            )
        )
    if args.fit_mode in (FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print(
            "fitted_base_rpy_deg=(%.6f, %.6f, %.6f)"
            % (
                result_dict["fitted_theta"]["arm_base_rot_roll_deg"],
                result_dict["fitted_theta"]["arm_base_rot_pitch_deg"],
                result_dict["fitted_theta"]["arm_base_rot_yaw_deg"],
            )
        )
    if args.fit_mode == FIT_MODE_EXTENDED_REG_RPY_AXES:
        print(
            "fitted_joint_axis_tilt_deg=(%.6f, %.6f, %.6f, %.6f)"
            % (
                result_dict["fitted_theta"]["joint1_axis_tilt_x_deg"],
                result_dict["fitted_theta"]["joint1_axis_tilt_y_deg"],
                result_dict["fitted_theta"]["joint2_axis_tilt_x_deg"],
                result_dict["fitted_theta"]["joint2_axis_tilt_z_deg"],
            )
        )
    if args.fit_mode in (FIT_MODE_DX_L2, FIT_MODE_FULL_L2, FIT_MODE_FULL_L1_L2, FIT_MODE_EXTENDED, FIT_MODE_EXTENDED_REG, FIT_MODE_EXTENDED_REG_RPY, FIT_MODE_EXTENDED_REG_RPY_AXES):
        print("fitted_l2_m=%.6f" % result_dict["fitted_l2_m"])
    print_launch_snippet(result_dict)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
