#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Stage-1 静态几何标定拟合脚本。
#
# 这个脚本运行在主机电脑上，不参与真机控制。
# 它的输入是一包已经录好的静态标定 rosbag，核心任务是：
# 1. 根据 /wjl/calibration/sample_index 自动找出每个有效静态样本窗口；
# 2. 对每个窗口内的 arm_base / arm_target / real_angle 数据取均值；
# 3. 建立“动捕测得的几何关系”和“关节角正运动学几何关系”之间的残差；
# 4. 用最小二乘拟合 stage-1 参数：
#    [arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m,
#     arm1_zero_offset_deg, arm2_zero_offset_deg]
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


# 默认输入话题。
# 这些默认值和录包脚本 record_static_calibration_data.sh 保持一致。
TOPIC_BASE = "/vrpn_client_node/arm_base/pose"
TOPIC_TARGET = "/vrpn_client_node/arm_target/pose"
TOPIC_ARM_REAL = "/wjl/arm/real/angle_r"
TOPIC_SAMPLE_INDEX = "/wjl/calibration/sample_index"


@dataclass
class Window:
    # 一个有效静态样本窗口。
    # sample_index 是采样节点打的标签，start/end 是这个标签在 bag 中持续有效的时间范围。
    sample_index: int
    start_sec: float
    end_sec: float

    @property
    def duration_sec(self) -> float:
        return self.end_sec - self.start_sec


@dataclass
class SampleMean:
    # 每个窗口经过“窗口内均值化”之后的结果。
    # 拟合器不直接用原始高频消息，而是用这里的每窗口均值样本。
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
    # 这里的最小窗口时长不是“平稳性检测”，而是对采样窗口做一次最基础的完整性筛选。
    # 例如采样中途被打断、编号只持续了很短一段时间，这类窗口会被直接丢弃。
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
    # 四元数转旋转矩阵，用于：
    # 1. 把 body 固定外参 [dx,dy,dz] 旋到世界系；
    # 2. 把世界系相对向量转回基座系。
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
    # 四元数平均前先做符号对齐。
    # 因为 q 和 -q 表示同一个姿态，如果不先对齐，直接平均会互相抵消。
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
    # 第一步：只看 /wjl/calibration/sample_index，自动提取“有效静态样本窗口”。
    #
    # 规则：
    # - sample_index >= 0  表示当前时刻属于某个有效样本窗口；
    # - sample_index == -1 表示移动/收敛阶段，这段数据不用于拟合；
    # - 同一个编号连续保持的时间段，构成一个窗口；
    # - 窗口长度必须 >= min_window_sec，太短的窗口直接丢弃。
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
    # 从整包数据里裁出某个窗口内、某个 topic 的所有消息。
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
    # 第二步：对每个有效窗口做“窗口内均值化”。
    #
    # 信息流是：
    #   Window -> 取出该窗口内的 base/target/real_angle 消息 -> 求均值 -> SampleMean
    #
    # 这里没有再做复杂的平稳性检测，只要：
    # - 三类消息都存在
    # - 窗口时长已经通过前一步筛选
    # 就会生成一个均值样本。
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
    # 理论正运动学模型。
    # 输入：实测关节角 + 待拟合的零位偏置
    # 输出：肩关节到末端工作点的理论向量（基座系）
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
    # 第三步：构造最小二乘残差。
    #
    # 参数向量 theta = [dx, dy, dz, dq1_deg, dq2_deg]
    # 其中：
    # - [dx,dy,dz] 是 arm_base 刚体原点相对真实基座原点的 body 固定外参
    # - dq1/dq2 是关节零位偏置
    #
    # 对每个样本：
    # 1. 用 base_quaternion 把 [dx,dy,dz] 旋到世界系，得到真实基座原点 p_B^W
    # 2. 用 target_position - p_B^W 算出世界系下“基座到末端”的向量
    # 3. 再转回基座系，减去 [0,0,-L1] 得到测量向量 p_rel_meas
    # 4. 用 forward_kinematics 得到理论向量 p_rel_fk
    # 5. 残差 = p_rel_meas - p_rel_fk
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
    # 把整体残差向量按样本拆回 3 维向量，便于输出每个样本的误差范数。
    vec = compute_residual_vector(theta, samples, l1_m, l2_m)
    reshaped = vec.reshape((-1, 3))
    return [float(np.linalg.norm(row)) for row in reshaped]


def rms_from_residual_vector(residual_vector: np.ndarray) -> float:
    if residual_vector.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(residual_vector))))


def fit_stage1(samples: Sequence[SampleMean], l1_m: float, l2_m: float):
    # 第四步：调用 scipy 的 least_squares 做 stage-1 拟合。
    #
    # 这里是一个带边界约束的非线性最小二乘问题。
    # 初值全部从 0 开始，表示先假设“没有外参偏移、没有零位偏置”。
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
    # 把“窗口均值样本”导出成 CSV，方便人工检查每个样本到底取到了什么均值。
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
    result,
    initial_theta: np.ndarray,
    samples: Sequence[SampleMean],
    l1_m: float,
    l2_m: float,
) -> Dict[str, object]:
    # 汇总拟合结果，输出成 JSON。
    fitted_theta = np.array(result.x, dtype=float)
    initial_residual_vector = compute_residual_vector(initial_theta, samples, l1_m, l2_m)
    fitted_residual_vector = compute_residual_vector(fitted_theta, samples, l1_m, l2_m)
    return {
        "l1_m": l1_m,
        "l2_m": l2_m,
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
        "per_sample_residual_norm_m": residual_norms(fitted_theta, samples, l1_m, l2_m),
        "optimizer": {
            "success": bool(result.success),
            "status": int(result.status),
            "message": str(result.message),
            "cost": float(result.cost),
            "nfev": int(result.nfev),
        },
    }


def print_launch_snippet(result_dict: Dict[str, object]) -> None:
    # 把最终参数打印成可直接抄回 launch 的片段。
    theta = result_dict["fitted_theta"]
    print("\nSuggested launch params:")
    print('<param name="arm_base_offset_x_m" value="%.6f" />' % theta["arm_base_offset_x_m"])
    print('<param name="arm_base_offset_y_m" value="%.6f" />' % theta["arm_base_offset_y_m"])
    print('<param name="arm_base_offset_z_m" value="%.6f" />' % theta["arm_base_offset_z_m"])
    print('<param name="arm1_zero_offset_deg" value="%.6f" />' % theta["arm1_zero_offset_deg"])
    print('<param name="arm2_zero_offset_deg" value="%.6f" />' % theta["arm2_zero_offset_deg"])


def run_synthetic_test() -> int:
    # 合成数据自检：
    # 人为构造一组已知真值参数，生成样本，再看拟合器能否大致回归出这些参数。
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

    result, initial_theta = fit_stage1(samples, l1_m, l2_m)
    fitted_theta = np.array(result.x, dtype=float)
    error = np.abs(fitted_theta - true_theta)
    print("synthetic_true_theta=", true_theta.tolist())
    print("synthetic_fitted_theta=", fitted_theta.tolist())
    print("synthetic_abs_error=", error.tolist())
    # 阈值足够宽松，只是为了防止脚本逻辑坏掉。
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

    # 主流程：
    #   bag -> extract_windows -> compute_sample_means -> fit_stage1 -> json/csv + launch snippet
    with rosbag.Bag(args.bag, "r") as bag:
        windows = extract_windows(bag, args.sample_topic, args.min_window_sec)
        if not windows:
            raise SystemExit("no valid sample windows found in %s" % args.sample_topic)
        samples = compute_sample_means(bag, windows, args.base_topic, args.target_topic, args.arm_real_topic)

    if not samples:
        raise SystemExit("no valid sample means were generated")

    result, initial_theta = fit_stage1(samples, args.l1, args.l2)
    result_dict = build_result_dict(result, initial_theta, samples, args.l1, args.l2)

    csv_path = os.path.join(output_dir, "sample_means.csv")
    json_path = os.path.join(output_dir, "fit_result.json")
    write_sample_csv(csv_path, samples)
    with open(json_path, "w") as f:
        json.dump(result_dict, f, indent=2, sort_keys=True)

    print("sample_count=%d" % len(samples))
    print("sample_csv=%s" % csv_path)
    print("fit_json=%s" % json_path)
    print("initial_rms_m=%.6f" % result_dict["initial_rms_m"])
    print("fitted_rms_m=%.6f" % result_dict["fitted_rms_m"])
    print_launch_snippet(result_dict)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
