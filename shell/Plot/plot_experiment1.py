#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 该脚本用于离线读取实验一的 rosbag 数据，并生成论文风格的结果图。
# 设计目标：
# 1. 尽量不依赖在线 ROS 节点，方便在实验结束后重复出图；
# 2. 代码结构清晰，便于后续替换坐标轴、叠加算法结果、调整版式；
# 3. 当前版本明确按“本机环境”编写，即 Ubuntu 20.04 / ROS Noetic / Python 3。
#
# 额外说明：
# 你的本机虽然是 Noetic + Python3，但两个工作空间里可能还残留旧的 Python2 消息导出。
# 因此这里不直接 import 自定义消息类，而是改成从 bag 的 message_definition 动态生成消息类型，
# 这样在本机上离线出图会更稳，也不依赖重新编译 Python 消息包。

import argparse
import math
import os

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from genpy.dynamic import generate_dynamic

try:
    import rosbag
except ImportError:
    rosbag = None


TOPIC_VRPN_POSE = "/vrpn_client_node/Tracker0/pose"
TOPIC_LOCAL_POSE = "/mavros/local_position/pose"
TOPIC_SETPOINT = "/mavros/setpoint_raw/local"
TOPIC_GUIDE_POSE = "/wjl/guidefly/pose_d"
TOPIC_ARM_DESIRED = "/wjl/arm/guidefly/angle_d"
TOPIC_ARM_REAL = "/wjl/arm/real/angle_r"
TOPIC_ARM_ERROR = "/wjl/arm/real/angle_error"

# 下面这组参数直接来自现有 2 自由度机械臂建模脚本 arm_B_dh.py。
# 这里不再重复做 ROS 可视化，而是只复用离线正运动学所需的 DH 参数。
ARM_DH_ALPHA = [-math.pi / 2.0, 0.0]
ARM_DH_A = [0.0, 0.21]
ARM_DH_D = [0.053, 0.007]
ARM_DH_THETA = [0.0, 0.0]
ARM_T_B0 = np.array(
    [[1.0, 0.0, 0.0, 0.104], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.10194], [0.0, 0.0, 0.0, 1.0]]
)


def parse_args():
    # 保持命令行接口尽量简单，重点是“输入 bag”和“输出目录”。
    # 其他参数只用于后续裁剪时段或叠加算法对比曲线。
    parser = argparse.ArgumentParser(description="Plot experiment 1 figures from rosbag.")
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    parser.add_argument("--algorithm-base-csv", default="", help="Optional algorithm base CSV")
    parser.add_argument("--t-start", type=float, default=None, help="Optional start time in seconds")
    parser.add_argument("--t-end", type=float, default=None, help="Optional end time in seconds")
    return parser.parse_args()


def ensure_rosbag_available():
    # rosbag 是该脚本最核心的依赖。
    # 如果当前 python 环境没有 rosbag，直接报错，避免后面出现一连串误导性异常。
    if rosbag is None:
        raise RuntimeError("rosbag python module is not available in this environment")


def decode_header_text(value):
    # Noetic + Python3 下，connection header 里的字段常常是 bytes；
    # 为了后面统一传给 genpy.dynamic，这里先全部转成 str。
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def deserialize_dynamic_message(raw_msg, connection_header, class_cache):
    # 关键兼容点：
    # 不依赖当前 Python 环境里已经安装好的自定义消息模块，
    # 而是直接使用 bag 自带的消息定义文本，动态生成消息类再反序列化。
    # 这样即使本机工作空间目前还是 Python2 生成的消息包，也能在 Python3 下离线读包。
    datatype = decode_header_text(connection_header["type"])
    message_definition = decode_header_text(connection_header["message_definition"])

    if datatype not in class_cache:
        class_cache[datatype] = generate_dynamic(datatype, message_definition)[datatype]

    msg_class = class_cache[datatype]
    msg = msg_class()
    msg.deserialize(raw_msg[1])
    return msg


def safe_stamp_to_sec(msg, fallback_time):
    # 不同消息的时间戳来源可能不同：
    # 1. 如果消息本身有 header.stamp，则优先使用消息时间；
    # 2. 否则回退到 bag 的记录时间。
    # 这样能兼容更多消息类型，并减少时间轴错位。
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    return fallback_time


def add_sample(store, key, time_sec, values):
    # 用一个统一的字典结构按 topic 暂存原始样本，便于后续统一整理成 numpy 数组。
    bucket = store.setdefault(key, [])
    bucket.append((time_sec, values))


def samples_to_arrays(samples):
    # 将离散样本列表转成结构统一的 ndarray：
    # {
    #   "time":  shape=(N,),
    #   "value": shape=(N, D)
    # }
    # 这样后续重采样、误差计算、绘图都能复用同一种接口。
    if not samples:
        return None
    samples.sort(key=lambda item: item[0])
    time_array = np.array([item[0] for item in samples], dtype=float)
    value_array = np.array([item[1] for item in samples], dtype=float)
    if value_array.ndim == 1:
        value_array = value_array.reshape((-1, 1))
    return {"time": time_array, "value": value_array}


def load_bag_data(bag_path):
    # 从 rosbag 中只读取本次论文风格成图所需的话题。
    # 这里不用 rosbag record -a 的全量数据，而是读白名单，能让后续逻辑更稳定。
    #
    # 实现细节：
    # 这里使用 raw=True + 动态消息反序列化，而不是直接让 rosbag 去 import 消息类。
    # 原因是你本机当前是 Noetic/Python3，但工程工作空间里仍可见 Python2.7 的消息导出；
    # 如果直接依赖静态 import，自定义消息很容易在本机上解包失败。
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
            # 统一将不同消息类型映射成数值向量。
            # 位姿类话题统一取 [x, y, z]
            # 关节类话题统一取 [arm1, arm2, hand]
            if topic in (TOPIC_VRPN_POSE, TOPIC_LOCAL_POSE):
                values = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            elif topic == TOPIC_SETPOINT:
                values = [msg.position.x, msg.position.y, msg.position.z]
            elif topic == TOPIC_GUIDE_POSE:
                values = [msg.x_d, msg.y_d, msg.z_d]
            elif topic in (TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR):
                values = [msg.arm1_angle, msg.arm2_angle, msg.hand_angle]
            else:
                continue
            add_sample(topic_store, topic, stamp, values)

    output = {}
    for topic, samples in topic_store.items():
        output[topic] = samples_to_arrays(samples)
    return output


def crop_series(series, t_start, t_end):
    # 用于手动截取实验有效时段。
    # 常见用途：
    # 1. 去掉起飞前的静止时间；
    # 2. 去掉降落段；
    # 3. 只看论文中某一段关键扰动区间。
    if series is None:
        return None
    time_array = series["time"]
    value_array = series["value"]
    mask = np.ones(time_array.shape, dtype=bool)
    if t_start is not None:
        mask &= time_array >= t_start
    if t_end is not None:
        mask &= time_array <= t_end
    time_array = time_array[mask]
    value_array = value_array[mask]
    if time_array.size == 0:
        return None
    return {"time": time_array, "value": value_array}


def normalize_series_times(series_map, t_start=None, t_end=None):
    # 所有话题的时间轴统一减去“全局最早起点”，让横轴从 0 秒开始。
    # 这样论文出图更整洁，也方便后续人工比对不同时次实验。
    valid_times = []
    for series in series_map.values():
        if series is not None:
            valid_times.append(series["time"][0])
    if not valid_times:
        raise RuntimeError("No valid data found in rosbag")
    global_start = min(valid_times)
    normalized = {}
    for key, series in series_map.items():
        if series is None:
            normalized[key] = None
            continue
        shifted = {"time": series["time"] - global_start, "value": series["value"]}
        normalized[key] = crop_series(shifted, t_start, t_end)
    return normalized


def load_algorithm_csv(path):
    # 这里只给“基座算法结果”留接口。
    # CSV 约定列名固定为：time_s,x,y,z
    # 如果后续算法代码成熟，只要导出同结构 CSV，就能无缝叠加到论文图中。
    if not path:
        return None
    if not os.path.isfile(path):
        raise RuntimeError("Algorithm CSV not found: %s" % path)
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        return None
    if data.shape == ():
        time_array = np.array([float(data["time_s"])], dtype=float)
        value_array = np.array([[float(data["x"]), float(data["y"]), float(data["z"])]], dtype=float)
    else:
        time_array = np.array(data["time_s"], dtype=float)
        value_array = np.column_stack([data["x"], data["y"], data["z"]]).astype(float)
    return {"time": time_array, "value": value_array}


def build_common_time(series_list, sample_count=800):
    # 不同话题采样频率不同，必须先插值到统一时间轴。
    # 这里取所有序列的重叠时间段，再做等间距采样，避免外推。
    valid = [series for series in series_list if series is not None and series["time"].size > 0]
    if not valid:
        raise RuntimeError("No valid series for interpolation")
    start_time = max(series["time"][0] for series in valid)
    end_time = min(series["time"][-1] for series in valid)
    if end_time <= start_time:
        ref = valid[0]["time"]
        return ref.copy()
    return np.linspace(start_time, end_time, sample_count)


def interp_series(series, target_time):
    # 对每一列独立做一维线性插值。
    # 这是整个离线绘图的基础：
    # 所有误差统计、末端轨迹计算、对比出图都建立在统一时间轴上。
    if series is None:
        return None
    source_time = series["time"]
    source_value = series["value"]
    if source_time.size == 1:
        return np.repeat(source_value, target_time.size, axis=0)
    result = np.zeros((target_time.size, source_value.shape[1]), dtype=float)
    for index in range(source_value.shape[1]):
        result[:, index] = np.interp(target_time, source_time, source_value[:, index])
    return result


def dh_matrix(alpha, a, d, theta):
    # 标准 DH 变换矩阵。
    # 后面正运动学会按 link 顺序将这些矩阵逐级相乘。
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)
    return np.array(
        [
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0.0, sin_alpha, cos_alpha, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward_kinematics(arm1_deg, arm2_deg):
    # 将两个关节角（单位：度）转换成末端在机体坐标系中的三维位置。
    # 当前论文图主用“末端主扰动轴”的时序曲线，因此这里只需要末端位置，不需要姿态。
    theta_values = [math.radians(arm1_deg), math.radians(arm2_deg)]
    transform = np.eye(4)
    transform = np.dot(transform, ARM_T_B0)
    for index in range(2):
        transform = np.dot(
            transform,
            dh_matrix(
                ARM_DH_ALPHA[index],
                ARM_DH_A[index],
                ARM_DH_D[index],
                ARM_DH_THETA[index] + theta_values[index],
            ),
        )
    return transform[:3, 3]


def compute_end_effector(series_values):
    # 将整段关节时序批量变换成末端轨迹。
    # 输入 shape=(N,3)，这里只使用前两个关节角，hand 不参与正运动学。
    positions = np.zeros((series_values.shape[0], 3), dtype=float)
    for index in range(series_values.shape[0]):
        positions[index, :] = forward_kinematics(series_values[index, 0], series_values[index, 1])
    return positions


def choose_dominant_axis(reference_positions):
    # 论文实验一更强调“主扰动方向”的跟踪表现。
    # 这里自动选择参考末端轨迹变化幅值最大的轴作为主展示轴，
    # 避免每次人工切换 x/y/z。
    ranges = np.ptp(reference_positions, axis=0)
    axis_index = int(np.argmax(ranges))
    labels = ["x", "y", "z"]
    return axis_index, labels[axis_index]


def compute_error(actual_values, reference_values):
    # 统一误差定义：actual - reference
    # 同时输出逐轴误差和欧氏范数，方便做论文里的 mean/max 统计。
    diff = actual_values - reference_values
    norm = np.linalg.norm(diff, axis=1)
    return diff, norm


def series_stats(values):
    # 论文风格图中常需要标注 mean / max error。
    return float(np.mean(values)), float(np.max(values))


def style_axis(axis):
    # 将每张图的风格统一成论文/汇报常用的“白底+黑轴+浅色网格”样式。
    axis.grid(True, linestyle="--", linewidth=0.6, alpha=0.4)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(direction="out", length=4, width=1)


def add_placeholder(axis, text):
    # 当算法 CSV 暂时不存在时，在图内保留一个明显但不刺眼的占位提示。
    # 后续只要补 CSV，就不用改图布局。
    axis.text(
        0.5,
        0.5,
        text,
        transform=axis.transAxes,
        ha="center",
        va="center",
        color="0.5",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.75", alpha=0.9),
    )


def save_main_figure(output_dir, common_time, ee_ref, ee_real, dominant_axis_label, ref_base, px4_base, algo_base):
    # 主图对应论文最核心的结果图，固定三行子图：
    # (a) 末端轨迹跟踪
    # (b) 基座位置跟踪
    # (c) 基座位置误差对比
    figure, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    dominant_index = ["x", "y", "z"].index(dominant_axis_label)
    # (a) 机械臂末端图不做算法占位，因为当前算法未变化，重点是参考与实际跟踪。
    axes[0].plot(common_time, ee_ref[:, dominant_index], color="#111111", linewidth=2.0, label="Reference")
    axes[0].plot(common_time, ee_real[:, dominant_index], color="#d62728", linewidth=1.8, label="PX4 / measured")
    axes[0].set_ylabel("EE %s (m)" % dominant_axis_label)
    axes[0].set_title("(a) End-effector tracking")
    axes[0].legend(loc="best", frameon=False)
    style_axis(axes[0])

    # (b) 基座位置对比图将 PX4 与未来算法统一放在一张图里，
    # 方便后续直接替换/对比，不需要重新改布局。
    for axis_index, axis_name in enumerate(axis_names):
        axes[1].plot(
            common_time,
            ref_base[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.2,
            linestyle=":",
            label="Ref %s" % axis_name,
        )
        axes[1].plot(
            common_time,
            px4_base[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="PX4 %s" % axis_name,
        )
        if algo_base is not None:
            axes[1].plot(
                common_time,
                algo_base[:, axis_index],
                color=axis_colors[axis_index],
                linewidth=1.4,
                linestyle="--",
                label="Algo %s" % axis_name,
            )
    if algo_base is None:
        add_placeholder(axes[1], "Algorithm placeholder / CSV not provided")
    axes[1].set_ylabel("Base position (m)")
    axes[1].set_title("(b) Base position tracking")
    axes[1].legend(loc="upper right", ncol=3, frameon=False, fontsize=8)
    style_axis(axes[1])

    # (c) 基座误差对比图，用误差范数做整体指标，并在图内直接标注 mean/max。
    _, px4_norm = compute_error(px4_base, ref_base)
    axes[2].plot(common_time, px4_norm, color="#d62728", linewidth=2.0, label="PX4 error norm")
    px4_mean, px4_max = series_stats(px4_norm)
    text_lines = ["PX4 mean = %.4f m" % px4_mean, "PX4 max = %.4f m" % px4_max]

    if algo_base is not None:
        _, algo_norm = compute_error(algo_base, ref_base)
        axes[2].plot(common_time, algo_norm, color="#4c78a8", linewidth=1.8, linestyle="--", label="Algo error norm")
        algo_mean, algo_max = series_stats(algo_norm)
        text_lines.append("Algo mean = %.4f m" % algo_mean)
        text_lines.append("Algo max = %.4f m" % algo_max)
    else:
        add_placeholder(axes[2], "Algorithm placeholder / CSV not provided")

    axes[2].text(
        0.02,
        0.98,
        "\n".join(text_lines),
        transform=axes[2].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="0.8", alpha=0.9),
    )
    axes[2].set_ylabel("Error norm (m)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("(c) Base position error comparison")
    axes[2].legend(loc="best", frameon=False)
    style_axis(axes[2])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp1_main_figure.png"), dpi=300)
    plt.close(figure)


def save_joint_diagnostics(output_dir, common_time, desired_arm, real_arm, error_arm):
    # 这张图用于工程调参与诊断，不完全对应论文主图，但对你后续微调很重要：
    # 1. 关节期望是否发出；
    # 2. 关节实测是否跟上；
    # 3. 误差主要出现在 arm1 还是 arm2。
    figure, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(common_time, desired_arm[:, 0], color="#111111", linewidth=2.0, label="arm1 desired")
    axes[0].plot(common_time, real_arm[:, 0], color="#d62728", linewidth=1.8, label="arm1 measured")
    axes[0].set_ylabel("arm1 (deg)")
    axes[0].set_title("Arm joint diagnostics")
    axes[0].legend(loc="best", frameon=False)
    style_axis(axes[0])

    axes[1].plot(common_time, desired_arm[:, 1], color="#111111", linewidth=2.0, label="arm2 desired")
    axes[1].plot(common_time, real_arm[:, 1], color="#1f77b4", linewidth=1.8, label="arm2 measured")
    axes[1].set_ylabel("arm2 (deg)")
    axes[1].legend(loc="best", frameon=False)
    style_axis(axes[1])

    axes[2].plot(common_time, error_arm[:, 0], color="#d62728", linewidth=1.5, label="arm1 error")
    axes[2].plot(common_time, error_arm[:, 1], color="#1f77b4", linewidth=1.5, label="arm2 error")
    axes[2].axhline(0.0, color="0.35", linewidth=1.0, linestyle=":")
    axes[2].set_ylabel("error (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best", frameon=False)
    style_axis(axes[2])

    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "exp1_joint_diagnostics.png"), dpi=300)
    plt.close(figure)


def save_base_error_summary(output_dir, common_time, ref_base, px4_base, algo_base):
    # 这张图比主图更工程化，便于单独看每个坐标轴误差和误差范数，
    # 尤其适合复现实验后快速比较不同参数组。
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axis_colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    axis_names = ["x", "y", "z"]

    px4_diff, px4_norm = compute_error(px4_base, ref_base)
    if algo_base is not None:
        algo_diff, algo_norm = compute_error(algo_base, ref_base)
    else:
        algo_diff, algo_norm = None, None

    for axis_index, axis_name in enumerate(axis_names):
        axes[axis_index].plot(
            common_time,
            px4_diff[:, axis_index],
            color=axis_colors[axis_index],
            linewidth=1.8,
            label="PX4 %s error" % axis_name,
        )
        if algo_diff is not None:
            axes[axis_index].plot(
                common_time,
                algo_diff[:, axis_index],
                color=axis_colors[axis_index],
                linewidth=1.4,
                linestyle="--",
                label="Algo %s error" % axis_name,
            )
        axes[axis_index].axhline(0.0, color="0.35", linewidth=1.0, linestyle=":")
        axes[axis_index].set_ylabel("%s err (m)" % axis_name)
        axes[axis_index].legend(loc="best", frameon=False)
        style_axis(axes[axis_index])

    axes[3].plot(common_time, px4_norm, color="#d62728", linewidth=2.0, label="PX4 norm")
    px4_mean, px4_max = series_stats(px4_norm)
    text_lines = ["PX4 mean = %.4f m" % px4_mean, "PX4 max = %.4f m" % px4_max]

    if algo_norm is not None:
        axes[3].plot(common_time, algo_norm, color="#4c78a8", linewidth=1.8, linestyle="--", label="Algo norm")
        algo_mean, algo_max = series_stats(algo_norm)
        text_lines.append("Algo mean = %.4f m" % algo_mean)
        text_lines.append("Algo max = %.4f m" % algo_max)
    else:
        add_placeholder(axes[3], "Algorithm placeholder / CSV not provided")

    axes[3].text(
        0.02,
        0.98,
        "\n".join(text_lines),
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
    figure.savefig(os.path.join(output_dir, "exp1_base_error_summary.png"), dpi=300)
    plt.close(figure)


def main():
    # 主流程固定为：
    # 1. 解析命令行参数；
    # 2. 读取并标准化 bag 数据；
    # 3. 对齐到统一时间轴；
    # 4. 离线计算末端轨迹；
    # 5. 输出三张 PNG 图。
    args = parse_args()

    # 当前脚本已明确面向本机的 Python3 环境，因此直接用更自然的写法。
    os.makedirs(args.output_dir, exist_ok=True)

    bag_data = load_bag_data(args.bag)
    bag_data = normalize_series_times(bag_data, args.t_start, args.t_end)

    required_topics = [TOPIC_VRPN_POSE, TOPIC_SETPOINT, TOPIC_ARM_DESIRED, TOPIC_ARM_REAL, TOPIC_ARM_ERROR]
    for topic in required_topics:
        if bag_data.get(topic) is None:
            raise RuntimeError("Required topic missing or empty in bag: %s" % topic)

    # 基座算法对比是后续扩展项，所以这里按“可选输入”处理。
    algorithm_base = load_algorithm_csv(args.algorithm_base_csv)
    if algorithm_base is not None:
        algorithm_base = crop_series(algorithm_base, args.t_start, args.t_end)

    # 构造所有核心曲线的公共时间轴。
    # 注意这里用的是动捕、参考、关节期望、关节实测的交集时间段。
    common_time = build_common_time(
        [bag_data[TOPIC_VRPN_POSE], bag_data[TOPIC_SETPOINT], bag_data[TOPIC_ARM_DESIRED], bag_data[TOPIC_ARM_REAL]]
        + ([algorithm_base] if algorithm_base is not None else [])
    )

    # 将所有曲线都插值到同一时间轴，保证后续比较是逐时刻对齐的。
    vrpn_pose = interp_series(bag_data[TOPIC_VRPN_POSE], common_time)
    ref_base = interp_series(bag_data[TOPIC_SETPOINT], common_time)
    desired_arm = interp_series(bag_data[TOPIC_ARM_DESIRED], common_time)
    real_arm = interp_series(bag_data[TOPIC_ARM_REAL], common_time)
    error_arm = interp_series(bag_data[TOPIC_ARM_ERROR], common_time)
    algorithm_base_values = interp_series(algorithm_base, common_time) if algorithm_base is not None else None

    # 由关节角离线反推出末端轨迹，这是对齐论文实验一图形表达的关键。
    ee_ref = compute_end_effector(desired_arm)
    ee_real = compute_end_effector(real_arm)
    dominant_axis_index, dominant_axis_label = choose_dominant_axis(ee_ref)
    if np.ptp(ee_ref[:, dominant_axis_index]) < 1e-9:
        dominant_axis_label = "x"

    save_main_figure(args.output_dir, common_time, ee_ref, ee_real, dominant_axis_label, ref_base, vrpn_pose, algorithm_base_values)
    save_joint_diagnostics(args.output_dir, common_time, desired_arm, real_arm, error_arm)
    save_base_error_summary(args.output_dir, common_time, ref_base, vrpn_pose, algorithm_base_values)

    print("Saved figures to %s" % args.output_dir)


if __name__ == "__main__":
    main()
