#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(
        description="Merge experiment 3 topic CSV files into one readable monitor log."
    )
    parser.add_argument("--desired-csv", required=True, help="CSV exported from /wjl/guidefly/pose_d")
    parser.add_argument("--vrpn-csv", required=True, help="CSV exported from /vrpn_client_node/arm_base/pose")
    parser.add_argument("--mavros-csv", required=True, help="CSV exported from /mavros/local_position/pose")
    parser.add_argument("--state-csv", required=True, help="CSV exported from /mavros/state")
    parser.add_argument("--bag", default="", help="Optional rosbag path for fallback topic extraction")
    parser.add_argument("--output", required=True, help="Readable merged log output path")
    return parser.parse_args()


def to_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def to_bool_text(value):
    if value is None or value == "":
        return ""
    if str(value).lower() in ("true", "1"):
        return "true"
    if str(value).lower() in ("false", "0"):
        return "false"
    return str(value)


def pick(row, *keys):
    for key in keys:
        if key in row and row[key] != "":
            return row[key]
    return ""


def load_topic_csv(path, kind):
    rows = []
    if not os.path.isfile(path):
        return rows

    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            stamp = to_float(pick(raw, "%time", "time"))
            if stamp is None:
                continue

            if kind == "desired":
                rows.append(
                    {
                        "t": stamp,
                        "desired_x": to_float(pick(raw, "field.x_d", "x_d")),
                        "desired_y": to_float(pick(raw, "field.y_d", "y_d")),
                        "desired_z": to_float(pick(raw, "field.z_d", "z_d")),
                        "desired_yaw": to_float(pick(raw, "field.yaw_d", "yaw_d")),
                        "desired_land": to_bool_text(pick(raw, "field.land_flag", "land_flag")),
                    }
                )
            elif kind == "vrpn":
                rows.append(
                    {
                        "t": stamp,
                        "vrpn_x": to_float(pick(raw, "field.pose.position.x", "pose.position.x")),
                        "vrpn_y": to_float(pick(raw, "field.pose.position.y", "pose.position.y")),
                        "vrpn_z": to_float(pick(raw, "field.pose.position.z", "pose.position.z")),
                    }
                )
            elif kind == "mavros":
                rows.append(
                    {
                        "t": stamp,
                        "mavros_x": to_float(pick(raw, "field.pose.position.x", "pose.position.x")),
                        "mavros_y": to_float(pick(raw, "field.pose.position.y", "pose.position.y")),
                        "mavros_z": to_float(pick(raw, "field.pose.position.z", "pose.position.z")),
                    }
                )
            elif kind == "state":
                rows.append(
                    {
                        "t": stamp,
                        "mavros_connected": to_bool_text(pick(raw, "field.connected", "connected")),
                        "mavros_armed": to_bool_text(pick(raw, "field.armed", "armed")),
                        "mavros_mode": pick(raw, "field.mode", "mode"),
                        "mavros_system_status": pick(
                            raw, "field.system_status", "system_status"
                        ),
                    }
                )
    return rows


def load_desired_from_bag(bag_path):
    rows = []
    if not bag_path or not os.path.isfile(bag_path):
        return rows

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=["/wjl/guidefly/pose_d"]):
            rows.append(
                {
                    "t": t.to_sec() * 1e9,
                    "desired_x": float(msg.x_d),
                    "desired_y": float(msg.y_d),
                    "desired_z": float(msg.z_d),
                    "desired_yaw": float(msg.yaw_d),
                    "desired_land": "true" if bool(msg.land_flag) else "false",
                }
            )
    return rows


def update_latest(rows, current_time, index, latest):
    while index < len(rows) and rows[index]["t"] <= current_time + 1e-9:
        latest = rows[index]
        index += 1
    return index, latest


def norm3(ax, ay, az, bx, by, bz):
    if None in (ax, ay, az, bx, by, bz):
        return ""
    dx = ax - bx
    dy = ay - by
    dz = az - bz
    return "%.6f" % math.sqrt(dx * dx + dy * dy + dz * dz)


def write_merged_csv(desired_rows, vrpn_rows, mavros_rows, state_rows, output_path):
    all_times = sorted(
        set([row["t"] for row in desired_rows + vrpn_rows + mavros_rows + state_rows])
    )
    if not all_times:
        raise RuntimeError("No monitor topic rows found to merge.")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "ros_time",
                "desired_x",
                "desired_y",
                "desired_z",
                "desired_yaw_deg",
                "desired_land_flag",
                "arm_base_x",
                "arm_base_y",
                "arm_base_z",
                "mavros_x",
                "mavros_y",
                "mavros_z",
                "mavros_connected",
                "mavros_armed",
                "mavros_mode",
                "mavros_system_status",
                "arm_base_minus_mavros_norm",
                "arm_base_minus_desired_norm",
            ]
        )

        desired_index = vrpn_index = mavros_index = state_index = 0
        latest_desired = {}
        latest_vrpn = {}
        latest_mavros = {}
        latest_state = {}

        for current_time in all_times:
            desired_index, latest_desired = update_latest(
                desired_rows, current_time, desired_index, latest_desired
            )
            vrpn_index, latest_vrpn = update_latest(
                vrpn_rows, current_time, vrpn_index, latest_vrpn
            )
            mavros_index, latest_mavros = update_latest(
                mavros_rows, current_time, mavros_index, latest_mavros
            )
            state_index, latest_state = update_latest(
                state_rows, current_time, state_index, latest_state
            )

            writer.writerow(
                [
                    "%.6f" % current_time,
                    latest_desired.get("desired_x", ""),
                    latest_desired.get("desired_y", ""),
                    latest_desired.get("desired_z", ""),
                    latest_desired.get("desired_yaw", ""),
                    latest_desired.get("desired_land", ""),
                    latest_vrpn.get("vrpn_x", ""),
                    latest_vrpn.get("vrpn_y", ""),
                    latest_vrpn.get("vrpn_z", ""),
                    latest_mavros.get("mavros_x", ""),
                    latest_mavros.get("mavros_y", ""),
                    latest_mavros.get("mavros_z", ""),
                    latest_state.get("mavros_connected", ""),
                    latest_state.get("mavros_armed", ""),
                    latest_state.get("mavros_mode", ""),
                    latest_state.get("mavros_system_status", ""),
                    norm3(
                        latest_vrpn.get("vrpn_x"),
                        latest_vrpn.get("vrpn_y"),
                        latest_vrpn.get("vrpn_z"),
                        latest_mavros.get("mavros_x"),
                        latest_mavros.get("mavros_y"),
                        latest_mavros.get("mavros_z"),
                    ),
                    norm3(
                        latest_vrpn.get("vrpn_x"),
                        latest_vrpn.get("vrpn_y"),
                        latest_vrpn.get("vrpn_z"),
                        latest_desired.get("desired_x"),
                        latest_desired.get("desired_y"),
                        latest_desired.get("desired_z"),
                    ),
                ]
            )


def main():
    args = parse_args()
    desired_rows = load_topic_csv(args.desired_csv, "desired")
    if not desired_rows:
        desired_rows = load_desired_from_bag(args.bag)
    vrpn_rows = load_topic_csv(args.vrpn_csv, "vrpn")
    mavros_rows = load_topic_csv(args.mavros_csv, "mavros")
    state_rows = load_topic_csv(args.state_csv, "state")
    write_merged_csv(desired_rows, vrpn_rows, mavros_rows, state_rows, args.output)
    print("Merged experiment 3 monitor log written to %s" % args.output)


if __name__ == "__main__":
    main()
