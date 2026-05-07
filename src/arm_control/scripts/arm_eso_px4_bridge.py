#!/usr/bin/env python

import math

import rospy
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v10 import common as mavlink1
from uam_message.msg import arm_angle


class ArmEsoPx4Bridge(object):
    SOURCE_SYSTEM_ID = 1
    SOURCE_COMPONENT_ID = 191
    MESSAGE_KEYS = (
        "AJQ0",
        "AJQ1",
        "AJQ2",
        "AJQ3",
        "AJD0",
        "AJD1",
        "AJD2",
        "AJD3",
        "AJVAL",
    )

    def __init__(self):
        self.arm_angle_topic = rospy.get_param(
            "~arm_angle_topic", "/wjl/arm/real/angle_r"
        )
        self.mavlink_topic = rospy.get_param("~mavlink_topic", "/mavlink/to")
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 120.0))
        self.source_timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~source_timeout", 0.25))
        )
        self.max_abs_dq = float(rospy.get_param("~max_abs_dq", 20.0))
        self.arm1_offset_deg = float(rospy.get_param("~arm1_offset_deg", 0.0))
        self.arm2_offset_deg = float(rospy.get_param("~arm2_offset_deg", 0.0))
        self.invert_arm1 = self._param_bool("~invert_arm1", False)
        self.invert_arm2 = self._param_bool("~invert_arm2", False)

        self._mavlink = mavlink1.MAVLink(
            None,
            srcSystem=self.SOURCE_SYSTEM_ID,
            srcComponent=self.SOURCE_COMPONENT_ID,
        )
        self._publisher = rospy.Publisher(self.mavlink_topic, Mavlink, queue_size=100)
        self._subscriber = rospy.Subscriber(
            self.arm_angle_topic, arm_angle, self._arm_angle_cb, queue_size=20
        )

        self._latest_stamp = None
        self._last_sample_stamp = None
        self._last_q0 = None
        self._last_q1 = None
        self._q0 = 0.0
        self._q1 = 0.0
        self._dq0 = 0.0
        self._dq1 = 0.0
        self._message_index = 0

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _angle_deg_to_rad(self, angle_deg, offset_deg, invert):
        value = angle_deg - offset_deg
        if invert:
            value = -value
        return math.radians(value)

    def _finite(self, value):
        try:
            return math.isfinite(value)
        except AttributeError:
            return not (math.isinf(value) or math.isnan(value))

    def _clamp(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _arm_angle_cb(self, msg):
        now = rospy.Time.now()
        q0 = self._angle_deg_to_rad(
            msg.arm1_angle, self.arm1_offset_deg, self.invert_arm1
        )
        q1 = self._angle_deg_to_rad(
            msg.arm2_angle, self.arm2_offset_deg, self.invert_arm2
        )

        if not self._finite(q0) or not self._finite(q1):
            rospy.logwarn_throttle(1.0, "arm_eso_px4_bridge received non-finite angle")
            return

        if self._last_sample_stamp is not None:
            dt = (now - self._last_sample_stamp).to_sec()
            if dt > 0.005 and dt < 0.5 and self._last_q0 is not None:
                self._dq0 = self._clamp((q0 - self._last_q0) / dt, self.max_abs_dq)
                self._dq1 = self._clamp((q1 - self._last_q1) / dt, self.max_abs_dq)
            else:
                self._dq0 = 0.0
                self._dq1 = 0.0
        else:
            self._dq0 = 0.0
            self._dq1 = 0.0

        self._q0 = q0
        self._q1 = q1
        self._last_q0 = q0
        self._last_q1 = q1
        self._last_sample_stamp = now
        self._latest_stamp = now

    def _source_fresh(self, now):
        if self._latest_stamp is None:
            return False
        return (now - self._latest_stamp) <= self.source_timeout

    def _publish_named_value(self, key, value, now):
        if not self._finite(value):
            value = 0.0
        time_boot_ms = int(now.to_sec() * 1000.0) & 0xFFFFFFFF
        mav_msg = self._mavlink.named_value_float_encode(
            time_boot_ms, key.encode("ascii"), float(value)
        )
        mav_msg.pack(self._mavlink)
        self._publisher.publish(convert_to_rosmsg(mav_msg, stamp=now))

    def spin(self):
        rospy.loginfo(
            "arm_eso_px4_bridge publishing %s -> %s at %.1f Hz",
            self.arm_angle_topic,
            self.mavlink_topic,
            self.send_rate_hz,
        )
        rate = rospy.Rate(self.send_rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            fresh = self._source_fresh(now)
            values = {
                "AJQ0": self._q0,
                "AJQ1": self._q1,
                "AJQ2": 0.0,
                "AJQ3": 0.0,
                "AJD0": self._dq0 if fresh else 0.0,
                "AJD1": self._dq1 if fresh else 0.0,
                "AJD2": 0.0,
                "AJD3": 0.0,
                "AJVAL": 1.0 if fresh else 0.0,
            }

            key = self.MESSAGE_KEYS[self._message_index]
            self._publish_named_value(key, values[key], now)
            self._message_index = (self._message_index + 1) % len(self.MESSAGE_KEYS)
            rate.sleep()


def main():
    rospy.init_node("arm_eso_px4_bridge")
    bridge = ArmEsoPx4Bridge()
    bridge.spin()


if __name__ == "__main__":
    main()
