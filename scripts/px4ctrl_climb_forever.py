#!/usr/bin/env python3

import argparse
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser(
        description="After px4ctrl takeoff/hover, climb to a target height and hover there."
    )
    parser.add_argument("--speed", type=float, default=2.0, help="Upward speed in m/s")
    parser.add_argument("--rate", type=float, default=50.0, help="Command publish rate in Hz")
    parser.add_argument("--vehicle", default="iris_0")
    parser.add_argument("--odom-topic", default="/iris_0/mavros/vision_odom/odom")
    parser.add_argument("--cmd-topic", default="/xtdrone/iris_0/planning/pos_cmd")
    parser.add_argument(
        "--mode-topic",
        action="append",
        default=["/iris_0/px4ctrl_mode", "/px4ctrl_mode"],
        help="px4ctrl mode topic. Can be passed more than once.",
    )
    parser.add_argument("--mode-timeout", type=float, default=30.0)
    parser.add_argument("--subscriber-timeout", type=float, default=10.0)
    parser.add_argument("--target-height", type=float, default=None)
    parser.add_argument("--default-height", type=float, default=80.0)
    return parser.parse_args()


def prompt_target_height(default_height):
    try:
        import tkinter as tk
        from tkinter import messagebox, simpledialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        height = simpledialog.askfloat(
            "飞行高度",
            "请输入目标悬停高度（m）：",
            initialvalue=default_height,
            minvalue=0.1,
            parent=root,
        )
        if height is None:
            messagebox.showinfo("已取消", "未输入目标高度，脚本已退出。", parent=root)
            root.destroy()
            sys.exit(0)
        root.destroy()
        return height
    except Exception as exc:
        print("无法打开高度输入窗口：{}".format(exc))
        raw = input("请输入目标悬停高度（m，默认 {:.1f}）：".format(default_height)).strip()
        return float(raw) if raw else default_height


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Px4ctrlClimbForever:
    READY_MODES = {"AUTO_HOVER", "CMD_CTRL"}

    def __init__(self, args):
        self.speed = abs(args.speed)
        self.rate = rospy.Rate(args.rate)
        self.vehicle = args.vehicle
        self.odom_topic = args.odom_topic
        self.cmd_topic = args.cmd_topic
        self.mode_topics = args.mode_topic
        self.mode_timeout = args.mode_timeout
        self.subscriber_timeout = args.subscriber_timeout
        self.target_height = args.target_height

        self.odom = None
        self.mode = ""
        self.mode_topic_seen = ""

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.mode_subs = [
            rospy.Subscriber(topic, String, self.mode_callback, callback_args=topic, queue_size=1)
            for topic in self.mode_topics
        ]
        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)

    def odom_callback(self, msg):
        self.odom = msg

    def mode_callback(self, msg, topic):
        self.mode = msg.data
        self.mode_topic_seen = topic

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odom: %s", self.odom_topic)
        try:
            self.odom = rospy.wait_for_message(self.odom_topic, Odometry, timeout=8.0)
        except rospy.ROSException:
            raise RuntimeError("No odom received from {}".format(self.odom_topic))

    def wait_for_px4ctrl_subscriber(self):
        rospy.loginfo("Waiting for px4ctrl command subscriber: %s", self.cmd_topic)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.cmd_pub.get_num_connections() > 0:
                return
            if (rospy.Time.now() - start).to_sec() > self.subscriber_timeout:
                raise RuntimeError(
                    "No subscriber on {}. Is px4ctrl singl_run.launch running?".format(
                        self.cmd_topic
                    )
                )
            self.rate.sleep()

    def wait_for_hover_or_cmd_mode(self):
        rospy.loginfo("Waiting for px4ctrl AUTO_HOVER/CMD_CTRL before sending commands")
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.mode in self.READY_MODES:
                rospy.loginfo("px4ctrl mode is %s (%s)", self.mode, self.mode_topic_seen)
                return
            if (rospy.Time.now() - start).to_sec() > self.mode_timeout:
                raise RuntimeError(
                    "px4ctrl is not ready. Last mode='{}' from '{}'. Click this after takeoff reaches hover.".format(
                        self.mode or "none", self.mode_topic_seen or "none"
                    )
                )
            self.rate.sleep()

    def make_command(self, x, y, z, vz, yaw, trajectory_id):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.trajectory_id = trajectory_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = vz
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        cmd.yaw = yaw
        cmd.yaw_dot = 0.0
        return cmd

    def run(self):
        self.wait_for_odom()
        self.wait_for_px4ctrl_subscriber()
        self.wait_for_hover_or_cmd_mode()

        start = rospy.Time.now()
        start_pose = self.odom.pose.pose
        hold_x = start_pose.position.x
        hold_y = start_pose.position.y
        start_z = start_pose.position.z
        if start_z >= self.target_height:
            rospy.logwarn(
                "Current z=%.2f m is already above target %.2f m; holding current x/y at target z",
                start_z,
                self.target_height,
            )
        yaw = yaw_from_quaternion(start_pose.orientation)
        trajectory_id = int(start.to_nsec() % 1000000000)

        rospy.loginfo(
            "Start climbing from z=%.2f m to %.2f m at %.2f m/s. Hold x=%.2f y=%.2f",
            start_z,
            self.target_height,
            self.speed,
            hold_x,
            hold_y,
        )

        reached_target = False
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            target_z = min(start_z + self.speed * elapsed, self.target_height)
            vz = 0.0 if target_z >= self.target_height else self.speed
            if target_z >= self.target_height and not reached_target:
                reached_target = True
                rospy.loginfo(
                    "Reached target height command %.2f m, keep hovering there",
                    self.target_height,
                )

            self.cmd_pub.publish(self.make_command(hold_x, hold_y, target_z, vz, yaw, trajectory_id))
            rospy.loginfo_throttle(
                0.5,
                "px4ctrl command: target_z=%.2f current_z=%.2f vz=%.2f",
                target_z,
                self.odom.pose.pose.position.z if self.odom else float("nan"),
                vz,
            )

            self.rate.sleep()


def main():
    args = parse_args()
    if args.target_height is None:
        args.target_height = prompt_target_height(args.default_height)
    rospy.init_node("px4ctrl_climb_forever")
    try:
        Px4ctrlClimbForever(args).run()
    except RuntimeError as exc:
        rospy.logerr("%s", exc)


if __name__ == "__main__":
    main()
