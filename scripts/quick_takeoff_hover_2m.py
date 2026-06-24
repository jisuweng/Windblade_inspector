#!/usr/bin/env python3

import argparse

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser(
        description="XTDrone-style quick takeoff, then hover near the target height."
    )
    parser.add_argument("vehicle_type", nargs="?", default="iris")
    parser.add_argument("vehicle_id", nargs="?", default="0")
    parser.add_argument("--height", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--tolerance", type=float, default=0.08)
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser.parse_args()


class QuickTakeoffHover:
    def __init__(self, args):
        self.vehicle = "{}_{}".format(args.vehicle_type, args.vehicle_id)
        self.height = args.height
        self.max_speed = abs(args.speed)
        self.tolerance = args.tolerance
        self.timeout = args.timeout
        self.pose = None

        self.pose_topic = "/{}/mavros/local_position/pose".format(self.vehicle)
        self.cmd_topic = "/xtdrone/{}/cmd".format(self.vehicle)
        self.vel_topic = "/xtdrone/{}/cmd_vel_flu".format(self.vehicle)

        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=1
        )
        self.cmd_pub = rospy.Publisher(self.cmd_topic, String, queue_size=3)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(30)

    def pose_callback(self, msg):
        self.pose = msg

    def altitude(self):
        return self.pose.pose.position.z

    def wait_for_pose(self):
        rospy.loginfo("Waiting for pose: %s", self.pose_topic)
        try:
            self.pose = rospy.wait_for_message(self.pose_topic, PoseStamped, timeout=8.0)
        except rospy.ROSException:
            raise RuntimeError("No pose received from {}".format(self.pose_topic))

    def wait_for_bridge(self):
        rospy.loginfo("Waiting for XTDrone command subscribers")
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.cmd_pub.get_num_connections() > 0 and self.vel_pub.get_num_connections() > 0:
                return
            if (rospy.Time.now() - start).to_sec() > 8.0:
                raise RuntimeError(
                    "No subscribers on {} or {}. Start multirotor_communication.py first.".format(
                        self.cmd_topic, self.vel_topic
                    )
                )
            self.rate.sleep()

    def keyboard_publish(self, twist, cmd=""):
        self.vel_pub.publish(twist)
        self.cmd_pub.publish(String(data=cmd))

    def publish_cmd(self, text, twist=None):
        if twist is None:
            twist = Twist()
        self.keyboard_publish(twist, text)
        rospy.loginfo("Sent command: %s", text)

    def spin_like_keyboard(self, seconds, twist=None):
        if twist is None:
            twist = Twist()
        end = rospy.Time.now() + rospy.Duration(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self.keyboard_publish(twist, "")
            self.rate.sleep()

    def climb_twist(self):
        twist = Twist()
        twist.linear.z = self.max_speed
        return twist

    def climb_to_height_with_velocity(self):
        rospy.loginfo("Climbing to %.2f m with upward speed %.2f m/s", self.height, self.max_speed)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.altitude() >= self.height - self.tolerance:
                return

            if (rospy.Time.now() - start).to_sec() > self.timeout:
                raise RuntimeError(
                    "Timeout: current altitude is {:.2f} m, target is {:.2f} m".format(
                        self.altitude(), self.height
                    )
                )

            self.keyboard_publish(self.climb_twist(), "")
            rospy.loginfo_throttle(
                0.5,
                "altitude %.2f m -> %.2f m, keyboard vz %.2f",
                self.altitude(),
                self.height,
                self.max_speed,
            )
            self.rate.sleep()

    def run(self):
        self.wait_for_pose()
        self.wait_for_bridge()

        climb = self.climb_twist()
        rospy.loginfo("Set upward speed first: %.2f m/s", self.max_speed)
        self.spin_like_keyboard(1.0, climb)
        self.publish_cmd("OFFBOARD", climb)
        self.spin_like_keyboard(0.5, climb)
        self.publish_cmd("ARM", climb)
        self.spin_like_keyboard(0.3, climb)
        self.climb_to_height_with_velocity()

        self.spin_like_keyboard(0.3)
        self.publish_cmd("OFFBOARD")
        self.spin_like_keyboard(0.3)
        self.publish_cmd("HOVER")
        self.spin_like_keyboard(1.0)
        rospy.loginfo("Done: hovering near %.2f m", self.height)


def main():
    args = parse_args()
    rospy.init_node("quick_takeoff_hover_2m")
    try:
        QuickTakeoffHover(args).run()
    except RuntimeError as exc:
        rospy.logerr("%s", exc)


if __name__ == "__main__":
    main()
