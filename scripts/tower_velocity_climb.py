#!/usr/bin/env python3

import argparse
import math

import rospy
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Publish XTDrone velocity commands for tower climbing. "
            "This node is intentionally topic-only and does not modify or import "
            "the existing mapping/control code."
        )
    )
    parser.add_argument("--vehicle", default="iris_0")
    parser.add_argument("--odom-topic", default="/wtb/odom")
    parser.add_argument("--control-point-topic", default="/wtb/tower_detector/control_point")
    parser.add_argument("--axis-topic", default="/wtb/tower_detector/axis_direction")
    parser.add_argument("--vel-topic", default=None)
    parser.add_argument("--frame", choices=["auto", "enu", "flu"], default="auto")

    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--ascend-speed", type=float, default=1.0)
    parser.add_argument("--distance-deadband", type=float, default=0.3)
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.0,
        help="Hard safety distance to the tower control point. Disabled when <= 0.",
    )
    parser.add_argument(
        "--surface-distance",
        type=float,
        default=10.0,
        help="Reference horizontal distance from drone to tower surface/control point, in meters.",
    )

    parser.add_argument("--kp", type=float, default=0.35)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.08)
    parser.add_argument("--integral-limit", type=float, default=3.0)
    parser.add_argument("--max-horizontal-speed", type=float, default=0.5)
    parser.add_argument("--max-vertical-speed", type=float, default=1.0)
    parser.add_argument("--max-total-speed", type=float, default=1.2)

    parser.add_argument("--yaw-kp", type=float, default=1.2)
    parser.add_argument("--max-yaw-rate", type=float, default=0.6)
    parser.add_argument("--no-yaw-control", action="store_true")

    parser.add_argument("--target-height", type=float, default=None)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument(
        "--allow-stale-control-point",
        action="store_true",
        help="Keep using the last tower control point when the detector temporarily stops publishing.",
    )
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-zero-on-exit", action="store_true")
    return parser.parse_args()


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_xy(x, y):
    norm = math.hypot(x, y)
    if norm < 1e-6:
        return None, None, 0.0
    return x / norm, y / norm, norm


def limit_xy(vx, vy, limit):
    speed = math.hypot(vx, vy)
    if speed > limit > 0.0:
        scale = limit / speed
        return vx * scale, vy * scale
    return vx, vy


def limit_xyz(vx, vy, vz, limit):
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed > limit > 0.0:
        scale = limit / speed
        return vx * scale, vy * scale, vz * scale
    return vx, vy, vz


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            derivative = 0.0
        elif self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt

        self.integral += error * max(dt, 0.0)
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class TowerVelocityClimb:
    def __init__(self, args):
        self.args = args
        self.rate = rospy.Rate(args.rate)
        self.vel_topic = args.vel_topic or "/xtdrone/{}/cmd_vel_enu".format(args.vehicle)
        self.frame = self.resolve_frame(args.frame, self.vel_topic)

        self.odom = None
        self.odom_time = rospy.Time(0)
        self.control_point = None
        self.control_point_time = rospy.Time(0)
        self.axis = (0.0, 0.0, 1.0)
        self.axis_time = rospy.Time(0)

        self.pid = PID(args.kp, args.ki, args.kd, args.integral_limit)
        self.last_update = None

        self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
        self.control_point_sub = rospy.Subscriber(
            args.control_point_topic, PointStamped, self.control_point_cb, queue_size=1
        )
        self.axis_sub = rospy.Subscriber(args.axis_topic, Vector3Stamped, self.axis_cb, queue_size=1)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

    @staticmethod
    def resolve_frame(frame_arg, vel_topic):
        if frame_arg != "auto":
            return frame_arg
        if vel_topic.endswith("_flu") or "/cmd_vel_flu" in vel_topic:
            return "flu"
        return "enu"

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_time = rospy.Time.now()

    def control_point_cb(self, msg):
        self.control_point = msg
        self.control_point_time = rospy.Time.now()

    def axis_cb(self, msg):
        ax = msg.vector.x
        ay = msg.vector.y
        az = msg.vector.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm
        if az < 0.0:
            ax, ay, az = -ax, -ay, -az
        self.axis = (ax, ay, az)
        self.axis_time = rospy.Time.now()

    def wait_for_inputs(self):
        rospy.loginfo("Waiting for odom: %s", self.args.odom_topic)
        rospy.loginfo("Waiting for tower control point: %s", self.args.control_point_topic)
        while not rospy.is_shutdown():
            if self.odom is not None and self.control_point is not None:
                return
            self.rate.sleep()

    def wait_for_subscriber(self):
        if self.args.subscriber_timeout <= 0.0 or self.args.dry_run:
            return
        rospy.loginfo("Waiting for velocity subscriber: %s", self.vel_topic)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.vel_pub.get_num_connections() > 0:
                return
            if (rospy.Time.now() - start).to_sec() > self.args.subscriber_timeout:
                rospy.logwarn(
                    "No subscriber on %s after %.1fs; publishing anyway",
                    self.vel_topic,
                    self.args.subscriber_timeout,
                )
                return
            self.rate.sleep()

    def reference_distance(self):
        return self.args.surface_distance

    def inputs_are_fresh(self, now):
        odom_age = (now - self.odom_time).to_sec()
        cp_age = (now - self.control_point_time).to_sec()
        if odom_age > self.args.stale_timeout:
            rospy.logwarn_throttle(
                1.0,
                "Input stale: odom %.2fs, control_point %.2fs; sending zero velocity",
                odom_age,
                cp_age,
            )
            return False
        if cp_age > self.args.stale_timeout:
            if self.args.allow_stale_control_point:
                rospy.logwarn_throttle(
                    1.0,
                    "Control point stale %.2fs; reusing last tower XY",
                    cp_age,
                )
                return True
            rospy.logwarn_throttle(
                1.0,
                "Input stale: odom %.2fs, control_point %.2fs; sending zero velocity",
                odom_age,
                cp_age,
            )
            return False
        return True

    def zero_twist(self):
        return Twist()

    def publish_twist(self, twist):
        if self.args.dry_run:
            return
        self.vel_pub.publish(twist)

    def make_command(self, now):
        pose = self.odom.pose.pose
        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z
        cp = self.control_point.point

        ut_x, ut_y, distance = normalize_xy(cp.x - px, cp.y - py)
        if ut_x is None:
            rospy.logwarn_throttle(1.0, "Drone is too close to tower control point projection")
            return self.zero_twist(), 0.0, 0.0, 0.0

        ref_distance = self.reference_distance()
        error = ref_distance - distance
        if self.last_update is None:
            dt = 0.0
        else:
            dt = (now - self.last_update).to_sec()
        self.last_update = now

        if abs(error) <= abs(self.args.distance_deadband):
            pid_output = 0.0
        else:
            pid_output = self.pid.update(error, dt)

        too_close = self.args.min_distance > 0.0 and distance <= self.args.min_distance
        if too_close:
            pid_output = max(abs(pid_output), self.args.max_horizontal_speed)

        vh_x = pid_output * (-ut_x)
        vh_y = pid_output * (-ut_y)
        vh_x, vh_y = limit_xy(vh_x, vh_y, self.args.max_horizontal_speed)

        ax, ay, az = self.axis
        vz_limit = abs(self.args.max_vertical_speed)
        ascend_speed = clamp(abs(self.args.ascend_speed), 0.0, vz_limit)
        v_x = vh_x + ascend_speed * ax
        v_y = vh_y + ascend_speed * ay
        v_z = 0.0 if too_close else ascend_speed * az
        v_x, v_y, v_z = limit_xyz(v_x, v_y, v_z, self.args.max_total_speed)

        yaw_rate = 0.0
        if not self.args.no_yaw_control:
            yaw = yaw_from_quaternion(pose.orientation)
            target_yaw = math.atan2(ut_y, ut_x)
            yaw_error = wrap_angle(target_yaw - yaw)
            yaw_rate = clamp(
                self.args.yaw_kp * yaw_error,
                -abs(self.args.max_yaw_rate),
                abs(self.args.max_yaw_rate),
            )

        if self.frame == "flu":
            yaw = yaw_from_quaternion(pose.orientation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            body_x = cos_yaw * v_x + sin_yaw * v_y
            body_y = -sin_yaw * v_x + cos_yaw * v_y
            out_x, out_y = body_x, body_y
        else:
            out_x, out_y = v_x, v_y

        twist = Twist()
        twist.linear.x = out_x
        twist.linear.y = out_y
        twist.linear.z = v_z
        twist.angular.z = yaw_rate
        return twist, distance, ref_distance, error

    def publish_zero_burst(self):
        if self.args.no_zero_on_exit or self.args.dry_run:
            return
        zero = self.zero_twist()
        for _ in range(10):
            self.vel_pub.publish(zero)
            rospy.sleep(0.03)

    def run(self):
        rospy.loginfo("Tower velocity climb output: %s (%s frame)", self.vel_topic, self.frame)
        self.wait_for_inputs()
        self.wait_for_subscriber()

        start = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                if self.args.duration > 0.0 and (now - start).to_sec() >= self.args.duration:
                    rospy.loginfo("Duration reached; sending zero velocity and exiting")
                    break

                if self.args.target_height is not None and self.odom is not None:
                    if self.odom.pose.pose.position.z >= self.args.target_height:
                        rospy.loginfo("Target height %.2f reached; sending zero velocity and exiting",
                                      self.args.target_height)
                        break

                if not self.inputs_are_fresh(now):
                    self.publish_twist(self.zero_twist())
                    self.rate.sleep()
                    continue

                twist, distance, ref_distance, error = self.make_command(now)
                self.publish_twist(twist)
                rospy.loginfo_throttle(
                    0.5,
                    "tower vel %s: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f surface_dist=%.2f surface_ref=%.2f err=%.2f",
                    self.frame,
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                    twist.angular.z,
                    distance,
                    ref_distance,
                    error,
                )
                self.rate.sleep()
        finally:
            self.publish_zero_burst()


def main():
    args = parse_args()
    rospy.init_node("tower_velocity_climb")
    TowerVelocityClimb(args).run()


if __name__ == "__main__":
    main()
