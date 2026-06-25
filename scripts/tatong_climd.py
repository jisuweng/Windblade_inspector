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
    parser.add_argument("--wait-timeout", type=float, default=30.0)
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
        self.frame = self._resolve_frame(args.frame, self.vel_topic)

        self.odom = None
        self.odom_time = rospy.Time(0)
        self.control_point = None
        self.control_point_time = rospy.Time(0)
        self.axis = None
        self.axis_time = rospy.Time(0)
        self._warned_default_axis = False

        self.pid = PID(args.kp, args.ki, args.kd, args.integral_limit)
        self._last_update = None
        self._dt_fallback = 1.0 / max(args.rate, 1.0)

        self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.control_point_sub = rospy.Subscriber(
            args.control_point_topic, PointStamped, self._control_point_cb, queue_size=1
        )
        self.axis_sub = rospy.Subscriber(args.axis_topic, Vector3Stamped, self._axis_cb, queue_size=1)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_frame(frame_arg, vel_topic):
        if frame_arg != "auto":
            return frame_arg
        if vel_topic.endswith("_flu") or "/cmd_vel_flu" in vel_topic:
            return "flu"
        return "enu"

    @staticmethod
    def _zero_twist():
        return Twist()

    def _dt(self, now):
        if self._last_update is None:
            dt = 0.0
        else:
            dt = (now - self._last_update).to_sec()
            if dt <= 0.0:
                dt = self._dt_fallback
        self._last_update = now
        return dt

    # ------------------------------------------------------------------
    # subscribers
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        self.odom = msg
        self.odom_time = rospy.Time.now()

    def _control_point_cb(self, msg):
        self.control_point = msg
        self.control_point_time = rospy.Time.now()

    def _axis_cb(self, msg):
        ax, ay, az = msg.vector.x, msg.vector.y, msg.vector.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        if az < 0.0:
            ax, ay, az = -ax, -ay, -az
        self.axis = (ax / norm, ay / norm, az / norm)
        self.axis_time = rospy.Time.now()

    # ------------------------------------------------------------------
    # initialisation waits
    # ------------------------------------------------------------------

    def _wait_for_inputs(self):
        rospy.loginfo("Waiting for odom: %s", self.args.odom_topic)
        rospy.loginfo("Waiting for tower control point: %s", self.args.control_point_topic)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.odom is not None and self.control_point is not None:
                return True
            if self.args.wait_timeout > 0:
                elapsed = (rospy.Time.now() - start).to_sec()
                if elapsed > self.args.wait_timeout:
                    rospy.logerr(
                        "Timed out waiting for inputs after %.1fs (odom=%s cp=%s)",
                        elapsed,
                        "ok" if self.odom is not None else "missing",
                        "ok" if self.control_point is not None else "missing",
                    )
                    return False
            self.rate.sleep()
        return False

    def _wait_for_subscriber(self):
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

    # ------------------------------------------------------------------
    # freshness check (pure return, no side-effects)
    # ------------------------------------------------------------------

    def _check_freshness(self, now):
        odom_age = (now - self.odom_time).to_sec()
        cp_age   = (now - self.control_point_time).to_sec()
        if odom_age > self.args.stale_timeout:
            return False, "stale_odom"
        if cp_age > self.args.stale_timeout:
            if self.args.allow_stale_control_point:
                return True, "stale_cp_reusing"
            return False, "stale_cp"
        return True, "ok"

    # ------------------------------------------------------------------
    # axis access (lazy default with one-shot warning)
    # ------------------------------------------------------------------

    def _get_axis(self):
        if self.axis is None:
            if not self._warned_default_axis:
                rospy.logwarn(
                    "Axis topic %s has not published yet; defaulting to (0,0,1) upward climb. "
                    "Verify your axis topic is correct.",
                    self.args.axis_topic,
                )
                self._warned_default_axis = True
            return (0.0, 0.0, 1.0)
        return self.axis

    # ------------------------------------------------------------------
    # distance error
    # ------------------------------------------------------------------

    def _compute_distance_error(self, pose, control_point):
        """Return (unit_x, unit_y, distance, error). unit_* may be None."""
        px, py = pose.position.x, pose.position.y
        cpx, cpy = control_point.x, control_point.y
        ut_x, ut_y, distance = normalize_xy(cpx - px, cpy - py)
        if ut_x is None:
            return None, None, 0.0, 0.0
        error = self.args.surface_distance - distance
        return ut_x, ut_y, distance, error

    # ------------------------------------------------------------------
    # horizontal velocity (PID-controlled approach / retreat)
    # ------------------------------------------------------------------

    def _compute_horizontal_velocity(self, error, ut_x, ut_y, dt, too_close):
        if abs(error) <= abs(self.args.distance_deadband):
            return 0.0, 0.0

        pid_output = self.pid.update(error, dt)

        if too_close:
            pid_output = -abs(self.args.max_horizontal_speed)
        else:
            pid_output = clamp(pid_output, -abs(self.args.max_horizontal_speed),
                               abs(self.args.max_horizontal_speed))

        vh_x = pid_output * (-ut_x)
        vh_y = pid_output * (-ut_y)
        return vh_x, vh_y

    # ------------------------------------------------------------------
    # vertical velocity along tower axis
    # ------------------------------------------------------------------

    def _compute_vertical_velocity(self, ax, ay, az, too_close):
        ascend_speed = clamp(abs(self.args.ascend_speed), 0.0,
                             abs(self.args.max_vertical_speed))
        if too_close:
            return ascend_speed * ax, ascend_speed * ay, 0.0
        return ascend_speed * ax, ascend_speed * ay, ascend_speed * az

    # ------------------------------------------------------------------
    # yaw rate
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_yaw_rate(pose, ut_x, ut_y, yaw_kp, max_yaw_rate):
        yaw = yaw_from_quaternion(pose.orientation)
        target_yaw = math.atan2(ut_y, ut_x)
        yaw_error = wrap_angle(target_yaw - yaw)
        return clamp(yaw_kp * yaw_error, -abs(max_yaw_rate), abs(max_yaw_rate))

    # ------------------------------------------------------------------
    # frame rotation  ENU → body-FLU
    # ------------------------------------------------------------------

    @staticmethod
    def _to_body_frame(vx, vy, pose):
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return cos_yaw * vx + sin_yaw * vy, -sin_yaw * vx + cos_yaw * vy

    # ------------------------------------------------------------------
    # command assembly
    # ------------------------------------------------------------------

    def make_command(self, now):
        pose = self.odom.pose.pose

        ut_x, ut_y, distance, error = self._compute_distance_error(pose, self.control_point.point)
        if ut_x is None:
            rospy.logwarn_throttle(1.0, "Drone is too close to tower control point projection")
            return self._zero_twist(), 0.0, 0.0, 0.0

        too_close = self.args.min_distance > 0.0 and distance <= self.args.min_distance
        dt = self._dt(now)

        vh_x, vh_y = self._compute_horizontal_velocity(error, ut_x, ut_y, dt, too_close)

        ax, ay, az = self._get_axis()
        vax, vay, vaz = self._compute_vertical_velocity(ax, ay, az, too_close)

        vx = vh_x + vax
        vy = vh_y + vay
        vz = vaz
        vx, vy, vz = limit_xyz(vx, vy, vz, self.args.max_total_speed)

        yaw_rate = 0.0
        if not self.args.no_yaw_control:
            yaw_rate = self._compute_yaw_rate(pose, ut_x, ut_y,
                                              self.args.yaw_kp, self.args.max_yaw_rate)

        if self.frame == "flu":
            out_x, out_y = self._to_body_frame(vx, vy, pose)
        else:
            out_x, out_y = vx, vy

        twist = Twist()
        twist.linear.x = out_x
        twist.linear.y = out_y
        twist.linear.z = vz
        twist.angular.z = yaw_rate
        return twist, distance, self.args.surface_distance, error

    # ------------------------------------------------------------------
    # publish helpers
    # ------------------------------------------------------------------

    def _publish(self, twist):
        if not self.args.dry_run:
            self.vel_pub.publish(twist)

    def _publish_zero_burst(self):
        if self.args.no_zero_on_exit or self.args.dry_run:
            return
        zero = self._zero_twist()
        for _ in range(10):
            self.vel_pub.publish(zero)
            rospy.sleep(0.03)

    # ------------------------------------------------------------------
    # termination checks
    # ------------------------------------------------------------------

    def _should_terminate(self, start_time):
        now = rospy.Time.now()
        if self.args.duration > 0.0 and (now - start_time).to_sec() >= self.args.duration:
            rospy.loginfo("Duration reached; sending zero velocity and exiting")
            return True
        if self.args.target_height is not None and self.odom is not None:
            if self.odom.pose.pose.position.z >= self.args.target_height:
                rospy.loginfo("Target height %.2f reached; sending zero velocity and exiting",
                              self.args.target_height)
                return True
        return False

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------

    def run(self):
        rospy.loginfo("Tower velocity climb output: %s (%s frame)", self.vel_topic, self.frame)
        if not self._wait_for_inputs():
            return
        self._wait_for_subscriber()

        start = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()

                if self._should_terminate(start):
                    break

                fresh, reason = self._check_freshness(now)
                if not fresh:
                    self._log_freshness(reason, now)
                    self._publish(self._zero_twist())
                    self.rate.sleep()
                    continue
                if reason == "stale_cp_reusing":
                    rospy.logwarn_throttle(
                        1.0, "Control point stale; reusing last tower XY")

                twist, distance, ref_dist, error = self.make_command(now)
                self._publish(twist)
                rospy.loginfo_throttle(
                    5.0,
                    "tower vel %s: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f "
                    "dist=%.2f ref=%.2f err=%.2f",
                    self.frame,
                    twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.z,
                    distance, ref_dist, error,
                )
                self.rate.sleep()
        finally:
            self._publish_zero_burst()

    def _log_freshness(self, reason, now):
        odom_age = (now - self.odom_time).to_sec()
        cp_age   = (now - self.control_point_time).to_sec()
        if reason == "stale_odom":
            rospy.logwarn_throttle(
                1.0, "Input stale: odom %.2fs; sending zero velocity", odom_age)
        elif reason == "stale_cp":
            rospy.logwarn_throttle(
                1.0, "Input stale: control_point %.2fs; sending zero velocity", cp_age)


def main():
    args = parse_args()
    rospy.init_node("tatong_climd")
    TowerVelocityClimb(args).run()


if __name__ == "__main__":
    main()
