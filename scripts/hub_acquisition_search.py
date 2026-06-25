#!/usr/bin/env python3

import argparse
import math
import random
import struct
from collections import OrderedDict

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Search around the tower top after tower control points stop, "
            "accumulate non-tower points, and fit a coarse rotor plane."
        )
    )
    parser.add_argument("--vehicle", default="iris_0")
    parser.add_argument("--odom-topic", default="/wtb/odom")
    parser.add_argument("--control-point-topic", default="/wtb/tower_detector/control_point")
    parser.add_argument("--radius-topic", default="/wtb/tower_detector/radius")
    parser.add_argument("--cloud-topic", default="/wtb/current_cloud_world")
    parser.add_argument("--vel-topic", default=None)
    parser.add_argument("--frame", choices=["auto", "enu", "flu"], default="auto")
    parser.add_argument("--world-frame", default="map")

    parser.add_argument("--param-namespace", default="/wtb/tower_detector")
    parser.add_argument("--legacy-param-namespace", default="/tower")
    parser.add_argument("--default-tower-radius", type=float, default=1.0)

    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--search-radius", type=float, default=10.0)
    parser.add_argument("--height-offset", type=float, default=4.0)
    parser.add_argument("--sweep-angle-deg", type=float, default=25.0)
    parser.add_argument("--sweep-speed-deg", type=float, default=4.0)
    parser.add_argument("--target-deadband", type=float, default=0.5)
    parser.add_argument("--height-deadband", type=float, default=0.5)

    parser.add_argument("--kp-xy", type=float, default=0.35)
    parser.add_argument("--kp-z", type=float, default=0.35)
    parser.add_argument("--yaw-kp", type=float, default=1.2)
    parser.add_argument("--max-horizontal-speed", type=float, default=0.4)
    parser.add_argument("--max-vertical-speed", type=float, default=0.4)
    parser.add_argument("--max-total-speed", type=float, default=0.65)
    parser.add_argument("--max-yaw-rate", type=float, default=0.6)

    parser.add_argument("--tower-margin", type=float, default=0.8)
    parser.add_argument("--roi-below", type=float, default=3.0)
    parser.add_argument("--roi-above", type=float, default=8.0)
    parser.add_argument("--roi-radius", type=float, default=22.0)
    parser.add_argument("--cloud-sample-limit", type=int, default=6000)
    parser.add_argument("--voxel-size", type=float, default=0.25)
    parser.add_argument("--max-buffer-points", type=int, default=20000)
    parser.add_argument("--min-non-tower-points", type=int, default=300)

    parser.add_argument("--ransac-iterations", type=int, default=200)
    parser.add_argument("--plane-distance-threshold", type=float, default=0.7)
    parser.add_argument("--plane-min-inliers", type=int, default=120)
    parser.add_argument("--plane-normal-z-max", type=float, default=0.45)
    parser.add_argument("--stable-angle-deg", type=float, default=12.0)
    parser.add_argument("--stable-required", type=int, default=3)
    parser.add_argument("--fit-rate", type=float, default=2.0)

    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--continue-after-lock", action="store_true")
    parser.add_argument("--no-zero-on-exit", action="store_true")
    return parser.parse_args()


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def point_msg(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def color(r, g, b, a):
    c = type("Color", (), {})()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


class HubAcquisitionSearch:
    def __init__(self, args):
        self.args = args
        self.rate = rospy.Rate(max(args.rate, 1.0))
        self.vel_topic = args.vel_topic or "/xtdrone/{}/cmd_vel_enu".format(args.vehicle)
        self.frame = self.resolve_frame(args.frame, self.vel_topic)

        self.odom = None
        self.odom_time = rospy.Time(0)
        self.last_control_point = None
        self.control_point_time = rospy.Time(0)
        self.tower_radius = max(args.default_tower_radius, 0.01)

        self.reference_locked = False
        self.tower_center = None
        self.last_control_z = None
        self.target_z = None
        self.center_angle = 0.0
        self.sweep_start_time = None
        self.locked_angle = None

        self.buffer = OrderedDict()
        self.last_added_points = 0
        self.latest_plane = None
        self.last_plane_normal = None
        self.stable_count = 0
        self.plane_locked = False
        self.last_fit_time = rospy.Time(0)
        self.current_target = None
        self.phase = "waiting"

        self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
        self.control_sub = rospy.Subscriber(
            args.control_point_topic, PointStamped, self.control_point_cb, queue_size=1
        )
        self.radius_sub = rospy.Subscriber(args.radius_topic, Float64, self.radius_cb, queue_size=1)
        self.cloud_sub = rospy.Subscriber(
            args.cloud_topic,
            PointCloud2,
            self.cloud_cb,
            queue_size=1,
            buff_size=100 * 1024 * 1024,
        )

        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/wtb/hub_acquisition/status", String, queue_size=1)
        self.normal_pub = rospy.Publisher(
            "/wtb/hub_acquisition/rotor_normal", Vector3Stamped, queue_size=1
        )
        self.target_pub = rospy.Publisher(
            "/wtb/hub_acquisition/search_target", PointStamped, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/wtb/hub_acquisition/markers", MarkerArray, queue_size=1
        )
        self.cloud_pub = rospy.Publisher(
            "/wtb/hub_acquisition/non_tower_cloud", PointCloud2, queue_size=1
        )

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
        self.last_control_point = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=float
        )
        self.control_point_time = rospy.Time.now()

    def radius_cb(self, msg):
        if math.isfinite(msg.data) and msg.data > 0.0:
            self.tower_radius = msg.data

    def clean_namespace(self, namespace):
        if not namespace:
            return ""
        return "/" + namespace.strip("/")

    def load_reference_from_params(self):
        for namespace in [self.args.param_namespace, self.args.legacy_param_namespace]:
            ns = self.clean_namespace(namespace)
            if not ns:
                continue
            required = [ns + "/control_x", ns + "/control_y", ns + "/control_z"]
            if not all(rospy.has_param(name) for name in required):
                continue
            try:
                cp = np.array(
                    [
                        float(rospy.get_param(required[0])),
                        float(rospy.get_param(required[1])),
                        float(rospy.get_param(required[2])),
                    ],
                    dtype=float,
                )
            except (TypeError, ValueError):
                continue
            if not np.all(np.isfinite(cp)):
                continue
            self.last_control_point = cp
            self.control_point_time = rospy.Time.now()
            if rospy.has_param(ns + "/radius"):
                try:
                    radius = float(rospy.get_param(ns + "/radius"))
                    if math.isfinite(radius) and radius > 0.0:
                        self.tower_radius = radius
                except (TypeError, ValueError):
                    pass
            rospy.loginfo("Loaded tower reference from params: %s", ns)
            return True
        return False

    def ensure_reference(self):
        if self.reference_locked:
            return True
        if self.last_control_point is None:
            self.load_reference_from_params()
        if self.last_control_point is None or self.odom is None:
            return False

        pose = self.odom.pose.pose.position
        cp = self.last_control_point
        self.tower_center = np.array([cp[0], cp[1]], dtype=float)
        self.last_control_z = float(cp[2])
        self.target_z = self.last_control_z + self.args.height_offset

        radial_x = pose.x - self.tower_center[0]
        radial_y = pose.y - self.tower_center[1]
        if math.hypot(radial_x, radial_y) < 1e-3:
            self.center_angle = 0.0
        else:
            self.center_angle = math.atan2(radial_y, radial_x)

        self.sweep_start_time = rospy.Time.now()
        self.reference_locked = True
        rospy.loginfo(
            "Hub acquisition reference locked: center=(%.2f, %.2f), z_last=%.2f, target_z=%.2f, radius=%.2f",
            self.tower_center[0],
            self.tower_center[1],
            self.last_control_z,
            self.target_z,
            self.tower_radius,
        )
        return True

    def cloud_cb(self, msg):
        if not self.ensure_reference():
            return

        try:
            raw_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except (AssertionError, struct.error, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Failed to read PointCloud2: %s", exc)
            return
        if not raw_points:
            self.last_added_points = 0
            return

        if len(raw_points) > self.args.cloud_sample_limit > 0:
            raw_points = random.sample(raw_points, self.args.cloud_sample_limit)

        pts = np.asarray(raw_points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] < 3:
            self.last_added_points = 0
            return
        pts = pts[:, :3]
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if pts.size == 0:
            self.last_added_points = 0
            return

        dx = pts[:, 0] - self.tower_center[0]
        dy = pts[:, 1] - self.tower_center[1]
        radial = np.sqrt(dx * dx + dy * dy)
        z_min = self.last_control_z - abs(self.args.roi_below)
        z_max = self.target_z + abs(self.args.roi_above)
        tower_cutoff = max(self.tower_radius + self.args.tower_margin, 0.05)

        mask = (
            (pts[:, 2] >= z_min)
            & (pts[:, 2] <= z_max)
            & (radial >= tower_cutoff)
            & (radial <= max(self.args.roi_radius, self.args.search_radius + 2.0))
        )
        kept = pts[mask]
        self.last_added_points = int(len(kept))
        if len(kept) == 0:
            return

        voxel = max(self.args.voxel_size, 1e-3)
        for p in kept:
            key = tuple(np.floor(p / voxel).astype(int).tolist())
            self.buffer[key] = (float(p[0]), float(p[1]), float(p[2]))
            self.buffer.move_to_end(key)

        while len(self.buffer) > max(self.args.max_buffer_points, 1):
            self.buffer.popitem(last=False)

    def buffer_points_array(self):
        if not self.buffer:
            return np.empty((0, 3), dtype=float)
        return np.asarray(list(self.buffer.values()), dtype=float)

    def fit_plane_ransac(self, points):
        if len(points) < self.args.min_non_tower_points:
            return None

        best_mask = None
        best_count = 0
        n_points = len(points)
        threshold = max(self.args.plane_distance_threshold, 1e-3)
        normal_z_max = clamp(abs(self.args.plane_normal_z_max), 0.0, 1.0)

        for _ in range(max(self.args.ransac_iterations, 1)):
            ids = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = points[ids]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            if abs(normal[2]) > normal_z_max:
                continue
            d = -float(np.dot(normal, p1))
            distances = np.abs(points.dot(normal) + d)
            mask = distances <= threshold
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count = count
                best_mask = mask

        if best_mask is None or best_count < self.args.plane_min_inliers:
            return None

        inliers = points[best_mask]
        centroid = np.mean(inliers, axis=0)
        centered = inliers - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        normal = vh[-1]
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            return None
        normal = normal / norm
        if abs(normal[2]) > normal_z_max:
            return None

        if self.odom is not None and self.tower_center is not None and self.target_z is not None:
            pose = self.odom.pose.pose.position
            drone = np.array([pose.x, pose.y, pose.z], dtype=float)
            hub_ref = np.array([self.tower_center[0], self.tower_center[1], self.target_z])
            if np.dot(normal, drone - hub_ref) < 0.0:
                normal = -normal

        distances = np.abs((points - centroid).dot(normal))
        refined_mask = distances <= threshold
        refined_count = int(np.count_nonzero(refined_mask))
        return {
            "normal": normal,
            "centroid": centroid,
            "inliers": refined_count,
            "total": len(points),
        }

    def update_plane_fit(self, now):
        if (now - self.last_fit_time).to_sec() < 1.0 / max(self.args.fit_rate, 0.1):
            return
        self.last_fit_time = now

        points = self.buffer_points_array()
        plane = self.fit_plane_ransac(points)
        if plane is None:
            self.latest_plane = None
            self.stable_count = 0 if not self.plane_locked else self.stable_count
            return

        normal = plane["normal"]
        if self.last_plane_normal is None:
            self.stable_count = 1
        else:
            cos_angle = abs(float(np.dot(normal, self.last_plane_normal)))
            cos_limit = math.cos(math.radians(max(self.args.stable_angle_deg, 0.0)))
            if cos_angle >= cos_limit:
                self.stable_count += 1
            else:
                self.stable_count = 1

        self.last_plane_normal = normal
        self.latest_plane = plane
        if self.stable_count >= max(self.args.stable_required, 1):
            if not self.plane_locked:
                rospy.loginfo(
                    "Rotor plane locked: normal=(%.3f, %.3f, %.3f), inliers=%d/%d",
                    normal[0],
                    normal[1],
                    normal[2],
                    plane["inliers"],
                    plane["total"],
                )
            self.plane_locked = True

    def sweep_angle_offset(self, now):
        amp = math.radians(abs(self.args.sweep_angle_deg))
        if amp < 1e-6 or self.sweep_start_time is None:
            return 0.0
        speed = math.radians(abs(self.args.sweep_speed_deg))
        if speed < 1e-6:
            return 0.0
        elapsed = (now - self.sweep_start_time).to_sec()
        return amp * math.sin(elapsed * speed / amp)

    def make_target(self, now):
        pose = self.odom.pose.pose.position
        current_radius = math.hypot(pose.x - self.tower_center[0], pose.y - self.tower_center[1])
        radius_err = self.args.search_radius - current_radius
        height_err = self.target_z - pose.z

        near_shell = (
            abs(radius_err) <= self.args.target_deadband
            and abs(height_err) <= self.args.height_deadband
        )

        if self.plane_locked and not self.args.continue_after_lock:
            self.phase = "plane_locked"
            if self.locked_angle is None:
                self.locked_angle = math.atan2(
                    pose.y - self.tower_center[1], pose.x - self.tower_center[0]
                )
            target_angle = self.locked_angle
        elif near_shell:
            self.phase = "sweep"
            target_angle = self.center_angle + self.sweep_angle_offset(now)
        else:
            self.phase = "approach"
            target_angle = self.center_angle

        tx = self.tower_center[0] + self.args.search_radius * math.cos(target_angle)
        ty = self.tower_center[1] + self.args.search_radius * math.sin(target_angle)
        tz = self.target_z
        self.current_target = np.array([tx, ty, tz], dtype=float)
        return self.current_target, radius_err, height_err

    def make_command(self, now):
        target, radius_err, height_err = self.make_target(now)
        pose = self.odom.pose.pose
        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z

        err_x = target[0] - px
        err_y = target[1] - py
        err_z = target[2] - pz

        vx = self.args.kp_xy * err_x
        vy = self.args.kp_xy * err_y
        vz = self.args.kp_z * err_z
        vx, vy = limit_xy(vx, vy, abs(self.args.max_horizontal_speed))
        vz = clamp(vz, -abs(self.args.max_vertical_speed), abs(self.args.max_vertical_speed))
        vx, vy, vz = limit_xyz(vx, vy, vz, abs(self.args.max_total_speed))

        target_yaw = math.atan2(self.tower_center[1] - py, self.tower_center[0] - px)
        yaw = yaw_from_quaternion(pose.orientation)
        yaw_error = wrap_angle(target_yaw - yaw)
        yaw_rate = clamp(
            self.args.yaw_kp * yaw_error,
            -abs(self.args.max_yaw_rate),
            abs(self.args.max_yaw_rate),
        )

        if self.frame == "flu":
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            out_x = cos_yaw * vx + sin_yaw * vy
            out_y = -sin_yaw * vx + cos_yaw * vy
        else:
            out_x, out_y = vx, vy

        twist = Twist()
        twist.linear.x = out_x
        twist.linear.y = out_y
        twist.linear.z = vz
        twist.angular.z = yaw_rate
        return twist, radius_err, height_err

    def publish_twist(self, twist):
        if self.args.dry_run:
            return
        self.vel_pub.publish(twist)

    def publish_zero_burst(self):
        if self.args.no_zero_on_exit or self.args.dry_run:
            return
        zero = Twist()
        for _ in range(10):
            self.vel_pub.publish(zero)
            rospy.sleep(0.03)

    def publish_target(self, now):
        if self.current_target is None:
            return
        msg = PointStamped()
        msg.header.frame_id = self.args.world_frame
        msg.header.stamp = now
        msg.point = point_msg(*self.current_target)
        self.target_pub.publish(msg)

    def publish_normal(self, now):
        plane = self.latest_plane
        if plane is None and self.plane_locked and self.last_plane_normal is not None:
            normal = self.last_plane_normal
        elif plane is not None:
            normal = plane["normal"]
        else:
            return

        msg = Vector3Stamped()
        msg.header.frame_id = self.args.world_frame
        msg.header.stamp = now
        msg.vector.x = float(normal[0])
        msg.vector.y = float(normal[1])
        msg.vector.z = float(normal[2])
        self.normal_pub.publish(msg)

    def publish_cloud(self, now):
        points = list(self.buffer.values())
        if not points:
            return
        msg = pc2.create_cloud_xyz32(
            header=self.make_header(now),
            points=points,
        )
        self.cloud_pub.publish(msg)

    def make_header(self, now):
        from std_msgs.msg import Header

        header = Header()
        header.frame_id = self.args.world_frame
        header.stamp = now
        return header

    def make_marker(self, ns, marker_id, marker_type, now):
        marker = Marker()
        marker.header.frame_id = self.args.world_frame
        marker.header.stamp = now
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def apply_color(self, marker, rgba):
        marker.color.r = rgba.r
        marker.color.g = rgba.g
        marker.color.b = rgba.b
        marker.color.a = rgba.a

    def publish_markers(self, now):
        if not self.reference_locked:
            return
        markers = MarkerArray()

        hub_ref = self.make_marker("hub_acquisition", 0, Marker.SPHERE, now)
        hub_ref.pose.position = point_msg(self.tower_center[0], self.tower_center[1], self.target_z)
        hub_ref.scale.x = 0.55
        hub_ref.scale.y = 0.55
        hub_ref.scale.z = 0.55
        self.apply_color(hub_ref, color(1.0, 0.8, 0.1, 1.0))
        markers.markers.append(hub_ref)

        arc = self.make_marker("hub_acquisition", 1, Marker.LINE_STRIP, now)
        arc.scale.x = 0.08
        self.apply_color(arc, color(0.0, 0.8, 1.0, 0.9))
        amp = math.radians(abs(self.args.sweep_angle_deg))
        if amp < 1e-6:
            angles = [self.center_angle]
        else:
            angles = np.linspace(self.center_angle - amp, self.center_angle + amp, 60)
        for angle_value in angles:
            arc.points.append(
                point_msg(
                    self.tower_center[0] + self.args.search_radius * math.cos(angle_value),
                    self.tower_center[1] + self.args.search_radius * math.sin(angle_value),
                    self.target_z,
                )
            )
        markers.markers.append(arc)

        if self.current_target is not None:
            target = self.make_marker("hub_acquisition", 2, Marker.SPHERE, now)
            target.pose.position = point_msg(*self.current_target)
            target.scale.x = 0.45
            target.scale.y = 0.45
            target.scale.z = 0.45
            self.apply_color(target, color(0.1, 1.0, 0.2, 1.0))
            markers.markers.append(target)

        plane = self.latest_plane
        normal = None
        start = np.array([self.tower_center[0], self.tower_center[1], self.target_z], dtype=float)
        if plane is not None:
            normal = plane["normal"]
            start = plane["centroid"]
        elif self.plane_locked and self.last_plane_normal is not None:
            normal = self.last_plane_normal
        if normal is not None:
            arrow = self.make_marker("hub_acquisition", 3, Marker.ARROW, now)
            arrow.scale.x = 0.12
            arrow.scale.y = 0.35
            arrow.scale.z = 0.45
            self.apply_color(arrow, color(0.2, 0.4, 1.0, 1.0))
            arrow.points.append(point_msg(*start))
            arrow.points.append(point_msg(*(start + 5.0 * normal)))
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)

    def publish_status(self, radius_err, height_err):
        plane_inliers = 0
        plane_total = len(self.buffer)
        normal_text = "none"
        if self.latest_plane is not None:
            plane_inliers = self.latest_plane["inliers"]
            n = self.latest_plane["normal"]
            normal_text = "({:.3f},{:.3f},{:.3f})".format(n[0], n[1], n[2])
        elif self.plane_locked and self.last_plane_normal is not None:
            n = self.last_plane_normal
            normal_text = "({:.3f},{:.3f},{:.3f})".format(n[0], n[1], n[2])

        msg = String()
        msg.data = (
            "phase={phase}, dry_run={dry}, radius_err={radius_err:.2f}, "
            "height_err={height_err:.2f}, target_z={target_z:.2f}, "
            "tower_radius={tower_radius:.2f}, added_points={added}, "
            "buffer_points={buffer_points}, plane_inliers={inliers}/{total}, "
            "stable_count={stable}, locked={locked}, normal={normal}"
        ).format(
            phase=self.phase,
            dry=self.args.dry_run,
            radius_err=radius_err,
            height_err=height_err,
            target_z=self.target_z if self.target_z is not None else float("nan"),
            tower_radius=self.tower_radius,
            added=self.last_added_points,
            buffer_points=len(self.buffer),
            inliers=plane_inliers,
            total=plane_total,
            stable=self.stable_count,
            locked=self.plane_locked,
            normal=normal_text,
        )
        self.status_pub.publish(msg)
        rospy.loginfo_throttle(1.0, "[HubAcquisition] %s", msg.data)

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

    def run(self):
        rospy.loginfo("Hub acquisition output: %s (%s frame)", self.vel_topic, self.frame)
        self.wait_for_subscriber()
        last_cloud_publish = rospy.Time(0)

        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                radius_err = 0.0
                height_err = 0.0

                if self.odom is None:
                    self.phase = "waiting_odom"
                    self.load_reference_from_params()
                    self.publish_status(radius_err, height_err)
                    self.rate.sleep()
                    continue

                if not self.ensure_reference():
                    self.phase = "waiting_tower_reference"
                    self.publish_status(radius_err, height_err)
                    self.rate.sleep()
                    continue

                self.update_plane_fit(now)
                twist, radius_err, height_err = self.make_command(now)
                self.publish_twist(twist)
                self.publish_target(now)
                self.publish_normal(now)
                self.publish_markers(now)
                self.publish_status(radius_err, height_err)

                if (now - last_cloud_publish).to_sec() >= 0.5:
                    self.publish_cloud(now)
                    last_cloud_publish = now

                self.rate.sleep()
        finally:
            self.publish_zero_burst()


def main():
    args = parse_args()
    rospy.init_node("hub_acquisition_search")
    HubAcquisitionSearch(args).run()


if __name__ == "__main__":
    main()
