#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云过滤 + 塔筒点统计。
订阅 /cloud_registered，转到机体坐标系，过滤后发布 /cloud_registered_filtered，
同时统计不同距离下打到塔筒上的点数，Ctrl+C 停止后生成统计图。
"""

import math
import os
import signal
import rospy

import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2


class FilterAndStats:
    def __init__(self):
        rospy.init_node("filter_and_stats")

        # ---- 输入输出 ----
        self.input_topic = rospy.get_param("~input_topic", "/cloud_registered")
        self.output_topic = rospy.get_param("~output_topic", "/cloud_registered_filtered")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")

        # ---- 过滤参数 ----
        self.ground_remove_z = rospy.get_param("~ground_remove_z", -999.0)
        self.z_max = rospy.get_param("~z_max", 3.0)
        self.range_min = rospy.get_param("~range_min", 1.0)
        self.range_max = rospy.get_param("~range_max", 8.0)
        self.front_angle_deg = rospy.get_param("~front_angle_deg", 90.0)
        self.x_min = rospy.get_param("~x_min", 0.0)
        self.y_abs = rospy.get_param("~y_abs", -1.0)
        self.use_3d_range = rospy.get_param("~use_3d_range", False)

        # ---- 塔筒统计参数 ----
        self.stats_enabled = rospy.get_param("~stats_enabled", True)
        self.bin_size = rospy.get_param("~stats_bin_size", 1.0)
        self.stats_max_range = rospy.get_param("~stats_max_range", 30.0)
        self.tower_angle_deg = rospy.get_param("~stats_tower_angle_deg", 20.0)
        self.tower_z_min = rospy.get_param("~stats_tower_z_min", -2.0)
        self.tower_z_max = rospy.get_param("~stats_tower_z_max", 10.0)
        self.tower_y_half = rospy.get_param("~stats_tower_y_half", 3.0)
        self.output_dir = rospy.get_param("~stats_output_dir", "")

        # ---- 内部状态 ----
        self.field_names = None
        self.latest_position = None
        self.latest_rotation = None

        # 统计累计
        self.total_frames = 0
        self.total_raw = 0
        self.total_filtered = 0
        self.bin_counts = {}
        self.bin_point_counts = {}

        # ---- Pub/Sub ----
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=10
        )
        self.pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=10
        )
        self.sub = rospy.Subscriber(
            self.input_topic, PointCloud2, self.callback, queue_size=10
        )

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo("=" * 55)
        rospy.loginfo("Filter + Tower Stats started.")
        rospy.loginfo("  Input: %s  ->  Output: %s", self.input_topic, self.output_topic)
        rospy.loginfo("  Odom: %s", self.odom_topic)
        rospy.loginfo("  Filter: ground_remove_z=%.2f, z_max=%.2f, range=[%.1f,%.1f], angle=%.1f deg, x_min=%.1f",
                      self.ground_remove_z, self.z_max,
                      self.range_min, self.range_max,
                      self.front_angle_deg, self.x_min)
        rospy.loginfo("  Stats: enabled=%s, bin=%.1fm, max_range=%.1fm, tower_angle=%.1f deg, z=[%.1f,%.1f]",
                      self.stats_enabled, self.bin_size, self.stats_max_range,
                      self.tower_angle_deg, self.tower_z_min, self.tower_z_max)
        rospy.loginfo("=" * 55)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_position = np.array([p.x, p.y, p.z])
        self.latest_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])

    # ============================================================
    #  过滤
    # ============================================================
    def keep_point(self, x, y, z):
        if self.ground_remove_z > -999.0 and z < self.ground_remove_z:
            return False
        if z > self.z_max:
            return False
        if x < self.x_min:
            return False

        if self.use_3d_range:
            r = math.sqrt(x * x + y * y + z * z)
        else:
            r = math.sqrt(x * x + y * y)

        if r < self.range_min:
            return False
        if r > self.range_max:
            return False

        angle_deg = math.degrees(math.atan2(y, x))
        if abs(angle_deg) > self.front_angle_deg / 2.0:
            return False

        if self.y_abs > 0.0:
            if abs(y) > self.y_abs:
                return False

        return True

    # ============================================================
    #  塔筒 ROI
    # ============================================================
    def in_tower_roi(self, x, y, z):
        if z < self.tower_z_min or z > self.tower_z_max:
            return False
        if abs(y) > self.tower_y_half:
            return False
        angle = abs(math.degrees(math.atan2(y, x)))
        if angle > self.tower_angle_deg:
            return False
        return True

    # ============================================================
    #  主回调
    # ============================================================
    def callback(self, msg):
        if self.field_names is None:
            self.field_names = [f.name for f in msg.fields]
            rospy.loginfo("Input fields: %s (%d pts)",
                          self.field_names, msg.width * msg.height)

        if self.latest_position is None:
            rospy.logwarn_throttle(3.0, "Waiting for odometry on %s ...", self.odom_topic)
            return

        pos_world = self.latest_position
        rot_world_to_body = self.latest_rotation.inv()

        filtered_points = []
        frame_bins = {}
        frame_bin_pts = {}

        n_raw = 0

        for p in pc2.read_points(msg, field_names=self.field_names, skip_nans=True):
            n_raw += 1
            pw = np.array([float(p[0]), float(p[1]), float(p[2])])
            rest = [float(v) for v in p[3:]]

            p_body = rot_world_to_body.apply(pw - pos_world)
            xb, yb, zb = p_body[0], p_body[1], p_body[2]

            # 过滤
            if self.keep_point(xb, yb, zb):
                filtered_points.append([pw[0], pw[1], pw[2]] + rest)

            # 塔筒统计（对所有点，不仅仅是过滤后的）
            if self.stats_enabled and self.in_tower_roi(xb, yb, zb):
                r = math.sqrt(xb * xb + yb * yb)
                if r <= self.stats_max_range:
                    idx = int(r / self.bin_size)
                    frame_bins[idx] = frame_bins.get(idx, 0) + 1
                    if idx not in frame_bin_pts:
                        frame_bin_pts[idx] = []
                    frame_bin_pts[idx].append(r)

        self.total_frames += 1
        self.total_raw += n_raw
        self.total_filtered += len(filtered_points)

        for idx, cnt in frame_bins.items():
            self.bin_counts[idx] = self.bin_counts.get(idx, 0) + cnt
            self.bin_point_counts[idx] = self.bin_point_counts.get(idx, 0) + 1

        # 发布过滤后点云
        filtered_cloud = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        self.pub.publish(filtered_cloud)

        rospy.loginfo_throttle(
            5.0,
            "Filtered: %d / %d pts (%.1f%%)  tower: %d pts  drone@[%.1f,%.1f,%.1f]",
            len(filtered_points), n_raw,
            100.0 * len(filtered_points) / max(1, n_raw),
            sum(frame_bins.values()),
            pos_world[0], pos_world[1], pos_world[2]
        )

    # ============================================================
    #  关闭 & 出图
    # ============================================================
    def signal_handler(self, sig, frame):
        rospy.signal_shutdown("Ctrl+C pressed")

    def on_shutdown(self):
        if self.stats_enabled and self.bin_counts:
            self.generate_chart()
        else:
            self.print_summary()

    def print_summary(self):
        rospy.loginfo("=" * 55)
        rospy.loginfo("Summary (%d frames):", self.total_frames)
        rospy.loginfo("  Raw points total: %d,  Filtered total: %d (%.1f%%)",
                      self.total_raw, self.total_filtered,
                      100.0 * self.total_filtered / max(1, self.total_raw))

        if not self.bin_counts:
            rospy.loginfo("=" * 55)
            return

        rospy.loginfo("  %8s  %12s  %12s", "range(m)", "total_pts", "avg/frame")
        for idx in sorted(self.bin_counts.keys()):
            r_lo = idx * self.bin_size
            r_hi = r_lo + self.bin_size
            total = self.bin_counts[idx]
            avg = total / self.bin_point_counts[idx] if self.bin_point_counts[idx] > 0 else 0
            rospy.loginfo("  [%4.1f-%4.1f]  %12d  %12.1f",
                          r_lo, r_hi, total, avg)
        rospy.loginfo("=" * 55)

    def generate_chart(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        idxs = sorted(self.bin_counts.keys())
        ranges = [idx * self.bin_size + self.bin_size / 2.0 for idx in idxs]
        total_pts = [self.bin_counts[idx] for idx in idxs]
        avg_pts = [self.bin_counts[idx] / max(1, self.bin_point_counts[idx])
                   for idx in idxs]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # 上图：总点数
        bars = ax1.bar(ranges, total_pts, width=self.bin_size * 0.8,
                       color='steelblue', edgecolor='white')
        ax1.set_xlabel('Distance (m)')
        ax1.set_ylabel('Total Tower Points')
        ax1.set_title(f'Tower Point Count vs Distance ({self.total_frames} frames)')

        for bar, val in zip(bars, total_pts):
            ax1.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + max(total_pts) * 0.01,
                     str(val), ha='center', va='bottom', fontsize=8)

        # 下图：平均每帧
        ax2.bar(ranges, avg_pts, width=self.bin_size * 0.8,
                color='darkorange', edgecolor='white')
        ax2.set_xlabel('Distance (m)')
        ax2.set_ylabel('Avg Tower Points per Frame')
        ax2.set_title('Average Tower Points per Frame vs Distance')

        for r_item, avg in zip(ranges, avg_pts):
            ax2.text(r_item, avg + max(avg_pts) * 0.01, f'{avg:.1f}',
                     ha='center', va='bottom', fontsize=8)

        labels = [f'{idx * self.bin_size:.0f}-{(idx + 1) * self.bin_size:.0f}'
                  for idx in idxs]
        ax1.set_xticks(ranges)
        ax1.set_xticklabels(labels, rotation=45)
        ax2.set_xticks(ranges)
        ax2.set_xticklabels(labels, rotation=45)

        plt.tight_layout()

        out_path = os.path.join(self.output_dir, 'tower_points_stats.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        plt.close(fig)

        rospy.loginfo("Chart saved to: %s", out_path)
        self.print_summary()


if __name__ == "__main__":
    node = FilterAndStats()
    rospy.spin()
