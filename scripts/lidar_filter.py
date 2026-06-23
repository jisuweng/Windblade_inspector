#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math

import rospy
import rostopic


class DirectCloudFilter:
    def __init__(self, args):
        self.input_topic = args.input_topic
        self.output_topic = args.output_topic
        self.msg_type = args.msg_type

        # 距离过滤
        self.use_range_filter = args.use_range_filter
        self.min_range = args.min_range
        self.max_range = args.max_range

        # z 轴高度过滤
        self.use_z_filter = args.use_z_filter
        self.z_min = args.z_min
        self.z_max = args.z_max

        # 俯仰角过滤
        self.use_angle_filter = args.use_angle_filter
        self.min_vertical_angle_deg = args.min_vertical_angle_deg
        self.max_vertical_angle_deg = args.max_vertical_angle_deg
        self.min_vertical_angle = math.radians(self.min_vertical_angle_deg)
        self.max_vertical_angle = math.radians(self.max_vertical_angle_deg)

        if self.msg_type == "auto":
            self.msg_type = self.detect_msg_type()

        if self.msg_type == "pointcloud2":
            self.init_pointcloud2_mode()
        elif self.msg_type == "livox_custom":
            self.init_livox_custom_mode()
        else:
            rospy.logerr("不支持的 msg_type: %s", self.msg_type)
            rospy.signal_shutdown("Unknown msg_type")

        self.print_params()

    def print_params(self):
        rospy.loginfo("========== Direct Cloud Filter ==========")
        rospy.loginfo("input_topic              : %s", self.input_topic)
        rospy.loginfo("output_topic             : %s", self.output_topic)
        rospy.loginfo("msg_type                 : %s", self.msg_type)

        rospy.loginfo("use_range_filter         : %s", self.use_range_filter)
        rospy.loginfo("min_range                : %.3f m", self.min_range)
        rospy.loginfo("max_range                : %.3f m", self.max_range)

        rospy.loginfo("use_z_filter             : %s", self.use_z_filter)
        rospy.loginfo("z_min                    : %.3f m", self.z_min)
        rospy.loginfo("z_max                    : %.3f m", self.z_max)

        rospy.loginfo("use_angle_filter         : %s", self.use_angle_filter)
        rospy.loginfo("min_vertical_angle_deg   : %.3f deg", self.min_vertical_angle_deg)
        rospy.loginfo("max_vertical_angle_deg   : %.3f deg", self.max_vertical_angle_deg)
        rospy.loginfo("=========================================")

    def detect_msg_type(self):
        rospy.loginfo("正在自动检测话题类型: %s", self.input_topic)

        topic_type, _, _ = rostopic.get_topic_type(self.input_topic, blocking=True)

        rospy.loginfo("检测到话题类型: %s", topic_type)

        if topic_type == "sensor_msgs/PointCloud2":
            return "pointcloud2"

        if topic_type in [
            "livox_ros_driver/CustomMsg",
            "livox_ros_driver2/CustomMsg"
        ]:
            return "livox_custom"

        rospy.logerr("无法自动识别该点云类型: %s", topic_type)
        rospy.logerr("请手动指定 --msg-type pointcloud2 或 --msg-type livox_custom")
        rospy.signal_shutdown("Unsupported topic type")
        return "unknown"

    def init_pointcloud2_mode(self):
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs.point_cloud2 as pc2

        self.PointCloud2 = PointCloud2
        self.pc2 = pc2

        self.pub = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=5
        )

        self.sub = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.pointcloud2_callback,
            queue_size=5,
            buff_size=100 * 1024 * 1024
        )

    def init_livox_custom_mode(self):
        CustomMsg = None

        try:
            from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
            CustomMsg = LivoxCustomMsg
            rospy.loginfo("使用 livox_ros_driver2/CustomMsg")
        except ImportError:
            pass

        if CustomMsg is None:
            try:
                from livox_ros_driver.msg import CustomMsg as LivoxCustomMsg
                CustomMsg = LivoxCustomMsg
                rospy.loginfo("使用 livox_ros_driver/CustomMsg")
            except ImportError:
                pass

        if CustomMsg is None:
            rospy.logerr("无法导入 Livox CustomMsg")
            rospy.logerr("请确认已经 source 了 livox_ros_driver 或 livox_ros_driver2 工作空间")
            rospy.signal_shutdown("Livox CustomMsg import failed")
            return

        self.CustomMsg = CustomMsg

        self.pub = rospy.Publisher(
            self.output_topic,
            CustomMsg,
            queue_size=5
        )

        self.sub = rospy.Subscriber(
            self.input_topic,
            CustomMsg,
            self.livox_custom_callback,
            queue_size=5,
            buff_size=100 * 1024 * 1024
        )

    def keep_point(self, x, y, z):
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            return False

        # 1. 基于距离过滤
        if self.use_range_filter:
            distance = math.sqrt(x * x + y * y + z * z)

            if distance < self.min_range:
                return False

            if distance > self.max_range:
                return False

        # 2. 基于 z 轴高度过滤
        if self.use_z_filter:
            if z < self.z_min:
                return False

            if z > self.z_max:
                return False

        # 3. 基于俯仰角过滤
        if self.use_angle_filter:
            xy_distance = math.sqrt(x * x + y * y)
            vertical_angle = math.atan2(z, xy_distance)

            if vertical_angle < self.min_vertical_angle:
                return False

            if vertical_angle > self.max_vertical_angle:
                return False

        return True

    def pointcloud2_callback(self, msg):
        field_names = [field.name for field in msg.fields]

        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            rospy.logerr_throttle(1.0, "PointCloud2 中没有 x/y/z 字段")
            return

        x_idx = field_names.index("x")
        y_idx = field_names.index("y")
        z_idx = field_names.index("z")

        filtered_points = []

        for point in self.pc2.read_points(msg, field_names=None, skip_nans=True):
            x = float(point[x_idx])
            y = float(point[y_idx])
            z = float(point[z_idx])

            if self.keep_point(x, y, z):
                filtered_points.append(point)

        output_msg = self.pc2.create_cloud(
            msg.header,
            msg.fields,
            filtered_points
        )

        self.pub.publish(output_msg)

        input_num = msg.width * msg.height
        output_num = len(filtered_points)

        rospy.loginfo_throttle(
            2.0,
            "PointCloud2 filter: %d -> %d points",
            input_num,
            output_num
        )

    def livox_custom_callback(self, msg):
        output_msg = self.CustomMsg()

        output_msg.header = msg.header

        if hasattr(msg, "timebase"):
            output_msg.timebase = msg.timebase

        if hasattr(msg, "lidar_id"):
            output_msg.lidar_id = msg.lidar_id

        if hasattr(msg, "rsvd"):
            output_msg.rsvd = msg.rsvd

        filtered_points = []

        for p in msg.points:
            if self.keep_point(p.x, p.y, p.z):
                filtered_points.append(p)

        output_msg.points = filtered_points

        if hasattr(output_msg, "point_num"):
            output_msg.point_num = len(filtered_points)

        self.pub.publish(output_msg)

        input_num = len(msg.points)
        output_num = len(filtered_points)

        rospy.loginfo_throttle(
            2.0,
            "Livox CustomMsg filter: %d -> %d points",
            input_num,
            output_num
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="直接过滤 Livox / PointCloud2 点云：距离 + z高度 + 俯仰角"
    )

    parser.add_argument(
        "--input-topic",
        type=str,
        default="/livox/lidar",
        help="输入点云话题"
    )

    parser.add_argument(
        "--output-topic",
        type=str,
        default="/livox/lidar_filtered",
        help="输出过滤后点云话题"
    )

    parser.add_argument(
        "--msg-type",
        type=str,
        default="auto",
        choices=["auto", "pointcloud2", "livox_custom"],
        help="输入点云类型"
    )

    # 距离过滤
    parser.add_argument(
        "--use-range-filter",
        action="store_true",
        default=True,
        help="启用距离过滤"
    )

    parser.add_argument(
        "--no-range-filter",
        action="store_false",
        dest="use_range_filter",
        help="关闭距离过滤"
    )

    parser.add_argument(
        "--min-range",
        type=float,
        default=0.3,
        help="最小保留距离，单位 m"
    )

    parser.add_argument(
        "--max-range",
        type=float,
        default=50.0,
        help="最大保留距离，单位 m"
    )

    # z 轴高度过滤
    parser.add_argument(
        "--use-z-filter",
        action="store_true",
        default=True,
        help="启用 z 轴高度过滤"
    )

    parser.add_argument(
        "--no-z-filter",
        action="store_false",
        dest="use_z_filter",
        help="关闭 z 轴高度过滤"
    )

    parser.add_argument(
        "--z-min",
        type=float,
        default=-0.5,
        help="z 最小值，低于该值的点会被删除"
    )

    parser.add_argument(
        "--z-max",
        type=float,
        default=999.0,
        help="z 最大值，高于该值的点会被删除"
    )

    # 俯仰角过滤
    parser.add_argument(
        "--use-angle-filter",
        action="store_true",
        default=True,
        help="启用俯仰角过滤"
    )

    parser.add_argument(
        "--no-angle-filter",
        action="store_false",
        dest="use_angle_filter",
        help="关闭俯仰角过滤"
    )

    parser.add_argument(
        "--min-vertical-angle-deg",
        type=float,
        default=-8.0,
        help="最小俯仰角，低于该角度的点会被删除"
    )

    parser.add_argument(
        "--max-vertical-angle-deg",
        type=float,
        default=90.0,
        help="最大俯仰角，高于该角度的点会被删除"
    )

    args, unknown = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rospy.init_node("direct_cloud_filter", anonymous=False)

    DirectCloudFilter(args)

    rospy.spin()


if __name__ == "__main__":
    main()
