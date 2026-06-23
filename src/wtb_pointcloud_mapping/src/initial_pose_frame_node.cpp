#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace wtb_pointcloud_mapping
{

class InitialPoseFrameNode
{
public:
  InitialPoseFrameNode() : nh_(), private_nh_("~")
  {
    readParam<std::string>("initial_pose_frame/input_odom_topic",
                           input_odom_topic_,
                           "/iris_0/mavros/vision_odom/odom");
    readParam<std::string>("initial_pose_frame/output_odom_topic",
                           output_odom_topic_,
                           "/wtb/odom");
    readParam<std::string>("initial_pose_frame/source_frame", source_frame_, "world");
    readParam<std::string>("initial_pose_frame/local_frame", local_frame_, "map");
    readParam<std::string>("initial_pose_frame/body_frame", body_frame_, "iris_0/base_link");

    if (source_frame_ == local_frame_)
    {
      throw std::invalid_argument("source_frame and local_frame must be different");
    }

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 20);
    odom_subscriber_ =
        nh_.subscribe(input_odom_topic_, 20, &InitialPoseFrameNode::odomCallback, this);

    ROS_INFO_STREAM("[InitialPoseFrame] waiting for initial pose on " << input_odom_topic_);
    ROS_INFO_STREAM("[InitialPoseFrame] will publish " << output_odom_topic_ << " and TF "
                                                        << source_frame_ << " -> " << local_frame_
                                                        << " -> " << body_frame_);
  }

private:
  template <typename T>
  void readParam(const std::string& key, T& value, const T& default_value)
  {
    if (private_nh_.getParam(key, value) || nh_.getParam(key, value))
    {
      return;
    }
    value = default_value;
  }

  static bool finitePosition(const geometry_msgs::Point& position)
  {
    return std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(position.z);
  }

  static void normalizeQuaternion(geometry_msgs::Quaternion& quaternion)
  {
    const double norm = std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                                  quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    if (!std::isfinite(norm) || norm < 1e-12)
    {
      quaternion.x = 0.0;
      quaternion.y = 0.0;
      quaternion.z = 0.0;
      quaternion.w = 1.0;
      return;
    }
    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    quaternion.w /= norm;
  }

  void captureInitialPosition(const nav_msgs::Odometry& odom)
  {
    initial_position_ = odom.pose.pose.position;
    initialized_ = true;

    geometry_msgs::TransformStamped source_to_local;
    source_to_local.header.stamp = odom.header.stamp;
    source_to_local.header.frame_id = source_frame_;
    source_to_local.child_frame_id = local_frame_;
    source_to_local.transform.translation.x = initial_position_.x;
    source_to_local.transform.translation.y = initial_position_.y;
    source_to_local.transform.translation.z = initial_position_.z;
    source_to_local.transform.rotation.w = 1.0;
    static_broadcaster_.sendTransform(source_to_local);

    ROS_INFO_STREAM("[InitialPoseFrame] map origin captured at " << source_frame_ << " position ["
                                                                 << initial_position_.x << ", "
                                                                 << initial_position_.y << ", "
                                                                 << initial_position_.z << "]");
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& input)
  {
    if (!finitePosition(input->pose.pose.position))
    {
      ROS_WARN_THROTTLE(1.0, "[InitialPoseFrame] ignoring non-finite odometry position");
      return;
    }

    if (!initialized_)
    {
      captureInitialPosition(*input);
    }

    nav_msgs::Odometry output = *input;
    output.header.frame_id = local_frame_;
    output.child_frame_id = body_frame_;
    output.pose.pose.position.x -= initial_position_.x;
    output.pose.pose.position.y -= initial_position_.y;
    output.pose.pose.position.z -= initial_position_.z;
    normalizeQuaternion(output.pose.pose.orientation);
    odom_publisher_.publish(output);

    geometry_msgs::TransformStamped local_to_body;
    local_to_body.header = output.header;
    local_to_body.child_frame_id = body_frame_;
    local_to_body.transform.translation.x = output.pose.pose.position.x;
    local_to_body.transform.translation.y = output.pose.pose.position.y;
    local_to_body.transform.translation.z = output.pose.pose.position.z;
    local_to_body.transform.rotation = output.pose.pose.orientation;
    dynamic_broadcaster_.sendTransform(local_to_body);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher odom_publisher_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string source_frame_;
  std::string local_frame_;
  std::string body_frame_;
  geometry_msgs::Point initial_position_;
  bool initialized_ = false;
};

}  // namespace wtb_pointcloud_mapping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial_pose_frame");
  try
  {
    wtb_pointcloud_mapping::InitialPoseFrameNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("[InitialPoseFrame] " << exception.what());
    return 1;
  }
  return 0;
}
