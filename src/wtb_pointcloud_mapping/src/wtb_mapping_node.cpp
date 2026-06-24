#include "wtb_pointcloud_mapping/wtb_mapping_node.h"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>

#include "wtb_pointcloud_mapping/ros_conversions.h"

namespace wtb_pointcloud_mapping
{

namespace
{
double poseDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

WTBMappingNode::WTBMappingNode()
  : nh_(),
    private_nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    config_(loadMappingConfig(nh_, private_nh_)),
    pipeline_(),
    has_odom_(false),
    T_body_lidar_(transformFromXYZRPY(config_.extrinsic.lidar_to_body_xyz,
                                      config_.extrinsic.lidar_to_body_rpy))
{
  pipeline_.configure(config_);

  if (config_.input.cloud_type != "livox_custom")
  {
    ROS_WARN_STREAM("[WTBMapping] input/cloud_type is '" << config_.input.cloud_type
                    << "', but this node expects livox_custom.");
  }

  initializeRos();
}

void WTBMappingNode::initializeRos()
{
  cloud_sub_ =
      nh_.subscribe(config_.input.cloud_topic, 5, &WTBMappingNode::cloudCallback, this);
  odom_sub_ =
      nh_.subscribe(config_.input.odom_topic, 20, &WTBMappingNode::odomCallback, this);

  current_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/current_cloud_world", 1);
  global_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/global_cloud", 1);
  occupied_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/occupied_cloud", 1);
  free_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/free_cloud", 1);
  if (config_.path.enable)
  {
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.path.topic, 1, true);
  }
  debug_info_pub_ =
      nh_.advertise<std_msgs::String>("/wtb/map_debug_info", 1);

  ROS_INFO_STREAM("[WTBMapping] subscribing cloud: " << config_.input.cloud_topic);
  ROS_INFO_STREAM("[WTBMapping] subscribing odom: " << config_.input.odom_topic);
  if (config_.path.enable)
  {
    ROS_INFO_STREAM("[WTBMapping] publishing UAV path: " << config_.path.topic
                    << ", min_distance=" << config_.path.min_distance
                    << ", max_points=" << config_.path.max_points);
  }
  ROS_INFO_STREAM("[WTBMapping] world_frame=" << config_.input.world_frame
                  << ", body_frame=" << config_.input.body_frame
                  << ", lidar_frame=" << config_.input.lidar_frame
                  << ", use_tf=" << (config_.input.use_tf ? "true" : "false"));
}

void WTBMappingNode::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  const nav_msgs::Odometry odom = *msg;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = odom;
    has_odom_ = true;
  }
  updateAndPublishPath(odom);
}

void WTBMappingNode::cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& msg)
{
  const ros::WallTime start_time = ros::WallTime::now();

  nav_msgs::Odometry odom;
  if (!getLatestOdom(odom))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[WTBMapping] waiting for odom on %s",
                      config_.input.odom_topic.c_str());
    return;
  }

  double time_diff = 0.0;
  if (!acceptTimeDiff(msg->header.stamp, odom.header.stamp, time_diff))
  {
    return;
  }

  Eigen::Isometry3d T_body_lidar = T_body_lidar_;
  if (config_.input.use_tf && !lookupBodyToLidarTransform(msg->header.stamp, T_body_lidar))
  {
    return;
  }

  const Eigen::Isometry3d T_map_lidar = odomToTransform(odom) * T_body_lidar;
  const MappingUpdate update = pipeline_.process(
      livoxCustomMsgToPCL(msg), T_map_lidar, ros::Time::now());

  publishMappingUpdate(update, msg->header.stamp);

  const double processing_ms = (ros::WallTime::now() - start_time).toSec() * 1000.0;
  publishDebugInfo(update.debug, time_diff, processing_ms);
}

bool WTBMappingNode::getLatestOdom(nav_msgs::Odometry& odom) const
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!has_odom_)
  {
    return false;
  }
  odom = latest_odom_;
  return true;
}

bool WTBMappingNode::acceptTimeDiff(const ros::Time& cloud_stamp,
                                    const ros::Time& odom_stamp,
                                    double& time_diff) const
{
  time_diff = std::abs((cloud_stamp - odom_stamp).toSec());
  if (time_diff <= config_.sync.max_time_diff)
  {
    return true;
  }

  ROS_WARN_THROTTLE(1.0,
                    "[WTBMapping] cloud-odom time diff %.6f s exceeds %.6f s",
                    time_diff,
                    config_.sync.max_time_diff);
  return !config_.sync.strict_time_sync;
}

bool WTBMappingNode::lookupBodyToLidarTransform(const ros::Time& stamp,
                                                Eigen::Isometry3d& T_body_lidar) const
{
  try
  {
    const ros::Time lookup_stamp = stamp.isZero() ? ros::Time(0) : stamp;
    const geometry_msgs::TransformStamped tf_msg =
        tf_buffer_.lookupTransform(config_.input.body_frame,
                                   config_.input.lidar_frame,
                                   lookup_stamp,
                                   ros::Duration(0.02));
    T_body_lidar = tf2::transformToEigen(tf_msg);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[WTBMapping] failed to lookup T_%s_%s: %s",
                      config_.input.body_frame.c_str(),
                      config_.input.lidar_frame.c_str(),
                      ex.what());
    return false;
  }
}

void WTBMappingNode::publishMappingUpdate(const MappingUpdate& update, const ros::Time& stamp)
{
  current_cloud_pub_.publish(
      cloudToRosMsg(update.current_cloud_world, config_.input.world_frame, stamp));

  if (update.publish_global_cloud)
  {
    global_cloud_pub_.publish(
        cloudToRosMsg(update.global_cloud, config_.input.world_frame, stamp));
  }

  if (update.publish_occupied_cloud)
  {
    occupied_cloud_pub_.publish(
        cloudToRosMsg(update.occupied_cloud, config_.input.world_frame, stamp));
  }

  if (update.publish_free_cloud)
  {
    free_cloud_pub_.publish(
        cloudToRosMsg(update.free_cloud, config_.input.world_frame, stamp));
  }
}

void WTBMappingNode::updateAndPublishPath(const nav_msgs::Odometry& odom)
{
  if (!config_.path.enable)
  {
    return;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = odom.header.stamp.isZero() ? ros::Time::now() : odom.header.stamp;
  pose.header.frame_id =
      config_.input.world_frame.empty() ? odom.header.frame_id : config_.input.world_frame;
  pose.pose = odom.pose.pose;

  nav_msgs::Path path_to_publish;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    uav_path_.header = pose.header;

    if (uav_path_.poses.empty())
    {
      uav_path_.poses.push_back(pose);
    }
    else if (poseDistance(uav_path_.poses.back().pose, pose.pose) >= config_.path.min_distance)
    {
      uav_path_.poses.push_back(pose);
    }
    else
    {
      uav_path_.poses.back() = pose;
    }

    if (config_.path.max_points > 0 &&
        uav_path_.poses.size() > static_cast<std::size_t>(config_.path.max_points))
    {
      const std::size_t overflow =
          uav_path_.poses.size() - static_cast<std::size_t>(config_.path.max_points);
      uav_path_.poses.erase(uav_path_.poses.begin(), uav_path_.poses.begin() + overflow);
    }

    path_to_publish = uav_path_;
  }

  path_pub_.publish(path_to_publish);
}

void WTBMappingNode::publishDebugInfo(const MappingDebugStats& stats,
                                      double time_diff,
                                      double processing_ms) const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "[WTBMapping] current lidar points: " << stats.lidar_points
     << "\n[WTBMapping] current world points: " << stats.world_points
     << "\n[WTBMapping] global cloud points: " << stats.global_cloud_points
     << "\n[WTBMapping] occupancy voxels: " << stats.occupancy_voxels
     << "\n[WTBMapping] occupied voxels: " << stats.occupied_voxels
     << "\n[WTBMapping] cloud-odom time diff: " << time_diff << " s"
     << "\n[WTBMapping] processing time: " << processing_ms << " ms";

  std_msgs::String debug_msg;
  debug_msg.data = ss.str();
  debug_info_pub_.publish(debug_msg);

  if (config_.debug.print_timing)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, debug_msg.data);
  }
}

}  // namespace wtb_pointcloud_mapping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wtb_mapping_node");
  wtb_pointcloud_mapping::WTBMappingNode node;
  ros::spin();
  return 0;
}
