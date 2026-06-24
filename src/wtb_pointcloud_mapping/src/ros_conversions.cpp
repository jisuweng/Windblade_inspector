#include "wtb_pointcloud_mapping/ros_conversions.h"

#include <cmath>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#include "wtb_pointcloud_mapping/mapping_config.h"

namespace wtb_pointcloud_mapping
{

pcl::PointCloud<pcl::PointXYZI>::Ptr livoxCustomMsgToPCL(
    const livox_laser_simulation::CustomMsgConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!msg)
  {
    cloud->height = 1;
    cloud->width = 0;
    cloud->is_dense = false;
    return cloud;
  }

  cloud->points.reserve(msg->points.size());
  for (const auto& livox_point : msg->points)
  {
    pcl::PointXYZI p;
    p.x = livox_point.x;
    p.y = livox_point.y;
    p.z = livox_point.z;
    p.intensity = static_cast<float>(livox_point.reflectivity);
    cloud->points.push_back(p);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

Eigen::Isometry3d odomToTransform(const nav_msgs::Odometry& odom)
{
  const Eigen::Vector3d t_map_body(odom.pose.pose.position.x,
                                   odom.pose.pose.position.y,
                                   odom.pose.pose.position.z);

  Eigen::Quaterniond q_map_body(odom.pose.pose.orientation.w,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z);
  if (q_map_body.norm() < 1e-12 || !std::isfinite(q_map_body.norm()))
  {
    ROS_WARN_THROTTLE(1.0, "[WTBMapping] odom quaternion is invalid; using identity rotation");
    q_map_body = Eigen::Quaterniond::Identity();
  }
  else
  {
    q_map_body.normalize();
  }

  Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
  T_map_body.linear() = q_map_body.toRotationMatrix();
  T_map_body.translation() = t_map_body;
  return T_map_body;
}

Eigen::Isometry3d transformFromXYZRPY(const std::vector<double>& xyz,
                                      const std::vector<double>& rpy)
{
  const std::vector<double> safe_xyz = ensureVector3(xyz, std::vector<double>{0.0, 0.0, 0.0});
  const std::vector<double> safe_rpy = ensureVector3(rpy, std::vector<double>{0.0, 0.0, 0.0});

  const double roll = safe_rpy[0];
  const double pitch = safe_rpy[1];
  const double yaw = safe_rpy[2];

  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(safe_xyz[0], safe_xyz[1], safe_xyz[2]);
  return transform;
}

sensor_msgs::PointCloud2 cloudToRosMsg(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& frame_id,
    const ros::Time& stamp)
{
  sensor_msgs::PointCloud2 msg;
  if (cloud)
  {
    pcl::toROSMsg(*cloud, msg);
  }
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

}  // namespace wtb_pointcloud_mapping
