#ifndef WTB_POINTCLOUD_MAPPING_ROS_CONVERSIONS_H
#define WTB_POINTCLOUD_MAPPING_ROS_CONVERSIONS_H

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <livox_laser_simulation/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

namespace wtb_pointcloud_mapping
{

pcl::PointCloud<pcl::PointXYZI>::Ptr livoxCustomMsgToPCL(
    const livox_laser_simulation::CustomMsgConstPtr& msg);

Eigen::Isometry3d odomToTransform(const nav_msgs::Odometry& odom);

Eigen::Isometry3d transformFromXYZRPY(const std::vector<double>& xyz,
                                      const std::vector<double>& rpy);

sensor_msgs::PointCloud2 cloudToRosMsg(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& frame_id,
    const ros::Time& stamp);

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_ROS_CONVERSIONS_H
