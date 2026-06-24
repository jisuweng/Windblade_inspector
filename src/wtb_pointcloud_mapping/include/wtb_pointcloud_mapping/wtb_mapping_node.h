#ifndef WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H
#define WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <livox_laser_simulation/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wtb_pointcloud_mapping/mapping_config.h"
#include "wtb_pointcloud_mapping/mapping_pipeline.h"

namespace wtb_pointcloud_mapping
{

class WTBMappingNode
{
public:
  WTBMappingNode();

private:
  void initializeRos();

  void cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  bool getLatestOdom(nav_msgs::Odometry& odom) const;
  bool acceptTimeDiff(const ros::Time& cloud_stamp,
                      const ros::Time& odom_stamp,
                      double& time_diff) const;
  bool lookupBodyToLidarTransform(const ros::Time& stamp,
                                  Eigen::Isometry3d& T_body_lidar) const;
  void publishMappingUpdate(const MappingUpdate& update, const ros::Time& stamp);
  void updateAndPublishPath(const nav_msgs::Odometry& odom);

  void publishDebugInfo(const MappingDebugStats& stats,
                        double time_diff,
                        double processing_ms) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher current_cloud_pub_;
  ros::Publisher global_cloud_pub_;
  ros::Publisher occupied_cloud_pub_;
  ros::Publisher free_cloud_pub_;
  ros::Publisher path_pub_;
  ros::Publisher debug_info_pub_;

  MappingConfig config_;
  MappingPipeline pipeline_;

  nav_msgs::Odometry latest_odom_;
  bool has_odom_;
  mutable std::mutex odom_mutex_;

  nav_msgs::Path uav_path_;
  mutable std::mutex path_mutex_;

  Eigen::Isometry3d T_body_lidar_;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H
