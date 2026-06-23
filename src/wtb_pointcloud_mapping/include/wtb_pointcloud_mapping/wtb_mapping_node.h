#ifndef WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H
#define WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <livox_laser_simulation/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "wtb_pointcloud_mapping/occupancy_grid_3d.h"
#include "wtb_pointcloud_mapping/pointcloud_stitcher.h"
#include "wtb_pointcloud_mapping/tower_axis_estimator.h"
#include "wtb_pointcloud_mapping/tower_ascent_controller.h"
#include <geometry_msgs/PoseStamped.h>

namespace wtb_pointcloud_mapping
{

class WTBMappingNode
{
public:
  WTBMappingNode();

private:
  template <typename T>
  void readParam(const std::string& key, T& value, const T& default_value)
  {
    if (private_nh_.getParam(key, value))
    {
      return;
    }
    if (nh_.getParam(key, value))
    {
      return;
    }
    value = default_value;
  }

  void loadParams();
  void initializeRos();

  void cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr convertLivoxCustomMsgToPCL(
      const livox_laser_simulation::CustomMsgConstPtr& msg) const;

  Eigen::Isometry3d odomToTransform(const nav_msgs::Odometry& odom) const;
  Eigen::Isometry3d makeTransformFromXYZRPY(const std::vector<double>& xyz,
                                            const std::vector<double>& rpy) const;
  bool lookupBodyToLidarTransform(const ros::Time& stamp,
                                  Eigen::Isometry3d& T_body_lidar) const;

  sensor_msgs::PointCloud2 cloudToRosMsg(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      const std::string& frame_id,
      const ros::Time& stamp) const;

  bool shouldPublish(const ros::Time& now,
                     const ros::Time& last_publish,
                     const ros::Duration& period) const;

  void publishDebugInfo(std::size_t lidar_points,
                        std::size_t world_points,
                        double time_diff,
                        double processing_ms) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr selectTowerInputCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud_world) const;

  void publishTowerAxisResult(const TowerAxisEstimator::Result& result,
                              const ros::Time& stamp) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr controlPointsToCloud(
      const std::vector<Eigen::Vector3d>& control_points) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr sliceCentersToCloud(
      const std::vector<TowerAxisEstimator::SliceCenter>& centers) const;

  visualization_msgs::Marker makeLineMarker(const TowerAxisEstimator::LineModel& line,
                                            const std::string& marker_ns,
                                            int marker_id,
                                            const ros::Time& stamp,
                                            double length,
                                            double width,
                                            float r,
                                            float g,
                                            float b,
                                            float a) const;

  visualization_msgs::Marker makeAxisMarker(const TowerAxisEstimator::LineModel& line,
                                            const std::string& marker_ns,
                                            int marker_id,
                                            const ros::Time& stamp,
                                            double z_min,
                                            double z_max,
                                            double width,
                                            float r,
                                            float g,
                                            float b,
                                            float a) const;

  visualization_msgs::Marker makeGlobalAxisMarker(const TowerAxisEstimator::Result& result,
                                                  const ros::Time& stamp) const;

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
  ros::Publisher debug_info_pub_;
  ros::Publisher tower_candidate_cloud_pub_;
  ros::Publisher tower_slice_centers_pub_;
  ros::Publisher tower_control_points_pub_;
  ros::Publisher tower_axis_marker_pub_;
  ros::Publisher tower_global_axis_marker_pub_;
  ros::Publisher tower_ransac_line_marker_pub_;
  ros::Publisher tower_debug_info_pub_;

  PointCloudStitcher stitcher_;
  OccupancyGrid3D occupancy_grid_;
  TowerAxisEstimator tower_axis_estimator_;
  TowerAscentController tower_ascent_controller_;

  ros::Publisher tower_setpoint_pub_;
  ros::Publisher tower_path_preview_pub_;

  nav_msgs::Odometry latest_odom_;
  bool has_odom_;
  mutable std::mutex odom_mutex_;

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string cloud_type_;
  bool use_tf_;
  std::string world_frame_;
  std::string body_frame_;
  std::string lidar_frame_;

  double max_time_diff_;
  bool strict_time_sync_;

  std::vector<double> lidar_to_body_xyz_;
  std::vector<double> lidar_to_body_rpy_;
  Eigen::Isometry3d T_body_lidar_;

  bool global_enable_;
  double global_publish_rate_;
  ros::Duration global_publish_period_;
  ros::Time last_global_publish_time_;

  bool occupancy_enable_;
  double occupancy_max_ray_length_;
  double occupancy_publish_rate_;
  ros::Duration occupancy_publish_period_;
  ros::Time last_occupied_publish_time_;
  ros::Time last_free_publish_time_;

  bool publish_free_cloud_;
  bool print_timing_;

  bool tower_axis_enable_;
  std::string tower_axis_method_;
  std::string tower_axis_input_cloud_mode_;
  bool tower_publish_candidate_cloud_;
  bool tower_publish_slice_centers_;
  bool tower_publish_control_points_;
  bool tower_publish_markers_;
  bool tower_publish_debug_ransac_line_;

  bool tower_ascent_enable_;
  TowerAscentController::Config tower_ascent_config_;
  ros::Time last_ascent_publish_time_;
  ros::Duration ascent_publish_period_;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_WTB_MAPPING_NODE_H
