#ifndef WTB_POINTCLOUD_MAPPING_MAPPING_CONFIG_H
#define WTB_POINTCLOUD_MAPPING_MAPPING_CONFIG_H

#include <string>
#include <vector>

#include <ros/node_handle.h>

namespace wtb_pointcloud_mapping
{

struct InputConfig
{
  std::string cloud_topic = "/livox/lidar_filtered";
  std::string odom_topic = "/iris_0/mavros/vision_odom/odom";
  std::string cloud_type = "livox_custom";
  bool use_tf = false;
  std::string world_frame = "map";
  std::string body_frame = "base_link";
  std::string lidar_frame = "livox_frame";
};

struct SyncConfig
{
  double max_time_diff = 0.05;
  bool strict_time_sync = false;
};

struct ExtrinsicConfig
{
  std::vector<double> lidar_to_body_xyz{0.0, 0.0, 0.0};
  std::vector<double> lidar_to_body_rpy{0.0, 0.0, 0.0};
};

struct GlobalCloudConfig
{
  bool enable = true;
  double voxel_leaf_size = 0.10;
  double publish_rate = 2.0;
  int max_points = 3000000;
};

struct OccupancyGridConfig
{
  bool enable = true;
  double resolution = 0.20;
  double log_odds_hit = 0.85;
  double log_odds_miss = -0.40;
  double log_odds_min = -2.0;
  double log_odds_max = 3.5;
  double occupied_threshold = 1.0;
  double free_threshold = -1.0;
  double max_ray_length = 80.0;
  double publish_rate = 2.0;
};

struct PathConfig
{
  bool enable = true;
  std::string topic = "/wtb/uav_path";
  double min_distance = 0.05;
  int max_points = 10000;
};

struct DebugConfig
{
  bool print_timing = true;
  bool publish_free_cloud = false;
};

struct MappingConfig
{
  InputConfig input;
  SyncConfig sync;
  ExtrinsicConfig extrinsic;
  GlobalCloudConfig global_cloud;
  OccupancyGridConfig occupancy_grid;
  PathConfig path;
  DebugConfig debug;
};

MappingConfig loadMappingConfig(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

std::vector<double> ensureVector3(const std::vector<double>& input,
                                  const std::vector<double>& fallback);

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_MAPPING_CONFIG_H
