#include "wtb_pointcloud_mapping/mapping_config.h"

#include <algorithm>

namespace wtb_pointcloud_mapping
{

namespace
{
template <typename T>
void readParam(ros::NodeHandle& nh,
               ros::NodeHandle& private_nh,
               const std::string& key,
               T& value)
{
  if (private_nh.getParam(key, value))
  {
    return;
  }
  nh.getParam(key, value);
}
}  // namespace

std::vector<double> ensureVector3(const std::vector<double>& input,
                                  const std::vector<double>& fallback)
{
  if (input.size() == 3)
  {
    return input;
  }
  return fallback;
}

MappingConfig loadMappingConfig(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  MappingConfig config;

  readParam(nh, private_nh, "input/cloud_topic", config.input.cloud_topic);
  readParam(nh, private_nh, "input/odom_topic", config.input.odom_topic);
  readParam(nh, private_nh, "input/cloud_type", config.input.cloud_type);
  readParam(nh, private_nh, "input/use_tf", config.input.use_tf);
  readParam(nh, private_nh, "input/world_frame", config.input.world_frame);
  readParam(nh, private_nh, "input/body_frame", config.input.body_frame);
  readParam(nh, private_nh, "input/lidar_frame", config.input.lidar_frame);

  readParam(nh, private_nh, "sync/max_time_diff", config.sync.max_time_diff);
  readParam(nh, private_nh, "sync/strict_time_sync", config.sync.strict_time_sync);

  readParam(nh, private_nh, "extrinsic/lidar_to_body_xyz",
            config.extrinsic.lidar_to_body_xyz);
  readParam(nh, private_nh, "extrinsic/lidar_to_body_rpy",
            config.extrinsic.lidar_to_body_rpy);
  config.extrinsic.lidar_to_body_xyz =
      ensureVector3(config.extrinsic.lidar_to_body_xyz, std::vector<double>{0.0, 0.0, 0.0});
  config.extrinsic.lidar_to_body_rpy =
      ensureVector3(config.extrinsic.lidar_to_body_rpy, std::vector<double>{0.0, 0.0, 0.0});

  readParam(nh, private_nh, "global_cloud/enable", config.global_cloud.enable);
  readParam(nh, private_nh, "global_cloud/voxel_leaf_size",
            config.global_cloud.voxel_leaf_size);
  readParam(nh, private_nh, "global_cloud/publish_rate",
            config.global_cloud.publish_rate);
  readParam(nh, private_nh, "global_cloud/max_points", config.global_cloud.max_points);

  readParam(nh, private_nh, "occupancy_grid/enable", config.occupancy_grid.enable);
  readParam(nh, private_nh, "occupancy_grid/resolution", config.occupancy_grid.resolution);
  readParam(nh, private_nh, "occupancy_grid/log_odds_hit",
            config.occupancy_grid.log_odds_hit);
  readParam(nh, private_nh, "occupancy_grid/log_odds_miss",
            config.occupancy_grid.log_odds_miss);
  readParam(nh, private_nh, "occupancy_grid/log_odds_min",
            config.occupancy_grid.log_odds_min);
  readParam(nh, private_nh, "occupancy_grid/log_odds_max",
            config.occupancy_grid.log_odds_max);
  readParam(nh, private_nh, "occupancy_grid/occupied_threshold",
            config.occupancy_grid.occupied_threshold);
  readParam(nh, private_nh, "occupancy_grid/free_threshold",
            config.occupancy_grid.free_threshold);
  readParam(nh, private_nh, "occupancy_grid/max_ray_length",
            config.occupancy_grid.max_ray_length);
  readParam(nh, private_nh, "occupancy_grid/publish_rate",
            config.occupancy_grid.publish_rate);

  readParam(nh, private_nh, "path/enable", config.path.enable);
  readParam(nh, private_nh, "path/topic", config.path.topic);
  readParam(nh, private_nh, "path/min_distance", config.path.min_distance);
  readParam(nh, private_nh, "path/max_points", config.path.max_points);
  config.path.min_distance = std::max(0.0, config.path.min_distance);

  readParam(nh, private_nh, "debug/print_timing", config.debug.print_timing);
  readParam(nh, private_nh, "debug/publish_free_cloud", config.debug.publish_free_cloud);

  return config;
}

}  // namespace wtb_pointcloud_mapping
