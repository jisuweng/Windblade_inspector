#include "wtb_pointcloud_mapping/mapping_pipeline.h"

#include <algorithm>
#include <cmath>

namespace wtb_pointcloud_mapping
{

namespace
{
ros::Duration periodFromRate(double rate)
{
  if (rate <= 0.0 || !std::isfinite(rate))
  {
    return ros::Duration(0.0);
  }
  return ros::Duration(1.0 / rate);
}
}  // namespace

MappingPipeline::MappingPipeline()
  : stitcher_(),
    occupancy_grid_(0.2),
    global_publish_period_(periodFromRate(2.0)),
    occupancy_publish_period_(periodFromRate(2.0))
{
}

void MappingPipeline::configure(const MappingConfig& config)
{
  global_enable_ = config.global_cloud.enable;
  stitcher_.setGlobalCloudParams(global_enable_,
                                 config.global_cloud.voxel_leaf_size,
                                 static_cast<std::size_t>(std::max(0, config.global_cloud.max_points)));
  global_publish_period_ = periodFromRate(config.global_cloud.publish_rate);

  occupancy_enable_ = config.occupancy_grid.enable;
  occupancy_max_ray_length_ = config.occupancy_grid.max_ray_length;
  occupancy_grid_.setResolution(config.occupancy_grid.resolution);
  occupancy_grid_.setLogOddsParams(config.occupancy_grid.log_odds_hit,
                                   config.occupancy_grid.log_odds_miss,
                                   config.occupancy_grid.log_odds_min,
                                   config.occupancy_grid.log_odds_max,
                                   config.occupancy_grid.occupied_threshold,
                                   config.occupancy_grid.free_threshold);
  occupancy_publish_period_ = periodFromRate(config.occupancy_grid.publish_rate);

  publish_free_cloud_ = config.debug.publish_free_cloud;
}

MappingUpdate MappingPipeline::process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
    const Eigen::Isometry3d& T_map_lidar,
    const ros::Time& now)
{
  MappingUpdate update;
  update.debug.lidar_points = cloud_lidar ? cloud_lidar->points.size() : 0;
  update.current_cloud_world = stitcher_.transformToWorld(cloud_lidar, T_map_lidar);
  update.debug.world_points =
      update.current_cloud_world ? update.current_cloud_world->points.size() : 0;

  if (global_enable_)
  {
    stitcher_.addToGlobalCloud(update.current_cloud_world);
    if (shouldPublish(now, last_global_publish_time_, global_publish_period_))
    {
      update.global_cloud = stitcher_.getGlobalCloud();
      update.publish_global_cloud = true;
      last_global_publish_time_ = now;
    }
  }

  if (occupancy_enable_)
  {
    occupancy_grid_.updatePointCloud(
        T_map_lidar.translation(), update.current_cloud_world, occupancy_max_ray_length_);

    if (shouldPublish(now, last_occupied_publish_time_, occupancy_publish_period_))
    {
      update.occupied_cloud = occupancy_grid_.getOccupiedCloud();
      update.publish_occupied_cloud = true;
      last_occupied_publish_time_ = now;
    }

    if (publish_free_cloud_ &&
        shouldPublish(now, last_free_publish_time_, occupancy_publish_period_))
    {
      update.free_cloud = occupancy_grid_.getFreeCloud();
      update.publish_free_cloud = true;
      last_free_publish_time_ = now;
    }
  }

  update.debug.global_cloud_points = stitcher_.globalCloudSize();
  update.debug.occupancy_voxels = occupancy_grid_.size();
  update.debug.occupied_voxels = occupancy_grid_.countOccupied();
  return update;
}

bool MappingPipeline::shouldPublish(const ros::Time& now,
                                    const ros::Time& last_publish,
                                    const ros::Duration& period) const
{
  return period.toSec() <= 0.0 || last_publish.isZero() || (now - last_publish) >= period;
}

}  // namespace wtb_pointcloud_mapping
