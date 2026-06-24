#ifndef WTB_POINTCLOUD_MAPPING_MAPPING_PIPELINE_H
#define WTB_POINTCLOUD_MAPPING_MAPPING_PIPELINE_H

#include <cstddef>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/duration.h>
#include <ros/time.h>

#include "wtb_pointcloud_mapping/mapping_config.h"
#include "wtb_pointcloud_mapping/occupancy_grid_3d.h"
#include "wtb_pointcloud_mapping/pointcloud_stitcher.h"

namespace wtb_pointcloud_mapping
{

struct MappingDebugStats
{
  std::size_t lidar_points = 0;
  std::size_t world_points = 0;
  std::size_t global_cloud_points = 0;
  std::size_t occupancy_voxels = 0;
  std::size_t occupied_voxels = 0;
};

struct MappingUpdate
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_world;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr occupied_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr free_cloud;

  bool publish_global_cloud = false;
  bool publish_occupied_cloud = false;
  bool publish_free_cloud = false;

  MappingDebugStats debug;
};

class MappingPipeline
{
public:
  MappingPipeline();

  void configure(const MappingConfig& config);

  MappingUpdate process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
                        const Eigen::Isometry3d& T_map_lidar,
                        const ros::Time& now);

private:
  bool shouldPublish(const ros::Time& now,
                     const ros::Time& last_publish,
                     const ros::Duration& period) const;

  PointCloudStitcher stitcher_;
  OccupancyGrid3D occupancy_grid_;

  bool global_enable_ = true;
  bool occupancy_enable_ = true;
  bool publish_free_cloud_ = false;
  double occupancy_max_ray_length_ = 80.0;

  ros::Duration global_publish_period_;
  ros::Duration occupancy_publish_period_;
  ros::Time last_global_publish_time_;
  ros::Time last_occupied_publish_time_;
  ros::Time last_free_publish_time_;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_MAPPING_PIPELINE_H
