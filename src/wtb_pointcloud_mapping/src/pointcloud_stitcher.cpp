#include "wtb_pointcloud_mapping/pointcloud_stitcher.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl/filters/voxel_grid.h>
#include <ros/console.h>

namespace wtb_pointcloud_mapping
{

PointCloudStitcher::PointCloudStitcher()
  : min_range_(0.5),
    max_range_(80.0),
    z_min_(-10.0),
    z_max_(120.0),
    remove_nan_(true),
    global_enable_(true),
    voxel_leaf_size_(0.10),
    max_points_(3000000),
    frames_since_downsample_(0),
    global_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{
  global_cloud_->height = 1;
  global_cloud_->is_dense = false;
}

void PointCloudStitcher::setFilterParams(double min_range,
                                         double max_range,
                                         double z_min,
                                         double z_max,
                                         bool remove_nan)
{
  min_range_ = std::max(0.0, min_range);
  max_range_ = std::max(min_range_, max_range);
  z_min_ = z_min;
  z_max_ = z_max;
  remove_nan_ = remove_nan;
}

void PointCloudStitcher::setGlobalCloudParams(bool enable,
                                              double voxel_leaf_size,
                                              std::size_t max_points)
{
  global_enable_ = enable;
  voxel_leaf_size_ = std::max(0.0, voxel_leaf_size);
  max_points_ = max_points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudStitcher::transformToWorld(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
    const Eigen::Isometry3d& T_map_lidar)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>);
  cloud_world->height = 1;
  cloud_world->is_dense = false;

  if (!cloud_lidar || cloud_lidar->empty())
  {
    cloud_world->width = 0;
    return cloud_world;
  }

  cloud_world->points.reserve(cloud_lidar->points.size());

  for (const auto& p_lidar : cloud_lidar->points)
  {
    const bool finite_lidar =
        std::isfinite(p_lidar.x) && std::isfinite(p_lidar.y) && std::isfinite(p_lidar.z);
    if (!finite_lidar)
    {
      if (remove_nan_)
      {
        continue;
      }
      continue;
    }

    const double range = std::sqrt(static_cast<double>(p_lidar.x) * p_lidar.x +
                                   static_cast<double>(p_lidar.y) * p_lidar.y +
                                   static_cast<double>(p_lidar.z) * p_lidar.z);
    if (range < min_range_ || range > max_range_)
    {
      continue;
    }

    const Eigen::Vector3d point_lidar(p_lidar.x, p_lidar.y, p_lidar.z);
    const Eigen::Vector3d point_map = T_map_lidar * point_lidar;

    if (!std::isfinite(point_map.x()) || !std::isfinite(point_map.y()) ||
        !std::isfinite(point_map.z()))
    {
      continue;
    }

    if (point_map.z() < z_min_ || point_map.z() > z_max_)
    {
      continue;
    }

    pcl::PointXYZI p_world;
    p_world.x = static_cast<float>(point_map.x());
    p_world.y = static_cast<float>(point_map.y());
    p_world.z = static_cast<float>(point_map.z());
    p_world.intensity = p_lidar.intensity;
    cloud_world->points.push_back(p_world);
  }

  cloud_world->width = static_cast<std::uint32_t>(cloud_world->points.size());
  return cloud_world;
}

void PointCloudStitcher::addToGlobalCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world)
{
  if (!global_enable_ || !cloud_world || cloud_world->empty())
  {
    return;
  }

  *global_cloud_ += *cloud_world;
  global_cloud_->height = 1;
  global_cloud_->width = static_cast<std::uint32_t>(global_cloud_->points.size());
  global_cloud_->is_dense = false;

  ++frames_since_downsample_;
  if (voxel_leaf_size_ > 0.0 &&
      (frames_since_downsample_ >= 10 ||
       (max_points_ > 0 && global_cloud_->points.size() > max_points_)))
  {
    downsampleGlobalCloud();
  }

  if (max_points_ > 0 && global_cloud_->points.size() > max_points_)
  {
    ROS_WARN_THROTTLE(2.0,
                      "[WTBMapping] global cloud still has %zu points after filtering; "
                      "increase voxel_leaf_size or max_points.",
                      global_cloud_->points.size());
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudStitcher::getGlobalCloud() const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy(new pcl::PointCloud<pcl::PointXYZI>);
  if (global_cloud_)
  {
    *copy = *global_cloud_;
  }
  copy->height = 1;
  copy->width = static_cast<std::uint32_t>(copy->points.size());
  copy->is_dense = false;
  return copy;
}

std::size_t PointCloudStitcher::globalCloudSize() const
{
  return global_cloud_ ? global_cloud_->points.size() : 0;
}

void PointCloudStitcher::clearGlobalCloud()
{
  global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  global_cloud_->height = 1;
  global_cloud_->width = 0;
  global_cloud_->is_dense = false;
  frames_since_downsample_ = 0;
}

void PointCloudStitcher::downsampleGlobalCloud()
{
  frames_since_downsample_ = 0;

  if (!global_cloud_ || global_cloud_->empty() || voxel_leaf_size_ <= 0.0)
  {
    return;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(global_cloud_);
  const float leaf = static_cast<float>(voxel_leaf_size_);
  voxel_grid.setLeafSize(leaf, leaf, leaf);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid.filter(*filtered);
  filtered->height = 1;
  filtered->width = static_cast<std::uint32_t>(filtered->points.size());
  filtered->is_dense = false;
  global_cloud_ = filtered;
}

}  // namespace wtb_pointcloud_mapping
