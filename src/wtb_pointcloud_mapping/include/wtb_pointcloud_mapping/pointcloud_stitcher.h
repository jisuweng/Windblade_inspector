#ifndef WTB_POINTCLOUD_MAPPING_POINTCLOUD_STITCHER_H
#define WTB_POINTCLOUD_MAPPING_POINTCLOUD_STITCHER_H

#include <cstddef>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace wtb_pointcloud_mapping
{

class PointCloudStitcher
{
public:
  PointCloudStitcher();

  void setFilterParams(double min_range,
                       double max_range,
                       double z_min,
                       double z_max,
                       bool remove_nan);

  void setGlobalCloudParams(bool enable,
                            double voxel_leaf_size,
                            std::size_t max_points);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformToWorld(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
      const Eigen::Isometry3d& T_map_lidar);

  void addToGlobalCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world);

  pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalCloud() const;

  std::size_t globalCloudSize() const;

  void clearGlobalCloud();

private:
  void downsampleGlobalCloud();

  double min_range_;
  double max_range_;
  double z_min_;
  double z_max_;
  bool remove_nan_;

  bool global_enable_;
  double voxel_leaf_size_;
  std::size_t max_points_;
  int frames_since_downsample_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud_;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_POINTCLOUD_STITCHER_H
