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

  bool global_enable_;
  double voxel_leaf_size_;
  std::size_t max_points_;
  int frames_since_downsample_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud_;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_POINTCLOUD_STITCHER_H
