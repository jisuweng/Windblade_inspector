#ifndef WTB_POINTCLOUD_MAPPING_OCCUPANCY_GRID_3D_H
#define WTB_POINTCLOUD_MAPPING_OCCUPANCY_GRID_3D_H

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>

namespace wtb_pointcloud_mapping
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey& key) const;
};

struct VoxelData
{
  double log_odds = 0.0;
  ros::Time last_update;
  int hit_count = 0;
  int miss_count = 0;
};

class OccupancyGrid3D
{
public:
  explicit OccupancyGrid3D(double resolution = 0.2);

  void setResolution(double resolution);

  void setLogOddsParams(double hit,
                        double miss,
                        double min_l,
                        double max_l,
                        double occupied_th,
                        double free_th);

  VoxelKey pointToKey(const Eigen::Vector3d& p) const;

  Eigen::Vector3d keyToCenter(const VoxelKey& key) const;

  void updateRay(const Eigen::Vector3d& sensor_origin,
                 const Eigen::Vector3d& hit_point,
                 double max_ray_length);

  void updatePointCloud(const Eigen::Vector3d& sensor_origin,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
                        double max_ray_length);

  bool isOccupied(const Eigen::Vector3d& p) const;

  bool isFree(const Eigen::Vector3d& p) const;

  bool isUnknown(const Eigen::Vector3d& p) const;

  std::vector<Eigen::Vector3d> getOccupiedCenters() const;

  std::vector<Eigen::Vector3d> getFreeCenters() const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr getOccupiedCloud() const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr getFreeCloud() const;

  std::vector<Eigen::Vector3d> queryOccupiedInSphere(
      const Eigen::Vector3d& center,
      double radius) const;

  std::vector<Eigen::Vector3d> queryOccupiedInCylinder(
      const Eigen::Vector3d& base_center,
      const Eigen::Vector3d& axis_dir,
      double radius,
      double height) const;

  std::vector<Eigen::Vector3d> queryOccupiedInRing(
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& normal,
      double radius,
      double ring_width,
      double angle_step_rad) const;

  void clear();

  std::size_t size() const;
  std::size_t countOccupied() const;
  std::size_t countFree() const;

private:
  bool isOccupiedData(const VoxelData& data) const;
  bool isFreeData(const VoxelData& data) const;

  double resolution_;
  double inv_resolution_;

  double log_odds_hit_;
  double log_odds_miss_;
  double log_odds_min_;
  double log_odds_max_;
  double occupied_threshold_;
  double free_threshold_;

  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels_;

  void updateFreeVoxel(const VoxelKey& key);
  void updateOccupiedVoxel(const VoxelKey& key);
  double clampLogOdds(double value) const;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_OCCUPANCY_GRID_3D_H
