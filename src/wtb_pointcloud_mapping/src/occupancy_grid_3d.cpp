#include "wtb_pointcloud_mapping/occupancy_grid_3d.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace wtb_pointcloud_mapping
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool isFiniteVector(const Eigen::Vector3d& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr centersToCloud(const std::vector<Eigen::Vector3d>& centers,
                                                    double intensity)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.reserve(centers.size());
  for (const auto& center : centers)
  {
    pcl::PointXYZI p;
    p.x = static_cast<float>(center.x());
    p.y = static_cast<float>(center.y());
    p.z = static_cast<float>(center.z());
    p.intensity = static_cast<float>(intensity);
    cloud->points.push_back(p);
  }
  cloud->height = 1;
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->is_dense = false;
  return cloud;
}
}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const
{
  std::size_t seed = 0;
  const auto mix = [&seed](int value) {
    seed ^= std::hash<int>()(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  };
  mix(key.x);
  mix(key.y);
  mix(key.z);
  return seed;
}

OccupancyGrid3D::OccupancyGrid3D(double resolution)
  : resolution_(0.2),
    inv_resolution_(5.0),
    log_odds_hit_(0.85),
    log_odds_miss_(-0.40),
    log_odds_min_(-2.0),
    log_odds_max_(3.5),
    occupied_threshold_(1.0),
    free_threshold_(-1.0)
{
  setResolution(resolution);
}

void OccupancyGrid3D::setResolution(double resolution)
{
  if (resolution <= 0.0 || !std::isfinite(resolution))
  {
    resolution = 0.2;
  }
  resolution_ = resolution;
  inv_resolution_ = 1.0 / resolution_;
  clear();
}

void OccupancyGrid3D::setLogOddsParams(double hit,
                                       double miss,
                                       double min_l,
                                       double max_l,
                                       double occupied_th,
                                       double free_th)
{
  log_odds_hit_ = hit;
  log_odds_miss_ = miss;
  log_odds_min_ = std::min(min_l, max_l);
  log_odds_max_ = std::max(min_l, max_l);
  occupied_threshold_ = occupied_th;
  free_threshold_ = free_th;
}

VoxelKey OccupancyGrid3D::pointToKey(const Eigen::Vector3d& p) const
{
  return VoxelKey{
      static_cast<int>(std::floor(p.x() * inv_resolution_)),
      static_cast<int>(std::floor(p.y() * inv_resolution_)),
      static_cast<int>(std::floor(p.z() * inv_resolution_))};
}

Eigen::Vector3d OccupancyGrid3D::keyToCenter(const VoxelKey& key) const
{
  return Eigen::Vector3d((static_cast<double>(key.x) + 0.5) * resolution_,
                         (static_cast<double>(key.y) + 0.5) * resolution_,
                         (static_cast<double>(key.z) + 0.5) * resolution_);
}

void OccupancyGrid3D::updateRay(const Eigen::Vector3d& sensor_origin,
                                const Eigen::Vector3d& hit_point,
                                double max_ray_length)
{
  if (!isFiniteVector(sensor_origin) || !isFiniteVector(hit_point) ||
      max_ray_length <= 0.0 || !std::isfinite(max_ray_length))
  {
    return;
  }

  Eigen::Vector3d ray = hit_point - sensor_origin;
  const double range = ray.norm();
  if (range < std::numeric_limits<double>::epsilon())
  {
    updateOccupiedVoxel(pointToKey(hit_point));
    return;
  }

  const Eigen::Vector3d dir = ray / range;
  const bool has_hit = range <= max_ray_length;
  const double trace_length = has_hit ? range : max_ray_length;
  const VoxelKey hit_key = pointToKey(hit_point);

  // Conservative discrete ray-casting: mark sampled voxels before the endpoint as free.
  const double step = std::max(resolution_ * 0.5, 1e-6);
  const int num_steps = static_cast<int>(std::floor(trace_length / step));
  for (int i = 0; i <= num_steps; ++i)
  {
    const double s = std::min(static_cast<double>(i) * step, trace_length);
    const Eigen::Vector3d sample = sensor_origin + dir * s;
    const VoxelKey key = pointToKey(sample);
    if (has_hit && key == hit_key)
    {
      continue;
    }
    updateFreeVoxel(key);
  }

  if (has_hit)
  {
    updateOccupiedVoxel(hit_key);
  }
}

void OccupancyGrid3D::updatePointCloud(
    const Eigen::Vector3d& sensor_origin,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
    double max_ray_length)
{
  if (!cloud_world || cloud_world->empty() || !isFiniteVector(sensor_origin))
  {
    return;
  }

  for (const auto& point : cloud_world->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }
    updateRay(sensor_origin, Eigen::Vector3d(point.x, point.y, point.z), max_ray_length);
  }
}

bool OccupancyGrid3D::isOccupied(const Eigen::Vector3d& p) const
{
  const auto it = voxels_.find(pointToKey(p));
  return it != voxels_.end() && isOccupiedData(it->second);
}

bool OccupancyGrid3D::isFree(const Eigen::Vector3d& p) const
{
  const auto it = voxels_.find(pointToKey(p));
  return it != voxels_.end() && isFreeData(it->second);
}

bool OccupancyGrid3D::isUnknown(const Eigen::Vector3d& p) const
{
  const auto it = voxels_.find(pointToKey(p));
  return it == voxels_.end() || (!isOccupiedData(it->second) && !isFreeData(it->second));
}

std::vector<Eigen::Vector3d> OccupancyGrid3D::getOccupiedCenters() const
{
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(voxels_.size());
  for (const auto& item : voxels_)
  {
    if (isOccupiedData(item.second))
    {
      centers.push_back(keyToCenter(item.first));
    }
  }
  return centers;
}

std::vector<Eigen::Vector3d> OccupancyGrid3D::getFreeCenters() const
{
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(voxels_.size());
  for (const auto& item : voxels_)
  {
    if (isFreeData(item.second))
    {
      centers.push_back(keyToCenter(item.first));
    }
  }
  return centers;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OccupancyGrid3D::getOccupiedCloud() const
{
  return centersToCloud(getOccupiedCenters(), 100.0);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OccupancyGrid3D::getFreeCloud() const
{
  return centersToCloud(getFreeCenters(), 20.0);
}

std::vector<Eigen::Vector3d> OccupancyGrid3D::queryOccupiedInSphere(
    const Eigen::Vector3d& center,
    double radius) const
{
  std::vector<Eigen::Vector3d> result;
  if (!isFiniteVector(center) || radius <= 0.0 || !std::isfinite(radius))
  {
    return result;
  }

  const Eigen::Vector3d min_point = center - Eigen::Vector3d::Constant(radius);
  const Eigen::Vector3d max_point = center + Eigen::Vector3d::Constant(radius);
  const VoxelKey min_key = pointToKey(min_point);
  const VoxelKey max_key = pointToKey(max_point);
  const double radius_sq = radius * radius;

  for (int ix = min_key.x; ix <= max_key.x; ++ix)
  {
    for (int iy = min_key.y; iy <= max_key.y; ++iy)
    {
      for (int iz = min_key.z; iz <= max_key.z; ++iz)
      {
        const VoxelKey key{ix, iy, iz};
        const auto it = voxels_.find(key);
        if (it == voxels_.end() || !isOccupiedData(it->second))
        {
          continue;
        }
        const Eigen::Vector3d voxel_center = keyToCenter(key);
        if ((voxel_center - center).squaredNorm() <= radius_sq)
        {
          result.push_back(voxel_center);
        }
      }
    }
  }

  return result;
}

std::vector<Eigen::Vector3d> OccupancyGrid3D::queryOccupiedInCylinder(
    const Eigen::Vector3d& base_center,
    const Eigen::Vector3d& axis_dir,
    double radius,
    double height) const
{
  std::vector<Eigen::Vector3d> result;
  if (!isFiniteVector(base_center) || !isFiniteVector(axis_dir) ||
      radius <= 0.0 || height <= 0.0 || !std::isfinite(radius) || !std::isfinite(height))
  {
    return result;
  }

  const double axis_norm = axis_dir.norm();
  if (axis_norm < std::numeric_limits<double>::epsilon())
  {
    return result;
  }
  const Eigen::Vector3d axis = axis_dir / axis_norm;

  for (const auto& item : voxels_)
  {
    if (!isOccupiedData(item.second))
    {
      continue;
    }

    const Eigen::Vector3d p = keyToCenter(item.first);
    const Eigen::Vector3d v = p - base_center;
    const double s = v.dot(axis);
    if (s < 0.0 || s > height)
    {
      continue;
    }

    const double radial_distance = (v - s * axis).norm();
    if (radial_distance <= radius)
    {
      result.push_back(p);
    }
  }

  return result;
}

std::vector<Eigen::Vector3d> OccupancyGrid3D::queryOccupiedInRing(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& normal,
    double radius,
    double ring_width,
    double angle_step_rad) const
{
  std::vector<Eigen::Vector3d> result;
  if (!isFiniteVector(center) || !isFiniteVector(normal) ||
      radius < 0.0 || ring_width < 0.0 || !std::isfinite(radius) ||
      !std::isfinite(ring_width))
  {
    return result;
  }

  const double normal_norm = normal.norm();
  if (normal_norm < std::numeric_limits<double>::epsilon())
  {
    return result;
  }

  const Eigen::Vector3d n = normal / normal_norm;
  const Eigen::Vector3d ref =
      (std::abs(n.dot(Eigen::Vector3d::UnitX())) < 0.9) ? Eigen::Vector3d::UnitX()
                                                        : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d e1 = n.cross(ref).normalized();
  const Eigen::Vector3d e2 = n.cross(e1).normalized();

  const double theta_step =
      (angle_step_rad > 0.0 && std::isfinite(angle_step_rad)) ? angle_step_rad : kPi / 180.0;
  const double r_min = std::max(0.0, radius - ring_width * 0.5);
  const double r_max = radius + ring_width * 0.5;
  const double r_step = std::max(resolution_, 1e-6);

  std::unordered_set<VoxelKey, VoxelKeyHash> unique_keys;
  for (double theta = 0.0; theta < 2.0 * kPi; theta += theta_step)
  {
    if (r_max <= r_min + 1e-9)
    {
      const Eigen::Vector3d p = center + radius * std::cos(theta) * e1 +
                                radius * std::sin(theta) * e2;
      const VoxelKey key = pointToKey(p);
      const auto it = voxels_.find(key);
      if (it != voxels_.end() && isOccupiedData(it->second))
      {
        unique_keys.insert(key);
      }
      continue;
    }

    for (double r = r_min; r <= r_max + 1e-9; r += r_step)
    {
      const Eigen::Vector3d p = center + r * std::cos(theta) * e1 +
                                r * std::sin(theta) * e2;
      const VoxelKey key = pointToKey(p);
      const auto it = voxels_.find(key);
      if (it != voxels_.end() && isOccupiedData(it->second))
      {
        unique_keys.insert(key);
      }
    }
  }

  result.reserve(unique_keys.size());
  for (const auto& key : unique_keys)
  {
    result.push_back(keyToCenter(key));
  }
  return result;
}

void OccupancyGrid3D::clear()
{
  voxels_.clear();
}

std::size_t OccupancyGrid3D::size() const
{
  return voxels_.size();
}

std::size_t OccupancyGrid3D::countOccupied() const
{
  std::size_t count = 0;
  for (const auto& item : voxels_)
  {
    if (isOccupiedData(item.second))
    {
      ++count;
    }
  }
  return count;
}

std::size_t OccupancyGrid3D::countFree() const
{
  std::size_t count = 0;
  for (const auto& item : voxels_)
  {
    if (isFreeData(item.second))
    {
      ++count;
    }
  }
  return count;
}

bool OccupancyGrid3D::isOccupiedData(const VoxelData& data) const
{
  return data.log_odds >= occupied_threshold_;
}

bool OccupancyGrid3D::isFreeData(const VoxelData& data) const
{
  return data.log_odds <= free_threshold_;
}

void OccupancyGrid3D::updateFreeVoxel(const VoxelKey& key)
{
  VoxelData& data = voxels_[key];
  data.log_odds = clampLogOdds(data.log_odds + log_odds_miss_);
  data.last_update = ros::Time::now();
  ++data.miss_count;
}

void OccupancyGrid3D::updateOccupiedVoxel(const VoxelKey& key)
{
  VoxelData& data = voxels_[key];
  data.log_odds = clampLogOdds(data.log_odds + log_odds_hit_);
  data.last_update = ros::Time::now();
  ++data.hit_count;
}

double OccupancyGrid3D::clampLogOdds(double value) const
{
  return std::max(log_odds_min_, std::min(log_odds_max_, value));
}

}  // namespace wtb_pointcloud_mapping
