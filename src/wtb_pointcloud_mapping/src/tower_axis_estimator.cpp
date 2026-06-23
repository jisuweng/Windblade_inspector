#include "wtb_pointcloud_mapping/tower_axis_estimator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

#include <Eigen/Eigenvalues>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace wtb_pointcloud_mapping
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool isFiniteVector(const Eigen::Vector3d& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

std::size_t cloudSize(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  return cloud ? cloud->points.size() : 0;
}

std::string boolText(bool value)
{
  return value ? "true" : "false";
}

double radToDeg(double rad)
{
  return rad * 180.0 / kPi;
}
}  // namespace

TowerAxisEstimator::TowerAxisEstimator() = default;

void TowerAxisEstimator::setConfig(const Config& config)
{
  config_ = config;

  if (!isFiniteVector(config_.vertical_axis) || config_.vertical_axis.norm() < 1e-9)
  {
    config_.vertical_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  config_.vertical_axis.normalize();

  config_.voxel_leaf_size = std::max(0.0, config_.voxel_leaf_size);
  if (config_.max_z < config_.min_z)
  {
    std::swap(config_.min_z, config_.max_z);
  }

  config_.roi_radius_xy = std::max(0.0, config_.roi_radius_xy);
  config_.roi_z_below = std::max(0.0, config_.roi_z_below);
  config_.roi_z_above = std::max(0.0, config_.roi_z_above);

  config_.min_range_from_drone = std::max(0.0, config_.min_range_from_drone);
  config_.max_range_from_drone =
      std::max(config_.min_range_from_drone, config_.max_range_from_drone);

  config_.slice_height = std::max(0.1, config_.slice_height);
  config_.slice_min_points = std::max(5, config_.slice_min_points);
  config_.slice_max_points = std::max(config_.slice_min_points, config_.slice_max_points);

  config_.tower_radius_min = std::max(0.1, config_.tower_radius_min);
  config_.tower_radius_max = std::max(config_.tower_radius_min, config_.tower_radius_max);
  config_.circle_fit_residual_threshold = std::max(0.01, config_.circle_fit_residual_threshold);
  config_.min_valid_slices = std::max(2, config_.min_valid_slices);

  config_.max_center_xy_deviation = std::max(0.1, config_.max_center_xy_deviation);
  config_.max_center_jump_between_slices = std::max(0.05, config_.max_center_jump_between_slices);

  config_.vertical_angle_threshold_deg =
      std::max(0.0, std::min(90.0, config_.vertical_angle_threshold_deg));

  config_.axis_point_alpha = std::max(0.0, std::min(1.0, config_.axis_point_alpha));
  config_.axis_dir_alpha = std::max(0.0, std::min(1.0, config_.axis_dir_alpha));

  config_.max_axis_lateral_jump = std::max(0.01, config_.max_axis_lateral_jump);
  config_.max_axis_angle_jump_deg = std::max(0.1, config_.max_axis_angle_jump_deg);
  config_.max_reject_count_before_reset = std::max(1, config_.max_reject_count_before_reset);

  config_.min_control_point_spacing = std::max(0.0, config_.min_control_point_spacing);
  config_.max_control_point_jump = std::max(0.0, config_.max_control_point_jump);
  config_.fit_global_axis_min_points = std::max(2, config_.fit_global_axis_min_points);

  config_.ransac_distance_threshold = std::max(1e-6, config_.ransac_distance_threshold);
  config_.ransac_max_iterations = std::max(1, config_.ransac_max_iterations);
  config_.ransac_min_inliers = std::max(1, config_.ransac_min_inliers);
  config_.ransac_parallel_eps_angle_deg =
      std::max(0.1, std::min(45.0, config_.ransac_parallel_eps_angle_deg));
}

TowerAxisEstimator::Result TowerAxisEstimator::process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
    const Eigen::Vector3d& drone_position_world)
{
  Result result;
  result.candidate_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  result.slice_center_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  if (!config_.enable)
  {
    result.debug_msg = "[TowerAxis] disabled";
    return result;
  }

  result.candidate_cloud = preprocessCloud(cloud_world, drone_position_world);

  const bool use_slice_method = (config_.method == "slice_center_axis");

  if (use_slice_method)
  {
    const auto raw_slices = splitCloudByHeight(result.candidate_cloud);

    std::vector<SliceCenter> all_centers;
    int circle_fit_success = 0;
    int centroid_fallback = 0;

    for (const auto& slice_points : raw_slices)
    {
      if (slice_points.empty())
      {
        continue;
      }

      double z_min = std::numeric_limits<double>::max();
      double z_max = std::numeric_limits<double>::lowest();
      for (const auto& p : slice_points)
      {
        z_min = std::min(z_min, static_cast<double>(p.z));
        z_max = std::max(z_max, static_cast<double>(p.z));
      }

      SliceCenter sc;
      if (estimateSliceCenter(slice_points, z_min, z_max, sc))
      {
        if (sc.valid && !sc.used_centroid_fallback)
        {
          circle_fit_success++;
        }
        if (sc.valid && sc.used_centroid_fallback)
        {
          centroid_fallback++;
        }
        all_centers.push_back(sc);
      }
    }

    std::vector<SliceCenter> valid_centers = all_centers;
    if (config_.center_outlier_reject_enable)
    {
      valid_centers = rejectOutlierSliceCenters(all_centers);
    }

    result.slice_centers = valid_centers;

    for (const auto& sc : valid_centers)
    {
      if (!sc.valid)
      {
        continue;
      }
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(sc.center.x());
      pt.y = static_cast<float>(sc.center.y());
      pt.z = static_cast<float>(sc.center.z());
      pt.intensity = sc.used_centroid_fallback ? 0.3f : 1.0f;
      result.slice_center_cloud->points.push_back(pt);
    }
    result.slice_center_cloud->height = 1;
    result.slice_center_cloud->width =
        static_cast<std::uint32_t>(result.slice_center_cloud->points.size());
    result.slice_center_cloud->is_dense = false;

    LineModel local_axis;
    bool local_axis_valid = fitAxisFromSliceCenters(valid_centers, local_axis);

    if (local_axis_valid)
    {
      result.local_line = local_axis;

      bool jump_rejected = false;
      LineModel output_axis;
      bool smoothed_ok = applyTemporalSmoothingAndJumpReject(local_axis, output_axis, jump_rejected);

      result.jump_rejected = jump_rejected;
      if (smoothed_ok)
      {
        result.smoothed_axis = output_axis;
        result.success = true;
      }
      else if (has_smoothed_axis_)
      {
        result.smoothed_axis = smoothed_axis_;
        result.success = true;
      }
    }
    else if (has_smoothed_axis_)
    {
      ++consecutive_reject_count_;
      result.smoothed_axis = smoothed_axis_;
      result.success = true;
    }
    else
    {
      ++consecutive_reject_count_;
      result.success = false;
    }

    if (result.success && result.smoothed_axis.valid && isFiniteVector(drone_position_world))
    {
      result.control_point =
          projectPointToLine(drone_position_world, result.smoothed_axis);
      if (shouldAddControlPoint(result.control_point, result.jump_rejected))
      {
        control_points_.push_back(result.control_point);
        result.added_new_control_point = true;
      }
    }

    result.control_points = control_points_;
    LineModel global_axis;
    if (fitGlobalAxisFromControlPoints(global_axis))
    {
      result.global_axis = global_axis;
    }

    int valid_slice_count = 0;
    for (const auto& sc : valid_centers)
    {
      if (sc.valid)
        valid_slice_count++;
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "[TowerAxis]\n"
       << "method: " << config_.method << "\n"
       << "candidate points: " << cloudSize(result.candidate_cloud) << "\n"
       << "raw slices: " << all_centers.size() << "\n"
       << "valid slice centers: " << valid_slice_count << "\n"
       << "circle fit success: " << circle_fit_success << "\n"
       << "centroid fallback: " << centroid_fallback << "\n"
       << "local axis valid: " << boolText(local_axis_valid) << "\n"
       << "smoothed axis valid: " << boolText(result.smoothed_axis.valid) << "\n"
       << "jump rejected: " << boolText(result.jump_rejected) << "\n"
       << "control points: " << result.control_points.size() << "\n"
       << "added control point: " << boolText(result.added_new_control_point) << "\n"
       << "global axis valid: " << boolText(result.global_axis.valid) << "\n"
       << "reject count: " << consecutive_reject_count_;
    result.debug_msg = ss.str();
  }
  else
  {
    LineModel fitted_line;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    bool fit_ok = fitLineRansac(result.candidate_cloud, fitted_line, inliers);
    if (fit_ok)
    {
      isNearlyVertical(fitted_line);
    }

    result.local_line = fitted_line;

    if (fit_ok && fitted_line.valid)
    {
      result.success = true;
      last_valid_line_ = fitted_line;
      has_last_valid_line_ = true;
      consecutive_reject_count_ = 0;
    }
    else
    {
      ++consecutive_reject_count_;
      const bool can_use_last =
          has_last_valid_line_ &&
          consecutive_reject_count_ <= config_.max_reject_count_before_reset;

      if (can_use_last)
      {
        result.success = true;
        result.local_line = last_valid_line_;
      }
    }

    if (result.success && result.local_line.valid && isFiniteVector(drone_position_world))
    {
      result.control_point = projectPointToLine(drone_position_world, result.local_line);
      if (shouldAddControlPoint(result.control_point, false))
      {
        control_points_.push_back(result.control_point);
        result.added_new_control_point = true;
      }
    }

    result.control_points = control_points_;
    LineModel global_axis;
    if (fitGlobalAxisFromControlPoints(global_axis))
    {
      result.global_axis = global_axis;
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "[TowerAxis]\n"
       << "method: ransac_line_debug\n"
       << "candidate points: " << cloudSize(result.candidate_cloud) << "\n"
       << "ransac inliers: " << fitted_line.inlier_count << "\n"
       << "vertical angle: " << fitted_line.vertical_angle_deg << " deg\n"
       << "local line valid: " << boolText(result.local_line.valid) << "\n"
       << "control points: " << result.control_points.size() << "\n"
       << "added control point: " << boolText(result.added_new_control_point) << "\n"
       << "global axis valid: " << boolText(result.global_axis.valid) << "\n"
       << "reject count: " << consecutive_reject_count_;
    result.debug_msg = ss.str();
  }

  if (config_.ransac_debug_enable && result.candidate_cloud &&
      !result.candidate_cloud->empty())
  {
    pcl::PointIndices::Ptr debug_inliers(new pcl::PointIndices);
    fitLineRansac(result.candidate_cloud, result.raw_ransac_line, debug_inliers);
  }

  return result;
}

void TowerAxisEstimator::clear()
{
  has_smoothed_axis_ = false;
  smoothed_axis_ = LineModel();
  has_last_valid_line_ = false;
  last_valid_line_ = LineModel();
  consecutive_reject_count_ = 0;
  control_points_.clear();
}

std::vector<Eigen::Vector3d> TowerAxisEstimator::getControlPoints() const
{
  return control_points_;
}

TowerAxisEstimator::LineModel TowerAxisEstimator::getGlobalAxis() const
{
  LineModel axis;
  fitGlobalAxisFromControlPoints(axis);
  return axis;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TowerAxisEstimator::preprocessCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
    const Eigen::Vector3d& drone_position_world) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  filtered->height = 1;
  filtered->is_dense = false;

  if (!cloud_world || cloud_world->empty() || !isFiniteVector(drone_position_world))
  {
    filtered->width = 0;
    return filtered;
  }

  const double roi_radius_xy2 = config_.roi_radius_xy * config_.roi_radius_xy;
  const double min_range2 = config_.min_range_from_drone * config_.min_range_from_drone;
  const double max_range2 = config_.max_range_from_drone * config_.max_range_from_drone;

  filtered->points.reserve(cloud_world->points.size());
  for (const auto& point : cloud_world->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }

    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (p.z() < config_.min_z || p.z() > config_.max_z)
    {
      continue;
    }

    const Eigen::Vector3d delta = p - drone_position_world;
    if (config_.use_drone_roi)
    {
      const double xy2 = delta.x() * delta.x() + delta.y() * delta.y();
      if (xy2 > roi_radius_xy2)
      {
        continue;
      }
      if (p.z() < drone_position_world.z() - config_.roi_z_below ||
          p.z() > drone_position_world.z() + config_.roi_z_above)
      {
        continue;
      }
    }

    const double range2 = delta.squaredNorm();
    if (range2 < min_range2 || range2 > max_range2)
    {
      continue;
    }

    filtered->points.push_back(point);
  }

  filtered->height = 1;
  filtered->width = static_cast<std::uint32_t>(filtered->points.size());
  filtered->is_dense = false;

  if (config_.voxel_leaf_size <= 0.0 || filtered->empty())
  {
    return filtered;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(filtered);
  const float leaf = static_cast<float>(config_.voxel_leaf_size);
  voxel_grid.setLeafSize(leaf, leaf, leaf);

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid.filter(*downsampled);
  downsampled->height = 1;
  downsampled->width = static_cast<std::uint32_t>(downsampled->points.size());
  downsampled->is_dense = false;
  return downsampled;
}

std::vector<TowerAxisEstimator::AlignedPointVector> TowerAxisEstimator::splitCloudByHeight(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
{
  std::vector<AlignedPointVector> slices;

  if (!cloud || cloud->empty())
  {
    return slices;
  }

  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
  for (const auto& p : cloud->points)
  {
    if (std::isfinite(p.z))
    {
      min_z = std::min(min_z, static_cast<double>(p.z));
      max_z = std::max(max_z, static_cast<double>(p.z));
    }
  }

  if (max_z <= min_z)
  {
    return slices;
  }

  const double slice_h = config_.slice_height;
  const int num_slices = static_cast<int>(std::ceil((max_z - min_z) / slice_h));
  slices.resize(num_slices);

  for (const auto& p : cloud->points)
  {
    if (!std::isfinite(p.z))
    {
      continue;
    }
    const int idx = static_cast<int>(std::floor((p.z - min_z) / slice_h));
    const int clamped = std::max(0, std::min(num_slices - 1, idx));
    slices[clamped].push_back(p);
  }

  std::vector<AlignedPointVector> valid_slices;
  valid_slices.reserve(slices.size());
  for (auto& slice : slices)
  {
    const int count = static_cast<int>(slice.size());
    if (count < config_.slice_min_points)
    {
      continue;
    }

    if (count > config_.slice_max_points)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
      temp->points = std::move(slice);
      temp->height = 1;
      temp->width = static_cast<std::uint32_t>(temp->points.size());
      temp->is_dense = false;

      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(temp);
      const float leaf = static_cast<float>(config_.slice_height * 0.2);
      vg.setLeafSize(leaf, leaf, leaf);

      pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
      vg.filter(*ds);

      AlignedPointVector ds_slice(ds->points.begin(), ds->points.end());
      if (static_cast<int>(ds_slice.size()) >= config_.slice_min_points)
      {
        valid_slices.push_back(std::move(ds_slice));
      }
    }
    else
    {
      valid_slices.push_back(std::move(slice));
    }
  }

  return valid_slices;
}

bool TowerAxisEstimator::fitCircle2DLeastSquares(
    const AlignedPointVector& slice_points,
    Eigen::Vector2d& center_xy,
    double& radius,
    double& residual) const
{
  center_xy = Eigen::Vector2d::Zero();
  radius = 0.0;
  residual = std::numeric_limits<double>::max();

  const int n = static_cast<int>(slice_points.size());
  if (n < 6)
  {
    return false;
  }

  Eigen::MatrixXd A(n, 3);
  Eigen::VectorXd b(n);

  for (int i = 0; i < n; ++i)
  {
    const double x = slice_points[i].x;
    const double y = slice_points[i].y;
    A(i, 0) = x;
    A(i, 1) = y;
    A(i, 2) = 1.0;
    b(i) = -(x * x + y * y);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::Vector3d params = svd.solve(b);

  const double A_coeff = params(0);
  const double B_coeff = params(1);
  const double C_coeff = params(2);

  const double cx = -A_coeff / 2.0;
  const double cy = -B_coeff / 2.0;
  const double r_sq = (A_coeff * A_coeff + B_coeff * B_coeff) / 4.0 - C_coeff;

  if (r_sq <= 0.0 || !std::isfinite(r_sq))
  {
    return false;
  }

  const double r = std::sqrt(r_sq);
  if (r < config_.tower_radius_min || r > config_.tower_radius_max)
  {
    return false;
  }

  if (config_.tower_radius_prior > 0.0)
  {
    const double prior = config_.tower_radius_prior;
    if (r < prior * 0.3 || r > prior * 3.0)
    {
      return false;
    }
  }

  double sum_err = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const double dx = slice_points[i].x - cx;
    const double dy = slice_points[i].y - cy;
    const double err = std::abs(std::sqrt(dx * dx + dy * dy) - r);
    sum_err += err;
  }
  residual = sum_err / static_cast<double>(n);

  if (residual > config_.circle_fit_residual_threshold)
  {
    return false;
  }

  center_xy = Eigen::Vector2d(cx, cy);
  radius = r;
  return true;
}

bool TowerAxisEstimator::estimateSliceCenter(
    const AlignedPointVector& slice_points,
    double z_min,
    double z_max,
    SliceCenter& slice_center) const
{
  slice_center = SliceCenter();
  slice_center.z_min = z_min;
  slice_center.z_max = z_max;
  slice_center.point_count = static_cast<int>(slice_points.size());

  if (slice_points.empty())
  {
    return false;
  }

  double sum_z = 0.0;
  for (const auto& p : slice_points)
  {
    sum_z += p.z;
  }
  const double mean_z = sum_z / static_cast<double>(slice_points.size());

  if (config_.circle_fit_enable)
  {
    Eigen::Vector2d center_xy;
    double radius = 0.0;
    double residual = 0.0;
    if (fitCircle2DLeastSquares(slice_points, center_xy, radius, residual))
    {
      slice_center.center = Eigen::Vector3d(center_xy.x(), center_xy.y(), mean_z);
      slice_center.radius = radius;
      slice_center.residual = residual;
      slice_center.valid = true;
      slice_center.used_centroid_fallback = false;
      return true;
    }
  }

  if (config_.fallback_to_slice_centroid)
  {
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : slice_points)
    {
      sum_x += p.x;
      sum_y += p.y;
    }
    const double cx = sum_x / static_cast<double>(slice_points.size());
    const double cy = sum_y / static_cast<double>(slice_points.size());

    slice_center.center = Eigen::Vector3d(cx, cy, mean_z);
    slice_center.radius = 0.0;
    slice_center.residual = 999.0;
    slice_center.valid = true;
    slice_center.used_centroid_fallback = true;
    return true;
  }

  return false;
}

std::vector<TowerAxisEstimator::SliceCenter> TowerAxisEstimator::rejectOutlierSliceCenters(
    const std::vector<SliceCenter>& centers) const
{
  if (centers.empty())
  {
    return centers;
  }

  std::vector<SliceCenter> valid_only;
  for (const auto& sc : centers)
  {
    if (sc.valid)
    {
      valid_only.push_back(sc);
    }
  }

  if (valid_only.empty())
  {
    return centers;
  }

  double sum_x = 0.0, sum_y = 0.0;
  int count = 0;
  for (const auto& sc : valid_only)
  {
    if (std::isfinite(sc.center.x()) && std::isfinite(sc.center.y()))
    {
      sum_x += sc.center.x();
      sum_y += sc.center.y();
      count++;
    }
  }

  if (count == 0)
  {
    return centers;
  }

  const double mean_x = sum_x / static_cast<double>(count);
  const double mean_y = sum_y / static_cast<double>(count);

  std::vector<SliceCenter> result;
  result.reserve(valid_only.size());
  for (const auto& sc : valid_only)
  {
    const double dx = sc.center.x() - mean_x;
    const double dy = sc.center.y() - mean_y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= config_.max_center_xy_deviation)
    {
      result.push_back(sc);
    }
  }

  if (result.size() >= 2 && config_.max_center_jump_between_slices > 0.0)
  {
    std::sort(result.begin(), result.end(),
              [](const SliceCenter& a, const SliceCenter& b) {
                return a.center.z() < b.center.z();
              });

    std::vector<SliceCenter> filtered;
    filtered.push_back(result.front());
    for (std::size_t i = 1; i < result.size(); ++i)
    {
      const Eigen::Vector3d& prev = filtered.back().center;
      const Eigen::Vector3d& curr = result[i].center;
      const double jump = std::sqrt(
          (curr.x() - prev.x()) * (curr.x() - prev.x()) +
          (curr.y() - prev.y()) * (curr.y() - prev.y()));
      if (jump <= config_.max_center_jump_between_slices)
      {
        filtered.push_back(result[i]);
      }
    }
    result = std::move(filtered);
  }

  return result;
}

bool TowerAxisEstimator::fitAxisFromSliceCenters(
    const std::vector<SliceCenter>& centers,
    LineModel& axis) const
{
  axis = LineModel();

  int valid_count = 0;
  for (const auto& sc : centers)
  {
    if (sc.valid)
      valid_count++;
  }

  if (valid_count < config_.min_valid_slices)
  {
    return false;
  }

  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  int n = 0;
  for (const auto& sc : centers)
  {
    if (!sc.valid)
      continue;
    if (!std::isfinite(sc.center.x()) || !std::isfinite(sc.center.y()) ||
        !std::isfinite(sc.center.z()))
      continue;
    sum_x += sc.center.x();
    sum_y += sc.center.y();
    sum_z += sc.center.z();
    n++;
  }

  if (n < config_.min_valid_slices)
  {
    return false;
  }

  const Eigen::Vector3d centroid(sum_x / n, sum_y / n, sum_z / n);

  if (config_.force_axis_vertical)
  {
    axis.point = centroid;
    axis.direction = config_.vertical_axis.normalized();
    axis.valid = true;
    axis.inlier_count = n;
    axis.vertical_angle_deg = 0.0;
    return true;
  }

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& sc : centers)
  {
    if (!sc.valid)
      continue;
    const Eigen::Vector3d centered = sc.center - centroid;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(n);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  Eigen::Vector3d direction = solver.eigenvectors().col(2);
  if (!isFiniteVector(direction) || direction.norm() < 1e-9)
  {
    return false;
  }

  direction.normalize();
  if (direction.z() < 0.0)
  {
    direction = -direction;
  }

  const double cos_angle =
      std::abs(std::max(-1.0, std::min(1.0, direction.dot(config_.vertical_axis))));
  const double angle_rad = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
  const double angle_deg = radToDeg(angle_rad);
  if (angle_deg > config_.vertical_angle_threshold_deg)
  {
    return false;
  }

  axis.point = centroid;
  axis.direction = direction;
  axis.valid = true;
  axis.inlier_count = n;
  axis.vertical_angle_deg = angle_deg;
  return true;
}

bool TowerAxisEstimator::applyTemporalSmoothingAndJumpReject(
    const LineModel& new_axis,
    LineModel& output_axis,
    bool& jump_rejected)
{
  jump_rejected = false;

  if (!new_axis.valid)
  {
    output_axis = new_axis;
    return false;
  }

  if (!has_smoothed_axis_)
  {
    smoothed_axis_ = new_axis;
    has_smoothed_axis_ = true;
    consecutive_reject_count_ = 0;
    output_axis = smoothed_axis_;
    return true;
  }

  const double lateral_jump =
      (new_axis.point - smoothed_axis_.point).norm();

  double angle_jump_deg = 0.0;
  {
    const double dot = std::max(-1.0, std::min(1.0,
        new_axis.direction.dot(smoothed_axis_.direction)));
    angle_jump_deg = radToDeg(std::acos(dot));
  }

  if (config_.jump_reject_enable)
  {
    const bool lateral_bad = lateral_jump > config_.max_axis_lateral_jump;
    const bool angle_bad = angle_jump_deg > config_.max_axis_angle_jump_deg;

    if (lateral_bad || angle_bad)
    {
      jump_rejected = true;
      consecutive_reject_count_++;

      if (consecutive_reject_count_ > config_.max_reject_count_before_reset)
      {
        smoothed_axis_ = new_axis;
        consecutive_reject_count_ = 0;
        jump_rejected = false;
        output_axis = smoothed_axis_;
        return true;
      }

      output_axis = smoothed_axis_;
      return true;
    }
  }

  consecutive_reject_count_ = 0;

  if (config_.temporal_smoothing_enable)
  {
    const double pa = config_.axis_point_alpha;
    output_axis.point =
        (1.0 - pa) * smoothed_axis_.point + pa * new_axis.point;

    const double da = config_.axis_dir_alpha;
    Eigen::Vector3d blended_dir =
        (1.0 - da) * smoothed_axis_.direction + da * new_axis.direction;
    if (blended_dir.norm() < 1e-9)
    {
      blended_dir = smoothed_axis_.direction;
    }
    blended_dir.normalize();
    if (blended_dir.z() < 0.0)
    {
      blended_dir = -blended_dir;
    }
    output_axis.direction = blended_dir;
  }
  else
  {
    output_axis.point = new_axis.point;
    output_axis.direction = new_axis.direction;
  }

  if (config_.force_axis_vertical)
  {
    output_axis.direction = config_.vertical_axis.normalized();
    output_axis.vertical_angle_deg = 0.0;
  }
  else
  {
    const double cos_angle =
        std::abs(std::max(-1.0, std::min(1.0,
            output_axis.direction.dot(config_.vertical_axis))));
    output_axis.vertical_angle_deg = radToDeg(std::acos(cos_angle));
  }

  output_axis.valid = true;
  output_axis.inlier_count = new_axis.inlier_count;

  smoothed_axis_ = output_axis;
  return true;
}

bool TowerAxisEstimator::fitLineRansac(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    LineModel& line,
    pcl::PointIndices::Ptr& inliers) const
{
  line = LineModel();
  if (!inliers)
  {
    inliers.reset(new pcl::PointIndices);
  }
  inliers->indices.clear();

  if (!cloud || static_cast<int>(cloud->points.size()) < config_.ransac_min_inliers)
  {
    return false;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(config_.ransac_distance_threshold);
  seg.setMaxIterations(config_.ransac_max_iterations);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (coefficients->values.size() < 6)
  {
    return false;
  }

  const Eigen::Vector3d point(coefficients->values[0],
                              coefficients->values[1],
                              coefficients->values[2]);
  Eigen::Vector3d direction(coefficients->values[3],
                            coefficients->values[4],
                            coefficients->values[5]);

  if (!isFiniteVector(point) || !isFiniteVector(direction) || direction.norm() < 1e-9)
  {
    return false;
  }

  direction.normalize();
  if (direction.z() < 0.0)
  {
    direction = -direction;
  }

  line.point = point;
  line.direction = direction;
  line.inlier_count = static_cast<int>(inliers->indices.size());
  line.valid = false;

  return line.inlier_count >= config_.ransac_min_inliers;
}

bool TowerAxisEstimator::isNearlyVertical(LineModel& line) const
{
  if (!isFiniteVector(line.direction) || line.direction.norm() < 1e-9)
  {
    line.valid = false;
    line.vertical_angle_deg = 999.0;
    return false;
  }

  line.direction.normalize();
  if (line.direction.z() < 0.0)
  {
    line.direction = -line.direction;
  }

  Eigen::Vector3d vertical_axis = config_.vertical_axis;
  if (!isFiniteVector(vertical_axis) || vertical_axis.norm() < 1e-9)
  {
    vertical_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  vertical_axis.normalize();

  const double cos_angle =
      std::abs(std::max(-1.0, std::min(1.0, line.direction.dot(vertical_axis))));
  const double angle_rad = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
  line.vertical_angle_deg = radToDeg(angle_rad);
  line.valid = line.inlier_count >= config_.ransac_min_inliers &&
               line.vertical_angle_deg <= config_.vertical_angle_threshold_deg;
  return line.valid;
}

Eigen::Vector3d TowerAxisEstimator::projectPointToLine(const Eigen::Vector3d& point,
                                                       const LineModel& line) const
{
  if (!line.valid || !isFiniteVector(point) || !isFiniteVector(line.point) ||
      !isFiniteVector(line.direction) || line.direction.norm() < 1e-9)
  {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d direction = line.direction.normalized();
  return line.point + direction * ((point - line.point).dot(direction));
}

bool TowerAxisEstimator::shouldAddControlPoint(const Eigen::Vector3d& new_point,
                                               bool jump_rejected) const
{
  if (!isFiniteVector(new_point))
  {
    return false;
  }

  if (jump_rejected)
  {
    return false;
  }

  if (control_points_.empty())
  {
    return true;
  }

  const Eigen::Vector3d& last = control_points_.back();
  const double distance = (new_point - last).norm();
  if (distance < config_.min_control_point_spacing)
  {
    return false;
  }

  if (config_.max_control_point_jump > 0.0 &&
      distance > config_.max_control_point_jump)
  {
    return false;
  }

  return true;
}

bool TowerAxisEstimator::fitGlobalAxisFromControlPoints(LineModel& axis) const
{
  axis = LineModel();
  if (static_cast<int>(control_points_.size()) < config_.fit_global_axis_min_points)
  {
    return false;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& point : control_points_)
  {
    centroid += point;
  }
  centroid /= static_cast<double>(control_points_.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : control_points_)
  {
    const Eigen::Vector3d centered = point - centroid;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(control_points_.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  Eigen::Vector3d direction = solver.eigenvectors().col(2);
  if (!isFiniteVector(direction) || direction.norm() < 1e-9)
  {
    return false;
  }

  direction.normalize();
  if (direction.z() < 0.0)
  {
    direction = -direction;
  }

  axis.point = centroid;
  axis.direction = direction;
  axis.valid = true;
  axis.inlier_count = static_cast<int>(control_points_.size());

  Eigen::Vector3d vertical_axis = config_.vertical_axis;
  if (!isFiniteVector(vertical_axis) || vertical_axis.norm() < 1e-9)
  {
    vertical_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  vertical_axis.normalize();
  const double cos_angle =
      std::abs(std::max(-1.0, std::min(1.0, axis.direction.dot(vertical_axis))));
  const double angle_rad = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
  axis.vertical_angle_deg = radToDeg(angle_rad);
  return true;
}

}  // namespace wtb_pointcloud_mapping
