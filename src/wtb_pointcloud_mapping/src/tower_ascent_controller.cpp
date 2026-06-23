#include "wtb_pointcloud_mapping/tower_ascent_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wtb_pointcloud_mapping
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi)  angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool isFiniteVector(const Eigen::Vector3d& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
}  // namespace

TowerAscentController::TowerAscentController() = default;

void TowerAscentController::setConfig(const Config& config)
{
  config_ = config;
  config_.target_distance_from_axis = std::max(1.0, config_.target_distance_from_axis);
  config_.tower_surface_clearance = std::max(0.5, config_.tower_surface_clearance);
  config_.ascent_speed = std::max(0.1, std::min(5.0, config_.ascent_speed));
  config_.spiral_revolution_time = std::max(10.0, config_.spiral_revolution_time);
  config_.publish_rate = std::max(5.0, std::min(50.0, config_.publish_rate));
  config_.horizontal_arrival_tolerance = std::max(0.1, config_.horizontal_arrival_tolerance);
  config_.vertical_arrival_tolerance = std::max(0.1, config_.vertical_arrival_tolerance);
}

void TowerAscentController::reset()
{
  state_ = State();
  initialized_ = false;
  last_setpoint_ = Setpoint();
}

TowerAscentController::Setpoint TowerAscentController::update(
    const TowerAxisEstimator::LineModel& smoothed_axis,
    const TowerAxisEstimator::LineModel& global_axis,
    const Eigen::Vector3d& drone_position,
    const std::vector<Eigen::Vector3d>& control_points,
    double tower_radius_estimate,
    const ros::Time& stamp,
    bool axis_valid,
    bool jump_rejected)
{
  if (!config_.enable)
  {
    state_.phase = State::IDLE;
    return Setpoint();
  }

  const TowerAxisEstimator::LineModel& axis = smoothed_axis.valid ? smoothed_axis : global_axis;

  if (!axis.valid || !isFiniteVector(drone_position))
  {
    state_.axis_lost_count++;
    if (state_.axis_lost_count > 50)  // ~2.5s at 20Hz
    {
      state_.axis_lost = true;
      state_.phase = State::HOLD;
    }
    return last_setpoint_.valid ? last_setpoint_ : Setpoint();
  }
  state_.axis_lost_count = 0;
  state_.axis_lost = false;

  if (!initialized_)
  {
    initial_drone_z_ = drone_position.z();
    min_z_limit_ = std::max(config_.min_height, initial_drone_z_ - config_.height_padding_below);
    if (!control_points.empty())
    {
      double cp_max_z = -std::numeric_limits<double>::max();
      for (const auto& cp : control_points)
      {
        if (std::isfinite(cp.z()))
          cp_max_z = std::max(cp_max_z, cp.z());
      }
      max_z_limit_ = std::min(config_.max_height, cp_max_z + config_.height_padding_above);
    }
    else
    {
      max_z_limit_ = config_.max_height;
    }
    max_z_limit_ = std::max(min_z_limit_ + 5.0, max_z_limit_);
    state_.current_target_z = min_z_limit_;
    state_.phase = State::APPROACH;
    state_.phase_start_time = stamp;
    initialized_ = true;
  }

  if (jump_rejected)
  {
    return last_setpoint_.valid ? last_setpoint_ : Setpoint();
  }

  const double safe_radius = computeTowerRadiusEstimate(axis, control_points, tower_radius_estimate);

  Setpoint sp;
  if (config_.strategy == "spiral")
  {
    sp = computeSpiralSetpoint(axis, drone_position, safe_radius, stamp);
  }
  else
  {
    sp = computeStepSetpoint(axis, drone_position, safe_radius, stamp);
  }

  if (sp.valid)
  {
    last_setpoint_ = sp;
  }
  return sp;
}

double TowerAscentController::computeTowerRadiusEstimate(
    const TowerAxisEstimator::LineModel& axis,
    const std::vector<Eigen::Vector3d>& control_points,
    double tower_radius_estimate) const
{
  double radius = tower_radius_estimate;
  if (radius <= 0.0 && !control_points.empty())
  {
    radius = 1.0;  // default tower radius guess
  }
  radius = std::max(0.3, radius);
  return config_.target_distance_from_axis + radius + config_.tower_surface_clearance;
}

TowerAscentController::Setpoint TowerAscentController::computeSpiralSetpoint(
    const TowerAxisEstimator::LineModel& axis,
    const Eigen::Vector3d& drone_position,
    double orbit_radius,
    const ros::Time& stamp)
{
  (void)drone_position;
  Setpoint sp;

  const double dt = (stamp - state_.phase_start_time).toSec();
  if (dt <= 0.0)
  {
    state_.phase_start_time = stamp;
    return last_setpoint_.valid ? last_setpoint_ : sp;
  }

  const double vz = config_.ascent_speed;
  const double target_z_raw = state_.current_target_z + vz * dt;
  const double target_z = std::max(min_z_limit_, std::min(max_z_limit_, target_z_raw));
  state_.current_target_z = target_z;

  if (target_z >= max_z_limit_ - 0.1)
  {
    state_.phase = State::HOLD;
    state_.current_target_z = max_z_limit_;
  }
  else if (state_.phase != State::HOLD)
  {
    state_.phase = State::ASCEND;
  }

  const double omega = 2.0 * kPi / config_.spiral_revolution_time;
  state_.orbit_angle = normalizeAngle(state_.orbit_angle + omega * dt);

  const Eigen::Vector3d axis_at_target =
      axis.point + axis.direction * ((target_z - axis.point.z()) / std::max(1e-6, axis.direction.z()));

  const double angle = state_.orbit_angle;
  const Eigen::Vector3d offset(orbit_radius * std::cos(angle),
                                orbit_radius * std::sin(angle),
                                0.0);

  sp.position = axis_at_target + offset;
  sp.position.z() = target_z;
  sp.yaw = computeYawToFaceAxis(sp.position, axis_at_target, axis);
  sp.valid = true;

  if (state_.phase == State::HOLD)
  {
    sp.position = last_setpoint_.valid ? last_setpoint_.position : sp.position;
    sp.position.z() = max_z_limit_;
  }

  return sp;
}

TowerAscentController::Setpoint TowerAscentController::computeStepSetpoint(
    const TowerAxisEstimator::LineModel& axis,
    const Eigen::Vector3d& drone_position,
    double orbit_radius,
    const ros::Time& stamp)
{
  Setpoint sp;

  const Eigen::Vector3d axis_point = projectToAxis(drone_position, axis);

  const double dt = (stamp - state_.phase_start_time).toSec();
  if (dt <= 0.0)
  {
    state_.phase_start_time = stamp;
    return last_setpoint_.valid ? last_setpoint_ : sp;
  }

  const double target_z = state_.current_target_z;

  const bool at_target_z = std::abs(drone_position.z() - target_z) < config_.vertical_arrival_tolerance;
  const double horz_dist = (Eigen::Vector2d(drone_position.x(), drone_position.y()) -
                            Eigen::Vector2d(axis_point.x(), axis_point.y())).norm();
  const bool at_target_xy = std::abs(horz_dist - orbit_radius) < config_.horizontal_arrival_tolerance;

  if (at_target_z && at_target_xy && state_.phase == State::ASCEND)
  {
    const double step = 3.0;
    state_.current_target_z = std::min(max_z_limit_, target_z + step);
    state_.phase_start_time = stamp;
    if (state_.current_target_z >= max_z_limit_ - 0.1)
    {
      state_.phase = State::HOLD;
    }
  }

  const double new_target_z = state_.current_target_z;
  const Eigen::Vector3d axis_at_target =
      axis.point + axis.direction * ((new_target_z - axis.point.z()) / std::max(1e-6, axis.direction.z()));

  const Eigen::Vector2d drone_xy(drone_position.x(), drone_position.y());
  const Eigen::Vector2d axis_xy(axis_point.x(), axis_point.y());
  Eigen::Vector2d dir_to_axis = axis_xy - drone_xy;
  const double dir_norm = dir_to_axis.norm();
  if (dir_norm < 1e-6)
  {
    dir_to_axis = Eigen::Vector2d(1.0, 0.0);
  }
  else
  {
    dir_to_axis /= dir_norm;
  }

  const Eigen::Vector2d target_xy = axis_xy - dir_to_axis * orbit_radius;

  sp.position = Eigen::Vector3d(target_xy.x(), target_xy.y(), new_target_z);
  sp.yaw = computeYawToFaceAxis(sp.position, axis_at_target, axis);
  sp.valid = true;

  if (state_.phase == State::IDLE)
  {
    state_.phase = State::APPROACH;
    state_.phase_start_time = stamp;
  }
  else if (state_.phase == State::APPROACH && at_target_xy)
  {
    state_.phase = State::ASCEND;
    state_.current_target_z = std::max(min_z_limit_, drone_position.z());
    state_.phase_start_time = stamp;
  }

  return sp;
}

Eigen::Vector3d TowerAscentController::projectToAxis(
    const Eigen::Vector3d& point,
    const TowerAxisEstimator::LineModel& axis) const
{
  if (!axis.valid || !isFiniteVector(point) || !isFiniteVector(axis.point) ||
      !isFiniteVector(axis.direction) || axis.direction.norm() < 1e-9)
  {
    return point;
  }
  const Eigen::Vector3d d = axis.direction.normalized();
  return axis.point + d * ((point - axis.point).dot(d));
}

double TowerAscentController::computeYawToFaceAxis(
    const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& target_pos,
    const TowerAxisEstimator::LineModel& axis) const
{
  (void)target_pos;
  const Eigen::Vector3d axis_point = projectToAxis(drone_pos, axis);
  return std::atan2(axis_point.y() - drone_pos.y(),
                    axis_point.x() - drone_pos.x());
}

}  // namespace wtb_pointcloud_mapping
