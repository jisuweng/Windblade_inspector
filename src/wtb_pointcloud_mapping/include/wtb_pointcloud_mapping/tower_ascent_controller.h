#ifndef WTB_POINTCLOUD_MAPPING_TOWER_ASCENT_CONTROLLER_H
#define WTB_POINTCLOUD_MAPPING_TOWER_ASCENT_CONTROLLER_H

#include <string>
#include <Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "wtb_pointcloud_mapping/tower_axis_estimator.h"

namespace wtb_pointcloud_mapping
{

class TowerAscentController
{
public:
  TowerAscentController();

  struct Config
  {
    bool enable = false;
    std::string strategy = "spiral";   // "spiral" or "step"

    double target_distance_from_axis = 8.0;   // m, horizontal distance from axis
    double tower_surface_clearance = 3.0;      // m, min clearance from tower surface

    double ascent_speed = 0.5;         // m/s vertical
    double spiral_revolution_time = 40.0; // seconds per full orbit

    double min_height = 2.0;           // m, don't descend below this
    double max_height = 80.0;          // m, tower top estimate
    double height_padding_below = 3.0; // start ascent this far below drone's initial z
    double height_padding_above = 5.0; // stop this far above top of control points

    double horizontal_arrival_tolerance = 0.5; // m
    double vertical_arrival_tolerance = 0.3;   // m

    double yaw_rate = 0.3;             // rad/s max yaw rate when facing tower

    double publish_rate = 20.0;        // Hz, must be >2Hz for PX4 OFFBOARD
    std::string setpoint_topic = "/iris_0/mavros/setpoint_position/local";
    std::string setpoint_frame = "map";

    bool publish_path_preview = true;
  };

  struct State
  {
    enum Phase
    {
      IDLE = 0,
      APPROACH = 1,
      ASCEND = 2,
      HOLD = 3,
      DESCEND = 4
    };

    Phase phase = IDLE;
    double current_target_z = 0.0;
    double orbit_angle = 0.0;
    ros::Time phase_start_time;
    bool axis_lost = false;
    int axis_lost_count = 0;
  };

  struct Setpoint
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    bool valid = false;
  };

  void setConfig(const Config& config);
  void reset();

  Setpoint update(const TowerAxisEstimator::LineModel& smoothed_axis,
                  const TowerAxisEstimator::LineModel& global_axis,
                  const Eigen::Vector3d& drone_position,
                  const std::vector<Eigen::Vector3d>& control_points,
                  double tower_radius_estimate,
                  const ros::Time& stamp,
                  bool axis_valid,
                  bool jump_rejected);

  const State& getState() const { return state_; }

private:
  Config config_;
  State state_;

  bool initialized_ = false;
  double initial_drone_z_ = 0.0;
  double min_z_limit_ = 0.0;
  double max_z_limit_ = 0.0;

  Setpoint last_setpoint_;

  double computeTowerRadiusEstimate(
      const TowerAxisEstimator::LineModel& smoothed_axis,
      const std::vector<Eigen::Vector3d>& control_points,
      double tower_radius_estimate) const;

  Setpoint computeSpiralSetpoint(
      const TowerAxisEstimator::LineModel& axis,
      const Eigen::Vector3d& drone_position,
      double radius,
      const ros::Time& stamp);

  Setpoint computeStepSetpoint(
      const TowerAxisEstimator::LineModel& axis,
      const Eigen::Vector3d& drone_position,
      double radius,
      const ros::Time& stamp);

  Eigen::Vector3d projectToAxis(const Eigen::Vector3d& point,
                                const TowerAxisEstimator::LineModel& axis) const;

  double computeYawToFaceAxis(const Eigen::Vector3d& drone_pos,
                              const Eigen::Vector3d& target_pos,
                              const TowerAxisEstimator::LineModel& axis) const;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_TOWER_ASCENT_CONTROLLER_H
