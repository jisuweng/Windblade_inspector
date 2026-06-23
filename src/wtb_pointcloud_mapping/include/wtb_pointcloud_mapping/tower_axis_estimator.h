#ifndef WTB_POINTCLOUD_MAPPING_TOWER_AXIS_ESTIMATOR_H
#define WTB_POINTCLOUD_MAPPING_TOWER_AXIS_ESTIMATOR_H

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

namespace wtb_pointcloud_mapping
{

class TowerAxisEstimator
{
public:
  TowerAxisEstimator();

  struct Config
  {
    bool enable = true;

    std::string method = "slice_center_axis";
    std::string input_cloud_mode = "current";

    double voxel_leaf_size = 0.10;
    double min_z = 0.5;
    double max_z = 120.0;

    bool use_drone_roi = true;
    double roi_radius_xy = 15.0;
    double roi_z_below = 8.0;
    double roi_z_above = 6.0;

    double min_range_from_drone = 1.0;
    double max_range_from_drone = 35.0;

    double slice_height = 0.5;
    int slice_min_points = 40;
    int slice_max_points = 20000;

    bool circle_fit_enable = true;
    double tower_radius_prior = 0.0;
    double tower_radius_min = 0.5;
    double tower_radius_max = 8.0;
    double circle_fit_residual_threshold = 0.35;
    int min_valid_slices = 4;

    bool fallback_to_slice_centroid = true;

    bool center_outlier_reject_enable = true;
    double max_center_xy_deviation = 2.0;
    double max_center_jump_between_slices = 1.5;

    bool force_axis_vertical = true;
    Eigen::Vector3d vertical_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    double vertical_angle_threshold_deg = 8.0;

    bool temporal_smoothing_enable = true;
    double axis_point_alpha = 0.15;
    double axis_dir_alpha = 0.10;

    bool jump_reject_enable = true;
    double max_axis_lateral_jump = 0.8;
    double max_axis_angle_jump_deg = 5.0;
    int max_reject_count_before_reset = 15;

    double min_control_point_spacing = 0.4;
    double max_control_point_jump = 2.0;
    int fit_global_axis_min_points = 6;

    bool ransac_debug_enable = true;
    double ransac_distance_threshold = 0.15;
    int ransac_max_iterations = 500;
    int ransac_min_inliers = 150;
    double ransac_parallel_eps_angle_deg = 5.0;
  };

  struct LineModel
  {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d(0.0, 0.0, 1.0);
    bool valid = false;
    int inlier_count = 0;
    double vertical_angle_deg = 999.0;
  };

  struct SliceCenter
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double z_min = 0.0;
    double z_max = 0.0;
    double radius = 0.0;
    double residual = 0.0;
    int point_count = 0;
    bool valid = false;
    bool used_centroid_fallback = false;
  };

  struct Result
  {
    bool success = false;
    bool jump_rejected = false;

    LineModel local_line;
    LineModel smoothed_axis;
    LineModel global_axis;
    LineModel raw_ransac_line;

    Eigen::Vector3d control_point = Eigen::Vector3d::Zero();
    bool added_new_control_point = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr slice_center_cloud;

    std::vector<SliceCenter> slice_centers;
    std::vector<Eigen::Vector3d> control_points;

    std::string debug_msg;
  };

  void setConfig(const Config& config);

  Result process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
                 const Eigen::Vector3d& drone_position_world);

  void clear();

  std::vector<Eigen::Vector3d> getControlPoints() const;

  LineModel getGlobalAxis() const;

private:
  Config config_;

  bool has_smoothed_axis_ = false;
  LineModel smoothed_axis_;

  bool has_last_valid_line_ = false;
  LineModel last_valid_line_;

  int consecutive_reject_count_ = 0;

  std::vector<Eigen::Vector3d> control_points_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_world,
      const Eigen::Vector3d& drone_position_world) const;

  using AlignedPointVector =
      std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>;

  std::vector<AlignedPointVector> splitCloudByHeight(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

  bool fitCircle2DLeastSquares(
      const AlignedPointVector& slice_points,
      Eigen::Vector2d& center_xy,
      double& radius,
      double& residual) const;

  bool estimateSliceCenter(
      const AlignedPointVector& slice_points,
      double z_min,
      double z_max,
      SliceCenter& slice_center) const;

  std::vector<SliceCenter> rejectOutlierSliceCenters(
      const std::vector<SliceCenter>& centers) const;

  bool fitAxisFromSliceCenters(
      const std::vector<SliceCenter>& centers,
      LineModel& axis) const;

  bool applyTemporalSmoothingAndJumpReject(
      const LineModel& new_axis,
      LineModel& output_axis,
      bool& jump_rejected);

  bool fitLineRansac(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                             LineModel& line,
                             pcl::PointIndices::Ptr& inliers) const;

  bool isNearlyVertical(LineModel& line) const;

  Eigen::Vector3d projectPointToLine(const Eigen::Vector3d& point,
                                     const LineModel& line) const;

  bool shouldAddControlPoint(const Eigen::Vector3d& new_point,
                             bool jump_rejected) const;

  bool fitGlobalAxisFromControlPoints(LineModel& axis) const;
};

}  // namespace wtb_pointcloud_mapping

#endif  // WTB_POINTCLOUD_MAPPING_TOWER_AXIS_ESTIMATOR_H
