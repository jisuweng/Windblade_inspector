#include "wtb_pointcloud_mapping/wtb_mapping_node.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>

namespace wtb_pointcloud_mapping
{

namespace
{
std::vector<double> ensureVector3(const std::vector<double>& input,
                                  const std::vector<double>& fallback)
{
  if (input.size() == 3)
  {
    return input;
  }
  return fallback;
}

ros::Duration periodFromRate(double rate)
{
  if (rate <= 0.0 || !std::isfinite(rate))
  {
    return ros::Duration(0.0);
  }
  return ros::Duration(1.0 / rate);
}
}  // namespace

WTBMappingNode::WTBMappingNode()
  : nh_(),
    private_nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    occupancy_grid_(0.2),
    has_odom_(false),
    use_tf_(false),
    max_time_diff_(0.05),
    strict_time_sync_(false),
    T_body_lidar_(Eigen::Isometry3d::Identity()),
    global_enable_(true),
    global_publish_rate_(2.0),
    occupancy_enable_(true),
    occupancy_max_ray_length_(80.0),
    occupancy_publish_rate_(2.0),
    publish_free_cloud_(false),
    print_timing_(true),
    tower_axis_enable_(true),
    tower_axis_method_("slice_center_axis"),
    tower_axis_input_cloud_mode_("current"),
    tower_publish_candidate_cloud_(true),
    tower_publish_slice_centers_(true),
    tower_publish_control_points_(true),
    tower_publish_markers_(true),
    tower_publish_debug_ransac_line_(true),
    tower_ascent_enable_(false),
    last_ascent_publish_time_(ros::Time(0))
{
  loadParams();
  initializeRos();
}

void WTBMappingNode::loadParams()
{
  readParam<std::string>("input/cloud_topic", cloud_topic_, "/livox/lidar");
  readParam<std::string>("input/odom_topic", odom_topic_, "/iris_0/mavros/vision_odom/odom");
  readParam<std::string>("input/cloud_type", cloud_type_, "livox_custom");
  readParam<bool>("input/use_tf", use_tf_, false);
  readParam<std::string>("input/world_frame", world_frame_, "map");
  readParam<std::string>("input/body_frame", body_frame_, "base_link");
  readParam<std::string>("input/lidar_frame", lidar_frame_, "livox_frame");

  readParam<double>("sync/max_time_diff", max_time_diff_, 0.05);
  readParam<bool>("sync/strict_time_sync", strict_time_sync_, false);

  readParam<std::vector<double>>("extrinsic/lidar_to_body_xyz",
                                 lidar_to_body_xyz_,
                                 std::vector<double>{0.0, 0.0, 0.0});
  readParam<std::vector<double>>("extrinsic/lidar_to_body_rpy",
                                 lidar_to_body_rpy_,
                                 std::vector<double>{0.0, 0.0, 0.0});
  lidar_to_body_xyz_ = ensureVector3(lidar_to_body_xyz_, std::vector<double>{0.0, 0.0, 0.0});
  lidar_to_body_rpy_ = ensureVector3(lidar_to_body_rpy_, std::vector<double>{0.0, 0.0, 0.0});
  T_body_lidar_ = makeTransformFromXYZRPY(lidar_to_body_xyz_, lidar_to_body_rpy_);

  double min_range = 0.5;
  double max_range = 80.0;
  double z_min = -10.0;
  double z_max = 120.0;
  bool remove_nan = true;
  readParam<double>("filter/min_range", min_range, min_range);
  readParam<double>("filter/max_range", max_range, max_range);
  readParam<double>("filter/z_min", z_min, z_min);
  readParam<double>("filter/z_max", z_max, z_max);
  readParam<bool>("filter/remove_nan", remove_nan, remove_nan);
  stitcher_.setFilterParams(min_range, max_range, z_min, z_max, remove_nan);

  double voxel_leaf_size = 0.10;
  int max_points = 3000000;
  readParam<bool>("global_cloud/enable", global_enable_, true);
  readParam<double>("global_cloud/voxel_leaf_size", voxel_leaf_size, voxel_leaf_size);
  readParam<double>("global_cloud/publish_rate", global_publish_rate_, global_publish_rate_);
  readParam<int>("global_cloud/max_points", max_points, max_points);
  stitcher_.setGlobalCloudParams(global_enable_,
                                 voxel_leaf_size,
                                 static_cast<std::size_t>(std::max(0, max_points)));
  global_publish_period_ = periodFromRate(global_publish_rate_);

  double resolution = 0.20;
  double log_odds_hit = 0.85;
  double log_odds_miss = -0.40;
  double log_odds_min = -2.0;
  double log_odds_max = 3.5;
  double occupied_threshold = 1.0;
  double free_threshold = -1.0;
  readParam<bool>("occupancy_grid/enable", occupancy_enable_, true);
  readParam<double>("occupancy_grid/resolution", resolution, resolution);
  readParam<double>("occupancy_grid/log_odds_hit", log_odds_hit, log_odds_hit);
  readParam<double>("occupancy_grid/log_odds_miss", log_odds_miss, log_odds_miss);
  readParam<double>("occupancy_grid/log_odds_min", log_odds_min, log_odds_min);
  readParam<double>("occupancy_grid/log_odds_max", log_odds_max, log_odds_max);
  readParam<double>("occupancy_grid/occupied_threshold", occupied_threshold, occupied_threshold);
  readParam<double>("occupancy_grid/free_threshold", free_threshold, free_threshold);
  readParam<double>("occupancy_grid/max_ray_length",
                    occupancy_max_ray_length_,
                    occupancy_max_ray_length_);
  readParam<double>("occupancy_grid/publish_rate", occupancy_publish_rate_, occupancy_publish_rate_);
  occupancy_grid_.setResolution(resolution);
  occupancy_grid_.setLogOddsParams(log_odds_hit,
                                   log_odds_miss,
                                   log_odds_min,
                                   log_odds_max,
                                   occupied_threshold,
                                   free_threshold);
  occupancy_publish_period_ = periodFromRate(occupancy_publish_rate_);

  readParam<bool>("debug/print_timing", print_timing_, true);
  readParam<bool>("debug/publish_free_cloud", publish_free_cloud_, false);

  TowerAxisEstimator::Config tower_axis_config;
  readParam<bool>("tower_axis/enable", tower_axis_enable_, true);
  tower_axis_config.enable = tower_axis_enable_;

  readParam<std::string>("tower_axis/method", tower_axis_method_, "slice_center_axis");
  tower_axis_config.method = tower_axis_method_;
  if (tower_axis_method_ != "slice_center_axis" && tower_axis_method_ != "ransac_line_debug")
  {
    ROS_WARN_STREAM("[WTBMapping] tower_axis/method '" << tower_axis_method_
                    << "' unknown; using slice_center_axis");
    tower_axis_method_ = "slice_center_axis";
    tower_axis_config.method = "slice_center_axis";
  }

  readParam<std::string>("tower_axis/input_cloud_mode", tower_axis_input_cloud_mode_, "current");
  tower_axis_config.input_cloud_mode = tower_axis_input_cloud_mode_;
  if (tower_axis_input_cloud_mode_ != "current" && tower_axis_input_cloud_mode_ != "global")
  {
    ROS_WARN_STREAM("[WTBMapping] tower_axis/input_cloud_mode is '"
                    << tower_axis_input_cloud_mode_ << "'; using current");
    tower_axis_input_cloud_mode_ = "current";
    tower_axis_config.input_cloud_mode = "current";
  }

  readParam<double>("tower_axis/voxel_leaf_size",
                    tower_axis_config.voxel_leaf_size,
                    tower_axis_config.voxel_leaf_size);
  readParam<double>("tower_axis/min_z", tower_axis_config.min_z, tower_axis_config.min_z);
  readParam<double>("tower_axis/max_z", tower_axis_config.max_z, tower_axis_config.max_z);
  readParam<bool>("tower_axis/use_drone_roi",
                  tower_axis_config.use_drone_roi,
                  tower_axis_config.use_drone_roi);
  readParam<double>("tower_axis/roi_radius_xy",
                    tower_axis_config.roi_radius_xy,
                    tower_axis_config.roi_radius_xy);
  readParam<double>("tower_axis/roi_z_below",
                    tower_axis_config.roi_z_below,
                    tower_axis_config.roi_z_below);
  readParam<double>("tower_axis/roi_z_above",
                    tower_axis_config.roi_z_above,
                    tower_axis_config.roi_z_above);
  readParam<double>("tower_axis/min_range_from_drone",
                    tower_axis_config.min_range_from_drone,
                    tower_axis_config.min_range_from_drone);
  readParam<double>("tower_axis/max_range_from_drone",
                    tower_axis_config.max_range_from_drone,
                    tower_axis_config.max_range_from_drone);

  readParam<double>("tower_axis/slice_height",
                    tower_axis_config.slice_height,
                    tower_axis_config.slice_height);
  readParam<int>("tower_axis/slice_min_points",
                 tower_axis_config.slice_min_points,
                 tower_axis_config.slice_min_points);
  readParam<int>("tower_axis/slice_max_points",
                 tower_axis_config.slice_max_points,
                 tower_axis_config.slice_max_points);

  readParam<bool>("tower_axis/circle_fit_enable",
                  tower_axis_config.circle_fit_enable,
                  tower_axis_config.circle_fit_enable);
  readParam<double>("tower_axis/tower_radius_prior",
                    tower_axis_config.tower_radius_prior,
                    tower_axis_config.tower_radius_prior);
  readParam<double>("tower_axis/tower_radius_min",
                    tower_axis_config.tower_radius_min,
                    tower_axis_config.tower_radius_min);
  readParam<double>("tower_axis/tower_radius_max",
                    tower_axis_config.tower_radius_max,
                    tower_axis_config.tower_radius_max);
  readParam<double>("tower_axis/circle_fit_residual_threshold",
                    tower_axis_config.circle_fit_residual_threshold,
                    tower_axis_config.circle_fit_residual_threshold);
  readParam<int>("tower_axis/min_valid_slices",
                 tower_axis_config.min_valid_slices,
                 tower_axis_config.min_valid_slices);

  readParam<bool>("tower_axis/fallback_to_slice_centroid",
                  tower_axis_config.fallback_to_slice_centroid,
                  tower_axis_config.fallback_to_slice_centroid);

  readParam<bool>("tower_axis/center_outlier_reject_enable",
                  tower_axis_config.center_outlier_reject_enable,
                  tower_axis_config.center_outlier_reject_enable);
  readParam<double>("tower_axis/max_center_xy_deviation",
                    tower_axis_config.max_center_xy_deviation,
                    tower_axis_config.max_center_xy_deviation);
  readParam<double>("tower_axis/max_center_jump_between_slices",
                    tower_axis_config.max_center_jump_between_slices,
                    tower_axis_config.max_center_jump_between_slices);

  readParam<bool>("tower_axis/force_axis_vertical",
                  tower_axis_config.force_axis_vertical,
                  tower_axis_config.force_axis_vertical);

  std::vector<double> vertical_axis{0.0, 0.0, 1.0};
  readParam<std::vector<double>>("tower_axis/vertical_axis", vertical_axis, vertical_axis);
  vertical_axis = ensureVector3(vertical_axis, std::vector<double>{0.0, 0.0, 1.0});
  tower_axis_config.vertical_axis =
      Eigen::Vector3d(vertical_axis[0], vertical_axis[1], vertical_axis[2]);
  readParam<double>("tower_axis/vertical_angle_threshold_deg",
                    tower_axis_config.vertical_angle_threshold_deg,
                    tower_axis_config.vertical_angle_threshold_deg);

  readParam<bool>("tower_axis/temporal_smoothing_enable",
                  tower_axis_config.temporal_smoothing_enable,
                  tower_axis_config.temporal_smoothing_enable);
  readParam<double>("tower_axis/axis_point_alpha",
                    tower_axis_config.axis_point_alpha,
                    tower_axis_config.axis_point_alpha);
  readParam<double>("tower_axis/axis_dir_alpha",
                    tower_axis_config.axis_dir_alpha,
                    tower_axis_config.axis_dir_alpha);

  readParam<bool>("tower_axis/jump_reject_enable",
                  tower_axis_config.jump_reject_enable,
                  tower_axis_config.jump_reject_enable);
  readParam<double>("tower_axis/max_axis_lateral_jump",
                    tower_axis_config.max_axis_lateral_jump,
                    tower_axis_config.max_axis_lateral_jump);
  readParam<double>("tower_axis/max_axis_angle_jump_deg",
                    tower_axis_config.max_axis_angle_jump_deg,
                    tower_axis_config.max_axis_angle_jump_deg);
  readParam<int>("tower_axis/max_reject_count_before_reset",
                 tower_axis_config.max_reject_count_before_reset,
                 tower_axis_config.max_reject_count_before_reset);

  readParam<double>("tower_axis/min_control_point_spacing",
                    tower_axis_config.min_control_point_spacing,
                    tower_axis_config.min_control_point_spacing);
  readParam<double>("tower_axis/max_control_point_jump",
                    tower_axis_config.max_control_point_jump,
                    tower_axis_config.max_control_point_jump);
  readParam<int>("tower_axis/fit_global_axis_min_points",
                 tower_axis_config.fit_global_axis_min_points,
                 tower_axis_config.fit_global_axis_min_points);

  readParam<bool>("tower_axis/ransac_debug_enable",
                  tower_axis_config.ransac_debug_enable,
                  tower_axis_config.ransac_debug_enable);
  readParam<double>("tower_axis/ransac_distance_threshold",
                    tower_axis_config.ransac_distance_threshold,
                    tower_axis_config.ransac_distance_threshold);
  readParam<int>("tower_axis/ransac_max_iterations",
                 tower_axis_config.ransac_max_iterations,
                 tower_axis_config.ransac_max_iterations);
  readParam<int>("tower_axis/ransac_min_inliers",
                 tower_axis_config.ransac_min_inliers,
                 tower_axis_config.ransac_min_inliers);
  readParam<double>("tower_axis/ransac_parallel_eps_angle_deg",
                    tower_axis_config.ransac_parallel_eps_angle_deg,
                    tower_axis_config.ransac_parallel_eps_angle_deg);

  readParam<bool>("tower_axis/publish_candidate_cloud",
                  tower_publish_candidate_cloud_,
                  true);
  readParam<bool>("tower_axis/publish_slice_centers",
                  tower_publish_slice_centers_,
                  true);
  readParam<bool>("tower_axis/publish_control_points",
                  tower_publish_control_points_,
                  true);
  readParam<bool>("tower_axis/publish_markers",
                  tower_publish_markers_,
                  true);
  readParam<bool>("tower_axis/publish_debug_ransac_line",
                  tower_publish_debug_ransac_line_,
                  true);

  tower_axis_estimator_.setConfig(tower_axis_config);

  readParam<bool>("tower_ascent/enable", tower_ascent_enable_, false);
  readParam<std::string>("tower_ascent/strategy", tower_ascent_config_.strategy, "spiral");
  readParam<double>("tower_ascent/target_distance_from_axis",
                    tower_ascent_config_.target_distance_from_axis, 8.0);
  readParam<double>("tower_ascent/tower_surface_clearance",
                    tower_ascent_config_.tower_surface_clearance, 3.0);
  readParam<double>("tower_ascent/ascent_speed",
                    tower_ascent_config_.ascent_speed, 0.5);
  readParam<double>("tower_ascent/spiral_revolution_time",
                    tower_ascent_config_.spiral_revolution_time, 40.0);
  readParam<double>("tower_ascent/min_height",
                    tower_ascent_config_.min_height, 2.0);
  readParam<double>("tower_ascent/max_height",
                    tower_ascent_config_.max_height, 80.0);
  readParam<double>("tower_ascent/height_padding_below",
                    tower_ascent_config_.height_padding_below, 3.0);
  readParam<double>("tower_ascent/height_padding_above",
                    tower_ascent_config_.height_padding_above, 5.0);
  readParam<double>("tower_ascent/horizontal_arrival_tolerance",
                    tower_ascent_config_.horizontal_arrival_tolerance, 0.5);
  readParam<double>("tower_ascent/vertical_arrival_tolerance",
                    tower_ascent_config_.vertical_arrival_tolerance, 0.3);
  readParam<double>("tower_ascent/yaw_rate",
                    tower_ascent_config_.yaw_rate, 0.3);
  readParam<double>("tower_ascent/publish_rate",
                    tower_ascent_config_.publish_rate, 20.0);
  readParam<std::string>("tower_ascent/setpoint_topic",
                         tower_ascent_config_.setpoint_topic,
                         "/iris_0/mavros/setpoint_position/local");
  readParam<std::string>("tower_ascent/setpoint_frame",
                         tower_ascent_config_.setpoint_frame, "map");
  readParam<bool>("tower_ascent/publish_path_preview",
                  tower_ascent_config_.publish_path_preview, true);
  tower_ascent_controller_.setConfig(tower_ascent_config_);
  ascent_publish_period_ = periodFromRate(tower_ascent_config_.publish_rate);

  if (cloud_type_ != "livox_custom")
  {
    ROS_WARN_STREAM("[WTBMapping] input/cloud_type is '" << cloud_type_
                    << "', but this node expects livox_custom.");
  }
}

void WTBMappingNode::initializeRos()
{
  cloud_sub_ = nh_.subscribe(cloud_topic_, 5, &WTBMappingNode::cloudCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 20, &WTBMappingNode::odomCallback, this);

  current_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/current_cloud_world", 1);
  global_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/global_cloud", 1);
  occupied_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/occupied_cloud", 1);
  free_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/free_cloud", 1);
  debug_info_pub_ =
      nh_.advertise<std_msgs::String>("/wtb/map_debug_info", 1);
  tower_candidate_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/tower_candidate_cloud", 1);
  tower_slice_centers_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/tower_slice_centers", 1);
  tower_control_points_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/wtb/tower_control_points", 1);
  tower_axis_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/wtb/tower_axis_marker", 1, true);
  tower_global_axis_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/wtb/tower_global_axis_marker", 1, true);
  tower_ransac_line_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/wtb/tower_ransac_line_marker", 1, true);
  tower_debug_info_pub_ =
      nh_.advertise<std_msgs::String>("/wtb/tower_debug_info", 1);
  tower_setpoint_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(tower_ascent_config_.setpoint_topic, 1);
  tower_path_preview_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/wtb/tower_ascent_path", 1, true);

  ROS_INFO_STREAM("[WTBMapping] subscribing cloud: " << cloud_topic_);
  ROS_INFO_STREAM("[WTBMapping] subscribing odom: " << odom_topic_);
  ROS_INFO_STREAM("[WTBMapping] world_frame=" << world_frame_
                  << ", body_frame=" << body_frame_
                  << ", lidar_frame=" << lidar_frame_
                  << ", use_tf=" << (use_tf_ ? "true" : "false"));
  ROS_INFO_STREAM("[WTBMapping] tower_axis="
                  << (tower_axis_enable_ ? "enabled" : "disabled")
                  << ", method=" << tower_axis_method_
                  << ", input_cloud_mode=" << tower_axis_input_cloud_mode_);
  ROS_INFO_STREAM("[WTBMapping] tower_ascent="
                  << (tower_ascent_enable_ ? "enabled" : "disabled")
                  << ", strategy=" << tower_ascent_config_.strategy);
}

void WTBMappingNode::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  has_odom_ = true;
}

void WTBMappingNode::cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& msg)
{
  const ros::WallTime start_time = ros::WallTime::now();

  nav_msgs::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_)
    {
      ROS_WARN_THROTTLE(1.0, "[WTBMapping] waiting for odom on %s", odom_topic_.c_str());
      return;
    }
    odom = latest_odom_;
  }

  const double time_diff = std::abs((msg->header.stamp - odom.header.stamp).toSec());
  if (time_diff > max_time_diff_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[WTBMapping] cloud-odom time diff %.6f s exceeds %.6f s",
                      time_diff,
                      max_time_diff_);
    if (strict_time_sync_)
    {
      return;
    }
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar = convertLivoxCustomMsgToPCL(msg);
  const Eigen::Isometry3d T_map_body = odomToTransform(odom);

  Eigen::Isometry3d T_body_lidar = T_body_lidar_;
  if (use_tf_ && !lookupBodyToLidarTransform(msg->header.stamp, T_body_lidar))
  {
    return;
  }

  const Eigen::Isometry3d T_map_lidar = T_map_body * T_body_lidar;
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world =
      stitcher_.transformToWorld(cloud_lidar, T_map_lidar);

  current_cloud_pub_.publish(cloudToRosMsg(cloud_world, world_frame_, msg->header.stamp));

  if (global_enable_)
  {
    stitcher_.addToGlobalCloud(cloud_world);
    const ros::Time now = ros::Time::now();
    if (shouldPublish(now, last_global_publish_time_, global_publish_period_))
    {
      global_cloud_pub_.publish(
          cloudToRosMsg(stitcher_.getGlobalCloud(), world_frame_, msg->header.stamp));
      last_global_publish_time_ = now;
    }
  }

  if (tower_axis_enable_)
  {
    const pcl::PointCloud<pcl::PointXYZI>::Ptr tower_input_cloud =
        selectTowerInputCloud(cloud_world);
    const TowerAxisEstimator::Result tower_result =
        tower_axis_estimator_.process(tower_input_cloud, T_map_body.translation());
    publishTowerAxisResult(tower_result, msg->header.stamp);

    if (tower_ascent_enable_)
    {
      const double tower_radius = 0.0;  // auto-estimate
      const TowerAscentController::Setpoint sp = tower_ascent_controller_.update(
          tower_result.smoothed_axis,
          tower_result.global_axis,
          T_map_body.translation(),
          tower_result.control_points,
          tower_radius,
          msg->header.stamp,
          tower_result.success,
          tower_result.jump_rejected);

      const ros::Time now = ros::Time::now();
      if (sp.valid && shouldPublish(now, last_ascent_publish_time_, ascent_publish_period_))
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = tower_ascent_config_.setpoint_frame;
        pose.header.stamp = msg->header.stamp;
        pose.pose.position.x = sp.position.x();
        pose.pose.position.y = sp.position.y();
        pose.pose.position.z = sp.position.z();
        const Eigen::Quaterniond q(
            Eigen::AngleAxisd(sp.yaw, Eigen::Vector3d::UnitZ()));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        tower_setpoint_pub_.publish(pose);
        last_ascent_publish_time_ = now;
      }
    }
  }

  if (occupancy_enable_)
  {
    const Eigen::Vector3d sensor_origin = T_map_lidar.translation();
    occupancy_grid_.updatePointCloud(sensor_origin, cloud_world, occupancy_max_ray_length_);

    const ros::Time now = ros::Time::now();
    if (shouldPublish(now, last_occupied_publish_time_, occupancy_publish_period_))
    {
      occupied_cloud_pub_.publish(
          cloudToRosMsg(occupancy_grid_.getOccupiedCloud(), world_frame_, msg->header.stamp));
      last_occupied_publish_time_ = now;
    }

    if (publish_free_cloud_ &&
        shouldPublish(now, last_free_publish_time_, occupancy_publish_period_))
    {
      free_cloud_pub_.publish(
          cloudToRosMsg(occupancy_grid_.getFreeCloud(), world_frame_, msg->header.stamp));
      last_free_publish_time_ = now;
    }
  }

  const double processing_ms = (ros::WallTime::now() - start_time).toSec() * 1000.0;
  publishDebugInfo(cloud_lidar ? cloud_lidar->points.size() : 0,
                   cloud_world ? cloud_world->points.size() : 0,
                   time_diff,
                   processing_ms);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr WTBMappingNode::convertLivoxCustomMsgToPCL(
    const livox_laser_simulation::CustomMsgConstPtr& msg) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!msg)
  {
    cloud->height = 1;
    cloud->width = 0;
    cloud->is_dense = false;
    return cloud;
  }

  cloud->points.reserve(msg->points.size());
  for (const auto& livox_point : msg->points)
  {
    pcl::PointXYZI p;
    p.x = livox_point.x;
    p.y = livox_point.y;
    p.z = livox_point.z;
    p.intensity = static_cast<float>(livox_point.reflectivity);
    cloud->points.push_back(p);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

Eigen::Isometry3d WTBMappingNode::odomToTransform(const nav_msgs::Odometry& odom) const
{
  const Eigen::Vector3d t_map_body(odom.pose.pose.position.x,
                                   odom.pose.pose.position.y,
                                   odom.pose.pose.position.z);

  Eigen::Quaterniond q_map_body(odom.pose.pose.orientation.w,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z);
  if (q_map_body.norm() < 1e-12 || !std::isfinite(q_map_body.norm()))
  {
    ROS_WARN_THROTTLE(1.0, "[WTBMapping] odom quaternion is invalid; using identity rotation");
    q_map_body = Eigen::Quaterniond::Identity();
  }
  else
  {
    q_map_body.normalize();
  }

  Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
  T_map_body.linear() = q_map_body.toRotationMatrix();
  T_map_body.translation() = t_map_body;
  return T_map_body;
}

Eigen::Isometry3d WTBMappingNode::makeTransformFromXYZRPY(
    const std::vector<double>& xyz,
    const std::vector<double>& rpy) const
{
  const std::vector<double> safe_xyz = ensureVector3(xyz, std::vector<double>{0.0, 0.0, 0.0});
  const std::vector<double> safe_rpy = ensureVector3(rpy, std::vector<double>{0.0, 0.0, 0.0});

  const double roll = safe_rpy[0];
  const double pitch = safe_rpy[1];
  const double yaw = safe_rpy[2];

  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(safe_xyz[0], safe_xyz[1], safe_xyz[2]);
  return transform;
}

bool WTBMappingNode::lookupBodyToLidarTransform(const ros::Time& stamp,
                                                Eigen::Isometry3d& T_body_lidar) const
{
  try
  {
    const ros::Time lookup_stamp = stamp.isZero() ? ros::Time(0) : stamp;
    const geometry_msgs::TransformStamped tf_msg =
        tf_buffer_.lookupTransform(body_frame_, lidar_frame_, lookup_stamp, ros::Duration(0.02));
    T_body_lidar = tf2::transformToEigen(tf_msg);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[WTBMapping] failed to lookup T_%s_%s: %s",
                      body_frame_.c_str(),
                      lidar_frame_.c_str(),
                      ex.what());
    return false;
  }
}

sensor_msgs::PointCloud2 WTBMappingNode::cloudToRosMsg(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& frame_id,
    const ros::Time& stamp) const
{
  sensor_msgs::PointCloud2 msg;
  if (cloud)
  {
    pcl::toROSMsg(*cloud, msg);
  }
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

bool WTBMappingNode::shouldPublish(const ros::Time& now,
                                   const ros::Time& last_publish,
                                   const ros::Duration& period) const
{
  return period.toSec() <= 0.0 || last_publish.isZero() || (now - last_publish) >= period;
}

void WTBMappingNode::publishDebugInfo(std::size_t lidar_points,
                                      std::size_t world_points,
                                      double time_diff,
                                      double processing_ms) const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "[WTBMapping] current lidar points: " << lidar_points
     << "\n[WTBMapping] current world points: " << world_points
     << "\n[WTBMapping] global cloud points: " << stitcher_.globalCloudSize()
     << "\n[WTBMapping] occupancy voxels: " << occupancy_grid_.size()
     << "\n[WTBMapping] occupied voxels: " << occupancy_grid_.countOccupied()
     << "\n[WTBMapping] cloud-odom time diff: " << time_diff << " s"
     << "\n[WTBMapping] processing time: " << processing_ms << " ms";

  std_msgs::String debug_msg;
  debug_msg.data = ss.str();
  debug_info_pub_.publish(debug_msg);

  if (print_timing_)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, debug_msg.data);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr WTBMappingNode::selectTowerInputCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud_world) const
{
  if (tower_axis_input_cloud_mode_ == "global")
  {
    return stitcher_.getGlobalCloud();
  }
  return current_cloud_world;
}

void WTBMappingNode::publishTowerAxisResult(const TowerAxisEstimator::Result& result,
                                            const ros::Time& stamp) const
{
  if (tower_publish_candidate_cloud_ && result.candidate_cloud)
  {
    tower_candidate_cloud_pub_.publish(
        cloudToRosMsg(result.candidate_cloud, world_frame_, stamp));
  }

  if (tower_publish_slice_centers_ && result.slice_center_cloud &&
      !result.slice_center_cloud->empty())
  {
    tower_slice_centers_pub_.publish(
        cloudToRosMsg(result.slice_center_cloud, world_frame_, stamp));
  }

  if (tower_publish_control_points_)
  {
    tower_control_points_pub_.publish(
        cloudToRosMsg(controlPointsToCloud(result.control_points), world_frame_, stamp));
  }

  if (tower_publish_markers_)
  {
    const TowerAxisEstimator::LineModel& axis =
        result.smoothed_axis.valid ? result.smoothed_axis : result.local_line;

    double axis_z_min = 0.0;
    double axis_z_max = 0.0;
    if (!result.slice_centers.empty())
    {
      axis_z_min = std::numeric_limits<double>::max();
      axis_z_max = std::numeric_limits<double>::lowest();
      for (const auto& sc : result.slice_centers)
      {
        if (sc.valid)
        {
          axis_z_min = std::min(axis_z_min, sc.center.z());
          axis_z_max = std::max(axis_z_max, sc.center.z());
        }
      }
      if (!std::isfinite(axis_z_min) || !std::isfinite(axis_z_max) ||
          axis_z_max <= axis_z_min)
      {
        axis_z_min = axis.point.z() - 5.0;
        axis_z_max = axis.point.z() + 5.0;
      }
    }
    else
    {
      axis_z_min = axis.point.z() - 5.0;
      axis_z_max = axis.point.z() + 5.0;
    }

    tower_axis_marker_pub_.publish(
        makeAxisMarker(axis,
                       "tower_axis",
                       0,
                       stamp,
                       axis_z_min,
                       axis_z_max,
                       0.12,
                       0.0f,
                       1.0f,
                       0.0f,
                       1.0f));

    tower_global_axis_marker_pub_.publish(makeGlobalAxisMarker(result, stamp));
  }

  if (tower_publish_debug_ransac_line_ && result.raw_ransac_line.valid)
  {
    const double z_len = 20.0;
    tower_ransac_line_marker_pub_.publish(
        makeAxisMarker(result.raw_ransac_line,
                       "tower_ransac_line",
                       0,
                       stamp,
                       result.raw_ransac_line.point.z() - z_len * 0.5,
                       result.raw_ransac_line.point.z() + z_len * 0.5,
                       0.08,
                       1.0f,
                       1.0f,
                       0.0f,
                       0.8f));
  }
  else if (tower_publish_debug_ransac_line_)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = stamp;
    marker.ns = "tower_ransac_line";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    tower_ransac_line_marker_pub_.publish(marker);
  }

  std_msgs::String debug_msg;
  debug_msg.data = result.debug_msg;
  tower_debug_info_pub_.publish(debug_msg);
  ROS_INFO_STREAM_THROTTLE(1.0, debug_msg.data);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr WTBMappingNode::controlPointsToCloud(
    const std::vector<Eigen::Vector3d>& control_points) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.reserve(control_points.size());

  for (std::size_t i = 0; i < control_points.size(); ++i)
  {
    const Eigen::Vector3d& point = control_points[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z()))
    {
      continue;
    }

    pcl::PointXYZI pcl_point;
    pcl_point.x = static_cast<float>(point.x());
    pcl_point.y = static_cast<float>(point.y());
    pcl_point.z = static_cast<float>(point.z());
    pcl_point.intensity = static_cast<float>(i);
    cloud->points.push_back(pcl_point);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  return cloud;
}

visualization_msgs::Marker WTBMappingNode::makeLineMarker(
    const TowerAxisEstimator::LineModel& line,
    const std::string& marker_ns,
    int marker_id,
    const ros::Time& stamp,
    double length,
    double width,
    float r,
    float g,
    float b,
    float a) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = stamp;
  marker.ns = marker_ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  if (!line.valid || line.direction.norm() < 1e-9)
  {
    return marker;
  }

  marker.action = visualization_msgs::Marker::ADD;

  const Eigen::Vector3d direction = line.direction.normalized();
  const Eigen::Vector3d p0 = line.point - 0.5 * length * direction;
  const Eigen::Vector3d p1 = line.point + 0.5 * length * direction;

  geometry_msgs::Point start;
  start.x = p0.x();
  start.y = p0.y();
  start.z = p0.z();

  geometry_msgs::Point end;
  end.x = p1.x();
  end.y = p1.y();
  end.z = p1.z();

  marker.points.push_back(start);
  marker.points.push_back(end);
  return marker;
}

visualization_msgs::Marker WTBMappingNode::makeAxisMarker(
    const TowerAxisEstimator::LineModel& line,
    const std::string& marker_ns,
    int marker_id,
    const ros::Time& stamp,
    double z_min,
    double z_max,
    double width,
    float r,
    float g,
    float b,
    float a) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = stamp;
  marker.ns = marker_ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  if (!line.valid || line.direction.norm() < 1e-9)
  {
    return marker;
  }

  const Eigen::Vector3d direction = line.direction.normalized();
  if (std::abs(direction.z()) < 1e-9)
  {
    return marker;
  }

  marker.action = visualization_msgs::Marker::ADD;

  const Eigen::Vector3d p0 =
      line.point + direction * ((z_min - line.point.z()) / direction.z());
  const Eigen::Vector3d p1 =
      line.point + direction * ((z_max - line.point.z()) / direction.z());

  geometry_msgs::Point start;
  start.x = p0.x();
  start.y = p0.y();
  start.z = p0.z();

  geometry_msgs::Point end;
  end.x = p1.x();
  end.y = p1.y();
  end.z = p1.z();

  marker.points.push_back(start);
  marker.points.push_back(end);
  return marker;
}

visualization_msgs::Marker WTBMappingNode::makeGlobalAxisMarker(
    const TowerAxisEstimator::Result& result,
    const ros::Time& stamp) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = stamp;
  marker.ns = "tower_global_axis";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.14;
  marker.color.r = 1.0f;
  marker.color.g = 0.1f;
  marker.color.b = 0.05f;
  marker.color.a = 1.0f;

  if (!result.global_axis.valid || result.control_points.empty())
  {
    return marker;
  }

  const TowerAxisEstimator::LineModel& axis = result.global_axis;
  if (std::abs(axis.direction.z()) < 1e-9)
  {
    return marker;
  }

  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
  for (const auto& point : result.control_points)
  {
    if (!std::isfinite(point.z()))
    {
      continue;
    }
    min_z = std::min(min_z, point.z());
    max_z = std::max(max_z, point.z());
  }

  if (!std::isfinite(min_z) || !std::isfinite(max_z))
  {
    return marker;
  }

  if (max_z - min_z < 1.0)
  {
    min_z -= 5.0;
    max_z += 5.0;
  }
  else
  {
    min_z -= 0.5;
    max_z += 0.5;
  }

  marker.action = visualization_msgs::Marker::ADD;

  const Eigen::Vector3d direction = axis.direction.normalized();
  const Eigen::Vector3d p0 =
      axis.point + direction * ((min_z - axis.point.z()) / direction.z());
  const Eigen::Vector3d p1 =
      axis.point + direction * ((max_z - axis.point.z()) / direction.z());

  geometry_msgs::Point start;
  start.x = p0.x();
  start.y = p0.y();
  start.z = p0.z();

  geometry_msgs::Point end;
  end.x = p1.x();
  end.y = p1.y();
  end.z = p1.z();

  marker.points.push_back(start);
  marker.points.push_back(end);
  return marker;
}

}  // namespace wtb_pointcloud_mapping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wtb_mapping_node");
  wtb_pointcloud_mapping::WTBMappingNode node;
  ros::spin();
  return 0;
}
