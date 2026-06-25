#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace wtb_pointcloud_mapping
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

bool isFinitePoint(const pcl::PointXYZI& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Eigen::Vector3d odomPosition(const nav_msgs::Odometry& odom)
{
  return Eigen::Vector3d(odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z);
}

geometry_msgs::Point toPointMsg(const Eigen::Vector3d& point)
{
  geometry_msgs::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std_msgs::ColorRGBA color(double r, double g, double b, double a)
{
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<float>(r);
  msg.g = static_cast<float>(g);
  msg.b = static_cast<float>(b);
  msg.a = static_cast<float>(a);
  return msg;
}

std::string normalizeParamNamespace(const std::string& ns)
{
  if (ns.empty())
  {
    return "";
  }
  std::string normalized = ns;
  if (normalized.front() != '/')
  {
    normalized = "/" + normalized;
  }
  while (normalized.size() > 1 && normalized.back() == '/')
  {
    normalized.pop_back();
  }
  return normalized;
}

template <typename T>
void readParam(const ros::NodeHandle& nh,
               const ros::NodeHandle& private_nh,
               const std::string& key,
               T& value)
{
  if (private_nh.getParam(key, value))
  {
    return;
  }
  nh.getParam("tower_detector/" + key, value);
}

struct TowerDetectorConfig
{
  std::string cloud_topic = "/wtb/current_cloud_world";
  std::string odom_topic = "/wtb/odom";
  std::string world_frame = "map";

  std::string status_topic = "/wtb/tower_detector/status";
  std::string control_point_topic = "/wtb/tower_detector/control_point";
  std::string control_pose_topic = "/wtb/tower_detector/control_pose";
  std::string axis_direction_topic = "/wtb/tower_detector/axis_direction";
  std::string markers_topic = "/wtb/tower_detector/markers";
  std::string inlier_cloud_topic = "/wtb/tower_detector/inlier_cloud";
  std::string path_topic = "/wtb/tower_detector/path";
  std::string diameter_topic = "/wtb/tower_detector/diameter";
  std::string radius_topic = "/wtb/tower_detector/radius";

  std::string param_namespace = "/wtb/tower_detector";
  std::string legacy_param_namespace = "/tower";

  double update_rate = 2.0;
  double distance_threshold = 0.8;
  int num_iterations = 80;
  double vertical_angle_max_deg = 15.0;
  double min_inlier_ratio = 0.05;
  int min_line_inliers = 30;
  int min_points = 30;
  int max_sample_points = 3000;
  double xy_tolerance = 1.5;
  double xy_smoothing_alpha = 0.6;
  double radius_min = 0.1;
  double radius_max = 10.0;
  double marker_cylinder_diameter = 0.6;
  double control_point_min_spacing = 1.5;
  int max_path_points = 5000;

  bool estimate_diameter_once = true;
  bool publish_markers = true;
  bool publish_inlier_cloud = true;
  bool print_status = true;
};

TowerDetectorConfig loadConfig(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
{
  TowerDetectorConfig config;
  readParam(nh, private_nh, "cloud_topic", config.cloud_topic);
  readParam(nh, private_nh, "odom_topic", config.odom_topic);
  readParam(nh, private_nh, "world_frame", config.world_frame);

  readParam(nh, private_nh, "status_topic", config.status_topic);
  readParam(nh, private_nh, "control_point_topic", config.control_point_topic);
  readParam(nh, private_nh, "control_pose_topic", config.control_pose_topic);
  readParam(nh, private_nh, "axis_direction_topic", config.axis_direction_topic);
  readParam(nh, private_nh, "markers_topic", config.markers_topic);
  readParam(nh, private_nh, "inlier_cloud_topic", config.inlier_cloud_topic);
  readParam(nh, private_nh, "path_topic", config.path_topic);
  readParam(nh, private_nh, "diameter_topic", config.diameter_topic);
  readParam(nh, private_nh, "radius_topic", config.radius_topic);

  readParam(nh, private_nh, "param_namespace", config.param_namespace);
  readParam(nh, private_nh, "legacy_param_namespace", config.legacy_param_namespace);

  readParam(nh, private_nh, "update_rate", config.update_rate);
  readParam(nh, private_nh, "distance_threshold", config.distance_threshold);
  readParam(nh, private_nh, "num_iterations", config.num_iterations);
  readParam(nh, private_nh, "vertical_angle_max_deg", config.vertical_angle_max_deg);
  readParam(nh, private_nh, "min_inlier_ratio", config.min_inlier_ratio);
  readParam(nh, private_nh, "min_line_inliers", config.min_line_inliers);
  readParam(nh, private_nh, "min_points", config.min_points);
  readParam(nh, private_nh, "max_sample_points", config.max_sample_points);
  readParam(nh, private_nh, "xy_tolerance", config.xy_tolerance);
  readParam(nh, private_nh, "xy_smoothing_alpha", config.xy_smoothing_alpha);
  readParam(nh, private_nh, "radius_min", config.radius_min);
  readParam(nh, private_nh, "radius_max", config.radius_max);
  readParam(nh, private_nh, "marker_cylinder_diameter", config.marker_cylinder_diameter);
  readParam(nh, private_nh, "control_point_min_spacing", config.control_point_min_spacing);
  readParam(nh, private_nh, "max_path_points", config.max_path_points);

  readParam(nh, private_nh, "estimate_diameter_once", config.estimate_diameter_once);
  readParam(nh, private_nh, "publish_markers", config.publish_markers);
  readParam(nh, private_nh, "publish_inlier_cloud", config.publish_inlier_cloud);
  readParam(nh, private_nh, "print_status", config.print_status);

  config.update_rate = std::max(0.0, config.update_rate);
  config.distance_threshold = std::max(0.01, config.distance_threshold);
  config.num_iterations = std::max(1, config.num_iterations);
  config.vertical_angle_max_deg = clamp(config.vertical_angle_max_deg, 0.0, 89.0);
  config.min_inlier_ratio = clamp(config.min_inlier_ratio, 0.0, 1.0);
  config.min_line_inliers = std::max(1, config.min_line_inliers);
  config.min_points = std::max(2, config.min_points);
  config.max_sample_points = std::max(0, config.max_sample_points);
  config.xy_tolerance = std::max(0.0, config.xy_tolerance);
  config.xy_smoothing_alpha = clamp(config.xy_smoothing_alpha, 0.0, 1.0);
  config.marker_cylinder_diameter = std::max(0.01, config.marker_cylinder_diameter);
  config.control_point_min_spacing = std::max(0.0, config.control_point_min_spacing);
  config.max_path_points = std::max(1, config.max_path_points);

  const double safe_radius_min = std::min(config.radius_min, config.radius_max);
  const double safe_radius_max = std::max(config.radius_min, config.radius_max);
  config.radius_min = std::max(0.01, safe_radius_min);
  config.radius_max = std::max(config.radius_min, safe_radius_max);

  config.param_namespace = normalizeParamNamespace(config.param_namespace);
  config.legacy_param_namespace = normalizeParamNamespace(config.legacy_param_namespace);
  return config;
}

struct LineFitResult
{
  bool valid = false;
  std::string reject_reason;
  Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
  std::vector<int> inlier_indices;
  int inliers = 0;
  double inlier_ratio = 0.0;
  double mean_error = std::numeric_limits<double>::max();
  double vertical_angle_deg = 90.0;
};

struct CircleFitResult
{
  bool valid = false;
  std::string reject_reason;
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;
};

}  // namespace

class WTBTowerDetectorNode
{
public:
  WTBTowerDetectorNode()
    : nh_(),
      private_nh_("~"),
      config_(loadConfig(nh_, private_nh_)),
      rng_(1337)
  {
    initializeRos();
    tower_path_.header.frame_id = config_.world_frame;
  }

private:
  void initializeRos()
  {
    cloud_sub_ = nh_.subscribe(config_.cloud_topic,
                               1,
                               &WTBTowerDetectorNode::cloudCallback,
                               this);
    odom_sub_ = nh_.subscribe(config_.odom_topic,
                              20,
                              &WTBTowerDetectorNode::odomCallback,
                              this);

    status_pub_ = nh_.advertise<std_msgs::String>(config_.status_topic, 1);
    control_point_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>(config_.control_point_topic, 1);
    control_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(config_.control_pose_topic, 1);
    axis_direction_pub_ =
        nh_.advertise<geometry_msgs::Vector3Stamped>(config_.axis_direction_topic, 1);
    marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(config_.markers_topic, 1);
    inlier_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(config_.inlier_cloud_topic, 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.path_topic, 1, true);
    diameter_pub_ = nh_.advertise<std_msgs::Float64>(config_.diameter_topic, 1, true);
    radius_pub_ = nh_.advertise<std_msgs::Float64>(config_.radius_topic, 1, true);

    ROS_INFO_STREAM("[WTBTowerDetector] subscribing cloud: " << config_.cloud_topic);
    ROS_INFO_STREAM("[WTBTowerDetector] subscribing odom: " << config_.odom_topic);
    ROS_INFO_STREAM("[WTBTowerDetector] world_frame=" << config_.world_frame
                    << ", update_rate=" << config_.update_rate
                    << ", distance_threshold=" << config_.distance_threshold);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (config_.update_rate > 0.0 && !last_update_time_.isZero() &&
        (now - last_update_time_).toSec() < 1.0 / config_.update_rate)
    {
      return;
    }
    last_update_time_ = now;

    nav_msgs::Odometry odom;
    if (!getLatestOdom(odom))
    {
      publishStatus("waiting for odom on " + config_.odom_topic);
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != config_.world_frame)
    {
      ROS_WARN_THROTTLE(2.0,
                        "[WTBTowerDetector] cloud frame '%s' differs from configured world frame '%s'",
                        msg->header.frame_id.c_str(),
                        config_.world_frame.c_str());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *input_cloud);

    std::vector<Eigen::Vector3d> points = sampleFinitePoints(*input_cloud);
    if (static_cast<int>(points.size()) < config_.min_points)
    {
      std::ostringstream ss;
      ss << "filtered points too few: " << points.size()
         << " < " << config_.min_points;
      publishStatus(ss.str());
      return;
    }

    const LineFitResult line = fitLineRansac(points);
    if (!line.valid)
    {
      publishStatus(line.reject_reason);
      return;
    }

    if (line.vertical_angle_deg > config_.vertical_angle_max_deg)
    {
      std::ostringstream ss;
      ss << "line tilt too large: " << line.vertical_angle_deg
         << " deg > " << config_.vertical_angle_max_deg << " deg";
      publishStatus(ss.str());
      return;
    }

    updateTowerState(points, line, odomPosition(odom), msg->header.stamp);
  }

  bool getLatestOdom(nav_msgs::Odometry& odom) const
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_)
    {
      return false;
    }
    odom = latest_odom_;
    return true;
  }

  std::vector<Eigen::Vector3d> sampleFinitePoints(const pcl::PointCloud<pcl::PointXYZI>& cloud)
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud.points.size());
    for (const pcl::PointXYZI& point : cloud.points)
    {
      if (isFinitePoint(point))
      {
        points.emplace_back(point.x, point.y, point.z);
      }
    }

    if (config_.max_sample_points > 0 &&
        points.size() > static_cast<size_t>(config_.max_sample_points))
    {
      std::vector<size_t> indices(points.size());
      std::iota(indices.begin(), indices.end(), 0);
      std::shuffle(indices.begin(), indices.end(), rng_);

      std::vector<Eigen::Vector3d> sampled;
      sampled.reserve(static_cast<size_t>(config_.max_sample_points));
      for (int i = 0; i < config_.max_sample_points; ++i)
      {
        sampled.push_back(points[indices[static_cast<size_t>(i)]]);
      }
      return sampled;
    }

    return points;
  }

  LineFitResult fitLineRansac(const std::vector<Eigen::Vector3d>& points)
  {
    LineFitResult best;
    if (points.size() < 2)
    {
      best.reject_reason = "line fit input points too few";
      return best;
    }

    std::uniform_int_distribution<int> distribution(
        0, static_cast<int>(points.size()) - 1);

    for (int iteration = 0; iteration < config_.num_iterations; ++iteration)
    {
      const int i = distribution(rng_);
      const int j = distribution(rng_);
      if (i == j)
      {
        continue;
      }

      Eigen::Vector3d direction = points[static_cast<size_t>(j)] -
                                  points[static_cast<size_t>(i)];
      const double direction_norm = direction.norm();
      if (direction_norm < 1e-6)
      {
        continue;
      }
      direction /= direction_norm;

      std::vector<int> inlier_indices;
      inlier_indices.reserve(points.size());
      double error_sum = 0.0;
      for (size_t point_index = 0; point_index < points.size(); ++point_index)
      {
        const double distance =
            ((points[point_index] - points[static_cast<size_t>(i)]).cross(direction)).norm();
        if (distance <= config_.distance_threshold)
        {
          inlier_indices.push_back(static_cast<int>(point_index));
          error_sum += distance;
        }
      }

      const int inliers = static_cast<int>(inlier_indices.size());
      const double mean_error =
          inliers > 0 ? error_sum / static_cast<double>(inliers)
                      : std::numeric_limits<double>::max();
      if (inliers > best.inliers ||
          (inliers == best.inliers && mean_error < best.mean_error))
      {
        best.inliers = inliers;
        best.mean_error = mean_error;
        best.inlier_indices = inlier_indices;
      }
    }

    const int min_required_inliers =
        std::max(config_.min_line_inliers,
                 static_cast<int>(std::ceil(config_.min_inlier_ratio *
                                            static_cast<double>(points.size()))));
    if (best.inliers < min_required_inliers)
    {
      std::ostringstream ss;
      ss << "line inliers too few: " << best.inliers
         << " < " << min_required_inliers;
      best.reject_reason = ss.str();
      return best;
    }

    if (!refineLineDirection(points, best.inlier_indices, best.direction))
    {
      best.reject_reason = "line PCA refinement failed";
      return best;
    }

    best.valid = true;
    best.inlier_ratio = static_cast<double>(best.inliers) /
                        static_cast<double>(points.size());
    const double vertical_score = std::abs(best.direction.normalized().dot(Eigen::Vector3d::UnitZ()));
    best.vertical_angle_deg =
        std::acos(clamp(vertical_score, -1.0, 1.0)) * 180.0 / kPi;
    return best;
  }

  bool refineLineDirection(const std::vector<Eigen::Vector3d>& points,
                           const std::vector<int>& indices,
                           Eigen::Vector3d& direction) const
  {
    if (indices.size() < 2)
    {
      return false;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const int index : indices)
    {
      centroid += points[static_cast<size_t>(index)];
    }
    centroid /= static_cast<double>(indices.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const int index : indices)
    {
      const Eigen::Vector3d centered = points[static_cast<size_t>(index)] - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(std::max<size_t>(1, indices.size() - 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
      return false;
    }

    direction = solver.eigenvectors().col(2).normalized();
    if (direction.z() < 0.0)
    {
      direction = -direction;
    }
    return std::isfinite(direction.x()) && std::isfinite(direction.y()) &&
           std::isfinite(direction.z());
  }

  CircleFitResult fitCircleLeastSquares(const std::vector<Eigen::Vector3d>& points,
                                        const std::vector<int>& indices) const
  {
    CircleFitResult result;
    if (indices.size() < 3)
    {
      result.reject_reason = "circle fit points too few";
      return result;
    }

    Eigen::MatrixXd a(indices.size(), 3);
    Eigen::VectorXd b(indices.size());
    for (size_t row = 0; row < indices.size(); ++row)
    {
      const Eigen::Vector3d& point = points[static_cast<size_t>(indices[row])];
      a(static_cast<int>(row), 0) = point.x();
      a(static_cast<int>(row), 1) = point.y();
      a(static_cast<int>(row), 2) = 1.0;
      b(static_cast<int>(row)) = point.x() * point.x() + point.y() * point.y();
    }

    const Eigen::Vector3d solution = a.colPivHouseholderQr().solve(b);
    const double cx = solution.x() * 0.5;
    const double cy = solution.y() * 0.5;
    const double radius_sq = solution.z() + cx * cx + cy * cy;

    if (!std::isfinite(radius_sq) || radius_sq <= 0.0)
    {
      result.reject_reason = "circle radius squared is invalid";
      return result;
    }

    const double radius = std::sqrt(radius_sq);
    if (radius < config_.radius_min || radius > config_.radius_max)
    {
      std::ostringstream ss;
      ss << "circle radius out of range: " << radius;
      result.reject_reason = ss.str();
      return result;
    }

    result.valid = true;
    result.center = Eigen::Vector2d(cx, cy);
    result.radius = radius;
    return result;
  }

  void updateTowerState(const std::vector<Eigen::Vector3d>& points,
                        const LineFitResult& line,
                        const Eigen::Vector3d& drone_position,
                        const ros::Time& stamp)
  {
    Eigen::Vector3d inlier_mean = Eigen::Vector3d::Zero();
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    inlier_cloud->reserve(line.inlier_indices.size());

    for (const int index : line.inlier_indices)
    {
      const Eigen::Vector3d& point = points[static_cast<size_t>(index)];
      inlier_mean += point;
      z_min = std::min(z_min, point.z());
      z_max = std::max(z_max, point.z());

      pcl::PointXYZI p;
      p.x = static_cast<float>(point.x());
      p.y = static_cast<float>(point.y());
      p.z = static_cast<float>(point.z());
      p.intensity = 100.0f;
      inlier_cloud->push_back(p);
    }
    inlier_mean /= static_cast<double>(line.inlier_indices.size());
    inlier_cloud->width = static_cast<std::uint32_t>(inlier_cloud->size());
    inlier_cloud->height = 1;
    inlier_cloud->is_dense = false;

    const double new_x = inlier_mean.x();
    const double new_y = inlier_mean.y();
    double lateral_jump = 0.0;
    std::string state_action = "init";

    if (!has_ref_line_)
    {
      ref_x_ = new_x;
      ref_y_ = new_y;
      ref_z_min_ = z_min;
      ref_z_max_ = z_max;
      has_ref_line_ = true;
    }
    else
    {
      lateral_jump = std::hypot(new_x - ref_x_, new_y - ref_y_);
      if (lateral_jump <= config_.xy_tolerance)
      {
        ref_x_ = (1.0 - config_.xy_smoothing_alpha) * ref_x_ +
                 config_.xy_smoothing_alpha * new_x;
        ref_y_ = (1.0 - config_.xy_smoothing_alpha) * ref_y_ +
                 config_.xy_smoothing_alpha * new_y;
        ref_z_min_ = std::min(ref_z_min_, z_min);
        ref_z_max_ = std::max(ref_z_max_, z_max);
        state_action = "track";
      }
      else
      {
        ref_x_ = new_x;
        ref_y_ = new_y;
        ref_z_min_ = z_min;
        ref_z_max_ = z_max;
        state_action = "new_axis";
      }
    }

    if (!has_radius_ || !config_.estimate_diameter_once)
    {
      const CircleFitResult circle = fitCircleLeastSquares(points, line.inlier_indices);
      if (circle.valid)
      {
        circle_center_ = circle.center;
        radius_ = circle.radius;
        diameter_ = radius_ * 2.0;
        has_radius_ = true;
        publishDiameter();
      }
      else
      {
        ROS_WARN_THROTTLE(2.0,
                          "[WTBTowerDetector] %s",
                          circle.reject_reason.c_str());
      }
    }

    const Eigen::Vector3d control_point(ref_x_, ref_y_, drone_position.z());
    recordControlPoint(control_point);
    publishParams(control_point);
    publishEstimate(line, control_point, inlier_cloud, stamp);

    std::ostringstream ss;
    ss << "accepted " << state_action
       << ", sampled_points=" << points.size()
       << ", line_inliers=" << line.inliers
       << ", inlier_ratio=" << line.inlier_ratio
       << ", vertical_angle_deg=" << line.vertical_angle_deg
       << ", measured_xy=(" << new_x << "," << new_y << ")"
       << ", ref_xy=(" << ref_x_ << "," << ref_y_ << ")"
       << ", z=[" << ref_z_min_ << "," << ref_z_max_ << "]"
       << ", control_point=(" << control_point.x() << ","
       << control_point.y() << "," << control_point.z() << ")";
    if (has_radius_)
    {
      ss << ", diameter=" << diameter_;
    }
    if (state_action != "init")
    {
      ss << ", lateral_jump=" << lateral_jump;
    }
    publishStatus(ss.str());
  }

  void recordControlPoint(const Eigen::Vector3d& control_point)
  {
    if (!control_points_.empty() &&
        (control_point - control_points_.back()).norm() < config_.control_point_min_spacing)
    {
      return;
    }

    control_points_.push_back(control_point);
    if (static_cast<int>(control_points_.size()) > config_.max_path_points)
    {
      const size_t erase_count =
          control_points_.size() - static_cast<size_t>(config_.max_path_points);
      control_points_.erase(control_points_.begin(),
                            control_points_.begin() + static_cast<long>(erase_count));
    }
  }

  void publishEstimate(const LineFitResult& line,
                       const Eigen::Vector3d& control_point,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& inlier_cloud,
                       const ros::Time& stamp)
  {
    publishControlPoint(control_point, stamp);
    publishAxisDirection(stamp);
    publishPath(control_point, stamp);

    if (config_.publish_inlier_cloud)
    {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*inlier_cloud, msg);
      msg.header.frame_id = config_.world_frame;
      msg.header.stamp = stamp;
      inlier_cloud_pub_.publish(msg);
    }

    if (config_.publish_markers)
    {
      publishMarkers(line, control_point, stamp);
    }
  }

  void publishControlPoint(const Eigen::Vector3d& control_point, const ros::Time& stamp)
  {
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = config_.world_frame;
    point_msg.header.stamp = stamp;
    point_msg.point = toPointMsg(control_point);
    control_point_pub_.publish(point_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = point_msg.header;
    pose_msg.pose.position = point_msg.point;
    pose_msg.pose.orientation.w = 1.0;
    control_pose_pub_.publish(pose_msg);
  }

  void publishAxisDirection(const ros::Time& stamp)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.frame_id = config_.world_frame;
    msg.header.stamp = stamp;
    msg.vector.z = 1.0;
    axis_direction_pub_.publish(msg);
  }

  void publishPath(const Eigen::Vector3d& control_point, const ros::Time& stamp)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config_.world_frame;
    pose.header.stamp = stamp;
    pose.pose.position = toPointMsg(control_point);
    pose.pose.orientation.w = 1.0;

    tower_path_.header.frame_id = config_.world_frame;
    tower_path_.header.stamp = stamp;
    tower_path_.poses.push_back(pose);
    if (static_cast<int>(tower_path_.poses.size()) > config_.max_path_points)
    {
      const size_t erase_count =
          tower_path_.poses.size() - static_cast<size_t>(config_.max_path_points);
      tower_path_.poses.erase(tower_path_.poses.begin(),
                              tower_path_.poses.begin() + static_cast<long>(erase_count));
    }
    path_pub_.publish(tower_path_);
  }

  visualization_msgs::Marker makeMarker(const std::string& ns,
                                        int id,
                                        int type,
                                        const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = config_.world_frame;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);
    return marker;
  }

  void publishMarkers(const LineFitResult& line,
                      const Eigen::Vector3d& control_point,
                      const ros::Time& stamp)
  {
    (void)line;
    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker axis =
        makeMarker("tower_axis", 0, visualization_msgs::Marker::LINE_STRIP, stamp);
    axis.scale.x = 0.08;
    axis.color = color(0.0, 1.0, 0.2, 1.0);
    axis.points.push_back(toPointMsg(Eigen::Vector3d(ref_x_, ref_y_, ref_z_min_)));
    axis.points.push_back(toPointMsg(Eigen::Vector3d(ref_x_, ref_y_, ref_z_max_)));
    array.markers.push_back(axis);

    visualization_msgs::Marker cylinder =
        makeMarker("tower_axis", 1, visualization_msgs::Marker::CYLINDER, stamp);
    const double height = std::max(0.5, ref_z_max_ - ref_z_min_);
    const double cylinder_diameter = has_radius_ ? diameter_ : config_.marker_cylinder_diameter;
    cylinder.pose.position = toPointMsg(
        Eigen::Vector3d(ref_x_, ref_y_, 0.5 * (ref_z_min_ + ref_z_max_)));
    cylinder.scale.x = cylinder_diameter;
    cylinder.scale.y = cylinder_diameter;
    cylinder.scale.z = height;
    cylinder.color = color(0.0, 1.0, 0.2, 0.15);
    array.markers.push_back(cylinder);

    visualization_msgs::Marker control =
        makeMarker("tower_control_point", 0, visualization_msgs::Marker::SPHERE, stamp);
    control.pose.position = toPointMsg(control_point);
    control.scale.x = 0.35;
    control.scale.y = 0.35;
    control.scale.z = 0.35;
    control.color = color(1.0, 0.75, 0.05, 1.0);
    array.markers.push_back(control);

    visualization_msgs::Marker lower =
        makeMarker("tower_endpoints", 0, visualization_msgs::Marker::SPHERE, stamp);
    lower.pose.position = toPointMsg(Eigen::Vector3d(ref_x_, ref_y_, ref_z_min_));
    lower.scale.x = 0.25;
    lower.scale.y = 0.25;
    lower.scale.z = 0.25;
    lower.color = color(0.0, 0.7, 1.0, 1.0);
    array.markers.push_back(lower);

    visualization_msgs::Marker upper =
        makeMarker("tower_endpoints", 1, visualization_msgs::Marker::SPHERE, stamp);
    upper.pose.position = toPointMsg(Eigen::Vector3d(ref_x_, ref_y_, ref_z_max_));
    upper.scale.x = 0.25;
    upper.scale.y = 0.25;
    upper.scale.z = 0.25;
    upper.color = color(1.0, 0.7, 0.0, 1.0);
    array.markers.push_back(upper);

    if (has_radius_)
    {
      const double z_mid = 0.5 * (ref_z_min_ + ref_z_max_);
      visualization_msgs::Marker center =
          makeMarker("tower_radius", 0, visualization_msgs::Marker::SPHERE, stamp);
      center.pose.position =
          toPointMsg(Eigen::Vector3d(circle_center_.x(), circle_center_.y(), z_mid));
      center.scale.x = 0.3;
      center.scale.y = 0.3;
      center.scale.z = 0.3;
      center.color = color(1.0, 0.5, 0.0, 1.0);
      array.markers.push_back(center);

      visualization_msgs::Marker ring =
          makeMarker("tower_radius", 1, visualization_msgs::Marker::LINE_STRIP, stamp);
      ring.scale.x = 0.06;
      ring.color = color(1.0, 0.5, 0.0, 1.0);
      constexpr int kSegments = 72;
      for (int i = 0; i <= kSegments; ++i)
      {
        const double angle = 2.0 * kPi * static_cast<double>(i) /
                             static_cast<double>(kSegments);
        ring.points.push_back(toPointMsg(
            Eigen::Vector3d(circle_center_.x() + radius_ * std::cos(angle),
                            circle_center_.y() + radius_ * std::sin(angle),
                            z_mid)));
      }
      array.markers.push_back(ring);
    }

    if (!control_points_.empty())
    {
      visualization_msgs::Marker history =
          makeMarker("tower_control_history", 0, visualization_msgs::Marker::SPHERE_LIST, stamp);
      history.scale.x = 0.18;
      history.scale.y = 0.18;
      history.scale.z = 0.18;
      history.color = color(0.0, 1.0, 0.35, 1.0);

      visualization_msgs::Marker history_line =
          makeMarker("tower_control_history", 1, visualization_msgs::Marker::LINE_STRIP, stamp);
      history_line.scale.x = 0.04;
      history_line.color = color(0.0, 1.0, 0.35, 0.9);

      for (const Eigen::Vector3d& point : control_points_)
      {
        const geometry_msgs::Point point_msg = toPointMsg(point);
        history.points.push_back(point_msg);
        history_line.points.push_back(point_msg);
      }
      array.markers.push_back(history);
      array.markers.push_back(history_line);
    }

    marker_pub_.publish(array);
  }

  void publishDiameter() const
  {
    std_msgs::Float64 diameter_msg;
    diameter_msg.data = diameter_;
    diameter_pub_.publish(diameter_msg);

    std_msgs::Float64 radius_msg;
    radius_msg.data = radius_;
    radius_pub_.publish(radius_msg);
  }

  void publishParams(const Eigen::Vector3d& control_point) const
  {
    publishParamsToNamespace(config_.param_namespace, control_point);
    publishParamsToNamespace(config_.legacy_param_namespace, control_point);
  }

  void publishParamsToNamespace(const std::string& ns,
                                const Eigen::Vector3d& control_point) const
  {
    if (ns.empty())
    {
      return;
    }

    ros::param::set(ns + "/ref_x", ref_x_);
    ros::param::set(ns + "/ref_y", ref_y_);
    ros::param::set(ns + "/z_min", ref_z_min_);
    ros::param::set(ns + "/z_max", ref_z_max_);
    ros::param::set(ns + "/control_x", control_point.x());
    ros::param::set(ns + "/control_y", control_point.y());
    ros::param::set(ns + "/control_z", control_point.z());
    ros::param::set(ns + "/direction", std::vector<double>{0.0, 0.0, 1.0});
    if (has_radius_)
    {
      ros::param::set(ns + "/diameter", diameter_);
      ros::param::set(ns + "/radius", radius_);
      ros::param::set(ns + "/circle_cx", circle_center_.x());
      ros::param::set(ns + "/circle_cy", circle_center_.y());
    }
  }

  void publishStatus(const std::string& status) const
  {
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);

    if (config_.print_status)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "[WTBTowerDetector] " << status);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  TowerDetectorConfig config_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher status_pub_;
  ros::Publisher control_point_pub_;
  ros::Publisher control_pose_pub_;
  ros::Publisher axis_direction_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher inlier_cloud_pub_;
  ros::Publisher path_pub_;
  ros::Publisher diameter_pub_;
  ros::Publisher radius_pub_;

  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  bool has_odom_ = false;
  ros::Time last_update_time_;
  std::mt19937 rng_;

  bool has_ref_line_ = false;
  double ref_x_ = 0.0;
  double ref_y_ = 0.0;
  double ref_z_min_ = 0.0;
  double ref_z_max_ = 0.0;

  bool has_radius_ = false;
  double radius_ = 0.0;
  double diameter_ = 0.0;
  Eigen::Vector2d circle_center_ = Eigen::Vector2d::Zero();

  std::vector<Eigen::Vector3d> control_points_;
  nav_msgs::Path tower_path_;
};

}  // namespace wtb_pointcloud_mapping

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wtb_tower_detector_node");
  wtb_pointcloud_mapping::WTBTowerDetectorNode node;
  ros::spin();
  return 0;
}
