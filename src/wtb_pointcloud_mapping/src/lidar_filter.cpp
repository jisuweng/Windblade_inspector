#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <livox_laser_simulation/CustomMsg.h>
#include <ros/ros.h>

namespace
{

class LidarFilter
{
public:
  LidarFilter() : nh_(), private_nh_("~")
  {
    loadParams();
    initializeRos();
  }

private:
  template <typename T>
  void readParam(const std::string& key, T& value, const T& default_value)
  {
    if (private_nh_.getParam(key, value))
    {
      return;
    }
    if (nh_.getParam(key, value))
    {
      return;
    }
    value = default_value;
  }

  void loadParams()
  {
    readParam<std::string>("lidar_filter/input_topic", input_topic_, "/livox/lidar");
    readParam<std::string>("lidar_filter/output_topic", output_topic_, "/livox/lidar_filtered");

    readParam<double>("lidar_filter/ground_remove_z", ground_remove_z_, -999.0);
    readParam<double>("lidar_filter/z_max", z_max_, 3.0);
    readParam<double>("lidar_filter/range_min", range_min_, 1.0);
    readParam<double>("lidar_filter/range_max", range_max_, 8.0);
    readParam<double>("lidar_filter/front_angle_deg", front_angle_deg_, 90.0);
    readParam<double>("lidar_filter/x_min", x_min_, 0.0);
    readParam<double>("lidar_filter/y_abs", y_abs_, -1.0);
    readParam<bool>("lidar_filter/use_3d_range", use_3d_range_, false);

    range_min_ = std::max(0.0, range_min_);
    range_max_ = std::max(range_min_, range_max_);
    front_angle_deg_ = std::max(0.0, std::min(360.0, front_angle_deg_));
  }

  void initializeRos()
  {
    cloud_pub_ = nh_.advertise<livox_laser_simulation::CustomMsg>(output_topic_, 10);
    cloud_sub_ = nh_.subscribe(input_topic_, 10, &LidarFilter::cloudCallback, this);

    ROS_INFO_STREAM("[LidarFilter] subscribing: " << input_topic_);
    ROS_INFO_STREAM("[LidarFilter] publishing: " << output_topic_);
    ROS_INFO_STREAM("[LidarFilter] preserving input point coordinates and frame_id");
    ROS_INFO_STREAM("[LidarFilter] ground_remove_z=" << ground_remove_z_
                    << ", z_max=" << z_max_
                    << ", range=[" << range_min_ << ", " << range_max_ << "]"
                    << ", front_angle_deg=" << front_angle_deg_
                    << ", x_min=" << x_min_
                    << ", y_abs=" << y_abs_
                    << ", use_3d_range=" << (use_3d_range_ ? "true" : "false"));
  }

  bool keepPoint(const livox_laser_simulation::CustomPoint& point) const
  {
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      return false;
    }

    if (ground_remove_z_ > -999.0 && z < ground_remove_z_)
    {
      return false;
    }
    if (z > z_max_)
    {
      return false;
    }
    if (x < x_min_)
    {
      return false;
    }

    const double point_range = use_3d_range_ ? std::sqrt(x * x + y * y + z * z) :
                                               std::sqrt(x * x + y * y);
    if (point_range < range_min_ || point_range > range_max_)
    {
      return false;
    }

    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const double angle_deg = std::atan2(y, x) * kRadToDeg;
    if (std::abs(angle_deg) > front_angle_deg_ * 0.5)
    {
      return false;
    }

    if (y_abs_ > 0.0 && std::abs(y) > y_abs_)
    {
      return false;
    }

    return true;
  }

  void cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }

    livox_laser_simulation::CustomMsg filtered_msg;
    filtered_msg.header = msg->header;
    filtered_msg.timebase = msg->timebase;
    filtered_msg.lidar_id = msg->lidar_id;
    filtered_msg.rsvd = msg->rsvd;
    filtered_msg.points.reserve(msg->points.size());

    for (const auto& point : msg->points)
    {
      if (keepPoint(point))
      {
        filtered_msg.points.push_back(point);
      }
    }

    filtered_msg.point_num = static_cast<std::uint32_t>(filtered_msg.points.size());
    cloud_pub_.publish(filtered_msg);

    total_frames_ += 1;
    total_raw_points_ += msg->points.size();
    total_filtered_points_ += filtered_msg.points.size();

    const double keep_percent =
        100.0 * static_cast<double>(filtered_msg.points.size()) /
        std::max<std::size_t>(1, msg->points.size());
    ROS_INFO_STREAM_THROTTLE(5.0,
                             "[LidarFilter] filtered "
                                 << filtered_msg.points.size() << " / " << msg->points.size()
                                 << " pts (" << keep_percent
                                 << "%), frame_id=" << msg->header.frame_id);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;
  double ground_remove_z_ = -999.0;
  double z_max_ = 3.0;
  double range_min_ = 1.0;
  double range_max_ = 8.0;
  double front_angle_deg_ = 90.0;
  double x_min_ = 0.0;
  double y_abs_ = -1.0;
  bool use_3d_range_ = false;

  std::size_t total_frames_ = 0;
  std::size_t total_raw_points_ = 0;
  std::size_t total_filtered_points_ = 0;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_filter");
  LidarFilter node;
  ros::spin();
  return 0;
}
