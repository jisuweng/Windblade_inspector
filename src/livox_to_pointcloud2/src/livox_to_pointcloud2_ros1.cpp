#include <algorithm>
#include <cstddef>
#include <string>

#include <livox_laser_simulation/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <livox_to_pointcloud2/livox_converter.hpp>

namespace livox_to_pointcloud2
{

class LivoxToPointCloud2
{
public:
  LivoxToPointCloud2() : nh_(), private_nh_("~")
  {
    loadParameters();
    if (publish_debug_pcl2_ && !debug_pcl2_topic_.empty())
    {
      debug_pcl2_publisher_ =
          nh_.advertise<sensor_msgs::PointCloud2>(debug_pcl2_topic_, queue_size_);
    }
    cropped_publisher_ =
        nh_.advertise<livox_laser_simulation::CustomMsg>(cropped_topic_, queue_size_);
    subscriber_ = nh_.subscribe(input_topic_, queue_size_, &LivoxToPointCloud2::cloudCallback,
                                this, ros::TransportHints().tcpNoDelay());

    const FilterConfig& config = converter_.config();
    ROS_INFO_STREAM("[LivoxToPointCloud2] subscribing to livox_laser_simulation/CustomMsg: "
                    << input_topic_);
    if (publish_debug_pcl2_ && !debug_pcl2_topic_.empty())
    {
      ROS_INFO_STREAM("[LivoxToPointCloud2] cropped debug PointCloud2: " << debug_pcl2_topic_);
    }
    else
    {
      ROS_INFO_STREAM("[LivoxToPointCloud2] cropped debug PointCloud2 output disabled");
    }
    ROS_INFO_STREAM("[LivoxToPointCloud2] cropped Livox CustomMsg: " << cropped_topic_);
    ROS_INFO_STREAM("[LivoxToPointCloud2] horizontal_fov=" << config.horizontal_fov_deg
                    << " deg, vertical_fov=" << config.vertical_fov_deg
                    << " deg, range=[" << config.min_range << ", "
                    << (std::isfinite(config.max_range) ? std::to_string(config.max_range) : "inf")
                    << "] m, forward_axis=+X");
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    private_nh_.param<std::string>("cropped_topic", cropped_topic_, "/livox/lidar_filtered");
    private_nh_.param<std::string>("debug_pcl2_topic", debug_pcl2_topic_,
                                   "/livox/debug/lidar_filtered_pcl2");
    private_nh_.param<bool>("publish_debug_pcl2", publish_debug_pcl2_, true);
    private_nh_.param<int>("queue_size", queue_size_, 10);

    FilterConfig config;
    private_nh_.param<double>("horizontal_fov_deg", config.horizontal_fov_deg, 60.0);
    private_nh_.param<double>("vertical_fov_deg", config.vertical_fov_deg, 60.0);
    private_nh_.param<double>("min_range", config.min_range, 0.0);
    private_nh_.param<double>("max_range", config.max_range, -1.0);
    private_nh_.param<std::string>("output_frame_id", config.output_frame_id, "");
    converter_.setConfig(config);

    queue_size_ = std::max(1, queue_size_);
  }

  void cloudCallback(const livox_laser_simulation::CustomMsgConstPtr& input)
  {
    const ConversionResult result = converter_.convert(*input);
    if (publish_debug_pcl2_ && debug_pcl2_publisher_)
    {
      debug_pcl2_publisher_.publish(result.cropped_pointcloud2);
    }
    cropped_publisher_.publish(result.cropped);

    const double keep_percentage =
        100.0 * static_cast<double>(result.kept_count) /
        std::max<std::size_t>(1, result.input_count);
    ROS_INFO_STREAM_THROTTLE(5.0, "[LivoxToPointCloud2] converted " << result.input_count
                                                                    << " points, cropped to "
                                                                    << result.kept_count << " ("
                                                                    << keep_percentage
                                                                    << "%), frame_id="
                                                                    << result.cropped.header.frame_id);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher debug_pcl2_publisher_;
  ros::Publisher cropped_publisher_;
  LivoxConverter converter_;

  std::string input_topic_;
  std::string cropped_topic_;
  std::string debug_pcl2_topic_;
  bool publish_debug_pcl2_ = true;
  int queue_size_ = 10;
};

}  // namespace livox_to_pointcloud2

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_to_pointcloud2");
  livox_to_pointcloud2::LivoxToPointCloud2 node;
  ros::spin();
  return 0;
}
