#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <livox_laser_simulation/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace livox_to_pointcloud2
{

using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointField = sensor_msgs::PointField;

struct FilterConfig
{
  double horizontal_fov_deg = 60.0;
  double vertical_fov_deg = 60.0;
  double min_range = 0.0;
  double max_range = std::numeric_limits<double>::infinity();
  std::string output_frame_id;
};

struct ConversionResult
{
  PointCloud2Ptr cropped_pointcloud2;
  livox_laser_simulation::CustomMsg cropped;
  std::size_t input_count = 0;
  std::size_t kept_count = 0;
};

class LivoxConverter
{
public:
  explicit LivoxConverter(const FilterConfig& config = FilterConfig()) : config_(sanitize(config)) {}

  void setConfig(const FilterConfig& config)
  {
    config_ = sanitize(config);
  }

  const FilterConfig& config() const
  {
    return config_;
  }

  ConversionResult convert(const livox_laser_simulation::CustomMsg& livox_msg) const
  {
    ConversionResult result;
    result.input_count = livox_msg.points.size();
    result.cropped.header = livox_msg.header;
    result.cropped.timebase = livox_msg.timebase;
    result.cropped.lidar_id = livox_msg.lidar_id;
    result.cropped.rsvd = livox_msg.rsvd;
    result.cropped.points.reserve(result.input_count);
    if (!config_.output_frame_id.empty())
    {
      result.cropped.header.frame_id = config_.output_frame_id;
    }

    for (const auto& point : livox_msg.points)
    {
      if (accepts(point.x, point.y, point.z))
      {
        result.cropped.points.push_back(point);
      }
    }

    result.kept_count = result.cropped.points.size();
    result.cropped.point_num = static_cast<std::uint32_t>(result.kept_count);
    result.cropped_pointcloud2 = createCloud(livox_msg, result.kept_count);

    std::uint8_t* destination = result.cropped_pointcloud2->data.data();
    for (const auto& point : result.cropped.points)
    {
      writePoint(destination, point);
      destination += result.cropped_pointcloud2->point_step;
    }

    return result;
  }

private:
  static constexpr std::uint32_t kXOffset = 0;
  static constexpr std::uint32_t kYOffset = 4;
  static constexpr std::uint32_t kZOffset = 8;
  static constexpr std::uint32_t kTimeOffset = 12;
  static constexpr std::uint32_t kIntensityOffset = 16;
  static constexpr std::uint32_t kTagOffset = 20;
  static constexpr std::uint32_t kLineOffset = 21;
  static constexpr std::uint32_t kPointStep = 22;
  static constexpr double kPi = 3.14159265358979323846;

  static FilterConfig sanitize(FilterConfig config)
  {
    config.horizontal_fov_deg = std::max(0.0, std::min(360.0, config.horizontal_fov_deg));
    config.vertical_fov_deg = std::max(0.0, std::min(180.0, config.vertical_fov_deg));
    config.min_range = std::max(0.0, config.min_range);
    if (config.max_range <= 0.0)
    {
      config.max_range = std::numeric_limits<double>::infinity();
    }
    config.max_range = std::max(config.min_range, config.max_range);
    return config;
  }

  bool accepts(const double x, const double y, const double z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      return false;
    }

    const double horizontal_range = std::hypot(x, y);
    const double range = std::hypot(horizontal_range, z);
    if (range < config_.min_range || range > config_.max_range)
    {
      return false;
    }

    const double horizontal_angle = std::atan2(y, x);
    const double vertical_angle = std::atan2(z, horizontal_range);
    const double horizontal_half_angle = config_.horizontal_fov_deg * kPi / 360.0;
    const double vertical_half_angle = config_.vertical_fov_deg * kPi / 360.0;

    return std::abs(horizontal_angle) <= horizontal_half_angle &&
           std::abs(vertical_angle) <= vertical_half_angle;
  }

  static void addField(PointCloud2& cloud, const std::string& name, const std::uint32_t offset,
                       const std::uint8_t datatype)
  {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  static void configureFields(PointCloud2& cloud)
  {
    addField(cloud, "x", kXOffset, PointField::FLOAT32);
    addField(cloud, "y", kYOffset, PointField::FLOAT32);
    addField(cloud, "z", kZOffset, PointField::FLOAT32);
    addField(cloud, "t", kTimeOffset, PointField::UINT32);
    addField(cloud, "intensity", kIntensityOffset, PointField::FLOAT32);
    addField(cloud, "tag", kTagOffset, PointField::UINT8);
    addField(cloud, "line", kLineOffset, PointField::UINT8);
    cloud.is_bigendian = false;
    cloud.point_step = kPointStep;
  }

  PointCloud2Ptr createCloud(const livox_laser_simulation::CustomMsg& livox_msg,
                             const std::size_t point_count) const
  {
    PointCloud2Ptr cloud(new PointCloud2);
    cloud->header = livox_msg.header;
    if (!config_.output_frame_id.empty())
    {
      cloud->header.frame_id = config_.output_frame_id;
    }
    configureFields(*cloud);
    cloud->height = 1;
    cloud->width = static_cast<std::uint32_t>(point_count);
    cloud->row_step = cloud->width * cloud->point_step;
    cloud->data.resize(cloud->row_step);
    cloud->is_dense = true;
    return cloud;
  }

  static void writePoint(std::uint8_t* destination,
                         const livox_laser_simulation::CustomPoint& point)
  {
    const float intensity = static_cast<float>(point.reflectivity);
    write(destination, kXOffset, point.x);
    write(destination, kYOffset, point.y);
    write(destination, kZOffset, point.z);
    write(destination, kTimeOffset, point.offset_time);
    write(destination, kIntensityOffset, intensity);
    write(destination, kTagOffset, point.tag);
    write(destination, kLineOffset, point.line);
  }

  template <typename Value>
  static void write(std::uint8_t* destination, const std::uint32_t offset, const Value& value)
  {
    std::memcpy(destination + offset, &value, sizeof(Value));
  }

  FilterConfig config_;
};

}  // namespace livox_to_pointcloud2
