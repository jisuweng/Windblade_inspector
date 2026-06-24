#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>
#include <livox_laser_simulation/CustomMsg.h>

#include <livox_to_pointcloud2/livox_converter.hpp>

namespace
{

livox_laser_simulation::CustomPoint pointAtAngles(const double horizontal_deg,
                                                   const double vertical_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  const double horizontal = horizontal_deg * kPi / 180.0;
  const double vertical = vertical_deg * kPi / 180.0;
  const double horizontal_range = std::cos(vertical);

  livox_laser_simulation::CustomPoint point;
  point.x = static_cast<float>(horizontal_range * std::cos(horizontal));
  point.y = static_cast<float>(horizontal_range * std::sin(horizontal));
  point.z = static_cast<float>(std::sin(vertical));
  point.reflectivity = 42;
  return point;
}

float readFloat(const sensor_msgs::PointCloud2& cloud, const std::size_t point_index,
                const std::size_t field_offset)
{
  float value = 0.0F;
  std::memcpy(&value, cloud.data.data() + point_index * cloud.point_step + field_offset,
              sizeof(value));
  return value;
}

TEST(LivoxConverter, PublishesCroppedPointCloud2AndCroppedLivoxCloud)
{
  livox_laser_simulation::CustomMsg input;
  input.header.frame_id = "livox_link";
  input.timebase = 123456U;
  input.lidar_id = 7U;
  input.points = {
      pointAtAngles(0.0, 0.0),
      pointAtAngles(29.9, 0.0),
      pointAtAngles(-29.9, 29.9),
      pointAtAngles(30.1, 0.0),
      pointAtAngles(0.0, -30.1),
      pointAtAngles(180.0, 0.0),
  };
  livox_laser_simulation::CustomPoint invalid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  input.points.push_back(invalid);
  input.point_num = input.points.size();

  livox_to_pointcloud2::LivoxConverter converter;
  const livox_to_pointcloud2::ConversionResult result = converter.convert(input);

  EXPECT_EQ(7U, result.input_count);
  EXPECT_EQ(3U, result.kept_count);
  ASSERT_EQ(3U, result.cropped_pointcloud2->width);
  ASSERT_EQ(3U, result.cropped.point_num);
  ASSERT_EQ(3U, result.cropped.points.size());
  EXPECT_TRUE(result.cropped_pointcloud2->is_dense);
  EXPECT_EQ("livox_link", result.cropped_pointcloud2->header.frame_id);
  EXPECT_EQ("livox_link", result.cropped.header.frame_id);
  EXPECT_EQ(input.timebase, result.cropped.timebase);
  EXPECT_EQ(input.lidar_id, result.cropped.lidar_id);
  EXPECT_FLOAT_EQ(42.0F, readFloat(*result.cropped_pointcloud2, 0, 16));
  EXPECT_EQ(42U, result.cropped.points[0].reflectivity);
}

TEST(LivoxConverter, RangeAffectsBothCroppedOutputsAndFrameOverrideAffectsBoth)
{
  livox_laser_simulation::CustomMsg input;
  input.header.frame_id = "sensor";
  input.points = { pointAtAngles(0.0, 0.0), pointAtAngles(0.0, 0.0) };
  input.points[0].x *= 0.5F;
  input.points[1].x *= 2.0F;
  input.point_num = input.points.size();

  livox_to_pointcloud2::FilterConfig config;
  config.min_range = 1.0;
  config.max_range = 3.0;
  config.output_frame_id = "body";
  livox_to_pointcloud2::LivoxConverter converter(config);
  const livox_to_pointcloud2::ConversionResult result = converter.convert(input);

  EXPECT_EQ(1U, result.cropped_pointcloud2->width);
  ASSERT_EQ(1U, result.cropped.point_num);
  ASSERT_EQ(1U, result.cropped.points.size());
  EXPECT_EQ("body", result.cropped_pointcloud2->header.frame_id);
  EXPECT_EQ("body", result.cropped.header.frame_id);
  EXPECT_FLOAT_EQ(2.0F, result.cropped.points[0].x);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
