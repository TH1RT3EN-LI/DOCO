#include "ugv_sim_tools/point_cloud_utils.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>

namespace ugv_sim_tools
{

namespace
{

struct PointFieldInfo
{
  bool found{false};
  uint32_t offset{0};
  uint8_t datatype{0};
};

PointFieldInfo findField(
  const sensor_msgs::msg::PointCloud2 & input,
  const std::string & field_name)
{
  for (const auto & field : input.fields) {
    if (field.name == field_name) {
      return PointFieldInfo{true, field.offset, field.datatype};
    }
  }
  return PointFieldInfo{};
}

}

PointCloudTransformResult transformCloudCameraToOptical(
  const sensor_msgs::msg::PointCloud2 & input)
{
  const auto x = findField(input, "x");
  const auto y = findField(input, "y");
  const auto z = findField(input, "z");

  if (!x.found || !y.found || !z.found) {
    return PointCloudTransformResult{
      PointCloudTransformStatus::kMissingFields,
      sensor_msgs::msg::PointCloud2{},
      "point cloud lacks x/y/z fields; skip cloud republish"};
  }

  if (
    x.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    y.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    z.datatype != sensor_msgs::msg::PointField::FLOAT32)
  {
    return PointCloudTransformResult{
      PointCloudTransformStatus::kCopiedWithoutTransform,
      input,
      "point cloud x/y/z are not FLOAT32; skip coordinate transform"};
  }

  sensor_msgs::msg::PointCloud2 output = input;
  auto & data = output.data;

  const std::size_t height = output.height > 0 ? output.height : 1;
  const std::size_t width = output.width;
  const std::size_t point_step = output.point_step;
  const std::size_t row_step = output.row_step > 0 ? output.row_step : width * point_step;
  const std::size_t max_offset = std::max({x.offset, y.offset, z.offset});

  for (std::size_t row = 0; row < height; ++row) {
    const std::size_t row_base = row * row_step;
    for (std::size_t col = 0; col < width; ++col) {
      const std::size_t base = row_base + col * point_step;
      if (base + max_offset + sizeof(float) > data.size()) {
        return PointCloudTransformResult{
          PointCloudTransformStatus::kCopiedWithoutTransform,
          input,
          "point cloud data buffer is smaller than declared field layout; skip coordinate transform"};
      }

      float x_value = 0.0F;
      float y_value = 0.0F;
      float z_value = 0.0F;
      std::memcpy(&x_value, data.data() + base + x.offset, sizeof(float));
      std::memcpy(&y_value, data.data() + base + y.offset, sizeof(float));
      std::memcpy(&z_value, data.data() + base + z.offset, sizeof(float));

      const float x_optical = -y_value;
      const float y_optical = -z_value;
      const float z_optical = x_value;

      std::memcpy(data.data() + base + x.offset, &x_optical, sizeof(float));
      std::memcpy(data.data() + base + y.offset, &y_optical, sizeof(float));
      std::memcpy(data.data() + base + z.offset, &z_optical, sizeof(float));
    }
  }

  return PointCloudTransformResult{PointCloudTransformStatus::kTransformed, output, ""};
}

}
