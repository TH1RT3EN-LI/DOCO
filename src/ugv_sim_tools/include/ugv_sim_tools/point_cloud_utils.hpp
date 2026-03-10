#pragma once

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ugv_sim_tools
{

enum class PointCloudTransformStatus
{
  kTransformed,
  kCopiedWithoutTransform,
  kMissingFields,
};

struct PointCloudTransformResult
{
  PointCloudTransformStatus status{PointCloudTransformStatus::kTransformed};
  sensor_msgs::msg::PointCloud2 cloud;
  std::string warning_message;
};

PointCloudTransformResult transformCloudCameraToOptical(
  const sensor_msgs::msg::PointCloud2 & input);

}
