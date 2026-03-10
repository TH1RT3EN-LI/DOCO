#pragma once

#include <array>

#include <geometry_msgs/msg/quaternion.hpp>

namespace uav_bridge
{
    float normalizeAngle(float angle);
    float enuYawToNed(float yaw_enu);
    std::array<float, 3> enuPositionToNed(float x_enu, float y_enu, float z_enu);
    float quaternionToYaw(const geometry_msgs::msg::Quaternion &q);
    bool isFiniteVector(const std::array<float, 3> &value);
    float vectorNorm(const std::array<float, 3> &value);
    bool clampVectorNorm(std::array<float, 3> &value, float max_norm);
}  // namespace uav_bridge