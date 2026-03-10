#include "uav_bridge/math_utils.hpp"

#include <cmath>

namespace uav_bridge
{

    float normalizeAngle(float angle)
    {
        while (angle > static_cast<float>(M_PI))
        {
            angle -= static_cast<float>(2.0 * M_PI);
        }
        while (angle < static_cast<float>(-M_PI))
        {
            angle += static_cast<float>(2.0 * M_PI);
        }
        return angle;
    }

    float enuYawToNed(float yaw_enu)
    {
        return normalizeAngle(static_cast<float>(M_PI_2) - yaw_enu);
    }

    std::array<float, 3> enuPositionToNed(float x_enu, float y_enu, float z_enu)
    {
        return {y_enu, x_enu, -z_enu};
    }

    float quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    }

    bool isFiniteVector(const std::array<float, 3> &value)
    {
        return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
    }

    float vectorNorm(const std::array<float, 3> &value)
    {
        return std::sqrt(
            (value[0] * value[0]) +
            (value[1] * value[1]) +
            (value[2] * value[2]));
    }

    bool clampVectorNorm(std::array<float, 3> &value, float max_norm)
    {
        if (max_norm <= 0.0f || !isFiniteVector(value))
        {
            return false;
        }

        const float norm = vectorNorm(value);
        if (!std::isfinite(norm) || norm <= max_norm)
        {
            return false;
        }

        const float scale = max_norm / norm;
        value[0] *= scale;
        value[1] *= scale;
        value[2] *= scale;
        return true;
    }

} // namespace uav_bridge
