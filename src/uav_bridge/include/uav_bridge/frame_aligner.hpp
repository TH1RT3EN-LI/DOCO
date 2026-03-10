#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace uav_bridge
{

    struct FrameAlignerConfig
    {
        bool lock_on_first_valid{true};
        float alpha{0.02f};
        bool relock_on_reset{true};
        bool enable_yaw_alignment{true};
    };

    class FrameAligner
    {
    public:
        using Config = FrameAlignerConfig;

        explicit FrameAligner(
            Config config = Config{},
            rclcpp::Logger logger = rclcpp::get_logger("frame_aligner"));

        void updatePlannerPose(const std::array<float, 3> &enu_pos, float yaw_enu);

        void updatePx4Pose(
            const std::array<float, 3> &ned_pos,
            uint8_t xy_reset_counter,
            uint8_t z_reset_counter,
            float delta_xy_x = std::numeric_limits<float>::quiet_NaN(),
            float delta_xy_y = std::numeric_limits<float>::quiet_NaN(),
            float delta_z = std::numeric_limits<float>::quiet_NaN(),
            float heading_ned = std::numeric_limits<float>::quiet_NaN());

        std::array<float, 3> getOffsetNed() const { return frame_offset_ned_; }
        float getYawOffsetNed() const { return yaw_offset_ned_; }

        bool isAligned() const { return has_alignment_; }

        bool hasPx4Pose() const { return has_px4_pos_; }

        void forceRealign() { force_realign_ = true; }

        void reset();

    private:
        void compute();

        Config config_;
        rclcpp::Logger logger_;

        bool has_planner_pos_{false};
        bool has_planner_yaw_{false};
        bool has_px4_pos_{false};
        bool has_px4_heading_{false};
        bool has_alignment_{false};
        bool has_yaw_alignment_{false};
        bool force_realign_{false};

        std::array<float, 3> planner_enu_{0.0f, 0.0f, 0.0f};
        float planner_yaw_enu_{0.0f};
        std::array<float, 3> px4_ned_{0.0f, 0.0f, 0.0f};
        float px4_heading_ned_{0.0f};
        std::array<float, 3> frame_offset_ned_{0.0f, 0.0f, 0.0f};
        float yaw_offset_ned_{0.0f};

        uint8_t px4_xy_reset_counter_{0};
        uint8_t px4_z_reset_counter_{0};
    };

} // namespace uav_bridge
