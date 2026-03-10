#include "uav_bridge/frame_aligner.hpp"

#include <cmath>

#include "uav_bridge/math_utils.hpp"

namespace uav_bridge
{

    namespace
    {
        std::array<float, 2> rotateNedXY(const std::array<float, 2> &xy, float yaw)
        {
            const float c = std::cos(yaw);
            const float s = std::sin(yaw);
            return {(c * xy[0]) - (s * xy[1]), (s * xy[0]) + (c * xy[1])};
        }
    } // namespace

    FrameAligner::FrameAligner(Config config, rclcpp::Logger logger)
        : config_(config), logger_(logger)
    {
    }

    void FrameAligner::updatePlannerPose(const std::array<float, 3> &enu_pos, float yaw_enu)
    {
        planner_enu_ = enu_pos;
        planner_yaw_enu_ = yaw_enu;
        has_planner_pos_ = true;
        has_planner_yaw_ = std::isfinite(yaw_enu);
        compute();
    }

    void FrameAligner::updatePx4Pose(
        const std::array<float, 3> &ned_pos,
        uint8_t xy_reset_counter,
        uint8_t z_reset_counter,
        float delta_xy_x,
        float delta_xy_y,
        float delta_z,
        float heading_ned)
    {
        if (config_.relock_on_reset && has_px4_pos_)
        {
            const bool xy_reset = (xy_reset_counter != px4_xy_reset_counter_);
            const bool z_reset = (z_reset_counter != px4_z_reset_counter_);
            if (xy_reset || z_reset)
            {
                if (has_alignment_ &&
                    std::isfinite(delta_xy_x) &&
                    std::isfinite(delta_xy_y) &&
                    std::isfinite(delta_z))
                {
                    frame_offset_ned_[0] += delta_xy_x;
                    frame_offset_ned_[1] += delta_xy_y;
                    frame_offset_ned_[2] += delta_z;
                    force_realign_ = false;

                    RCLCPP_WARN(
                        logger_,
                        "vehicle_local_position reset detected (xy:%u->%u z:%u->%u), "
                        "shifted frame alignment by [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                        px4_xy_reset_counter_, xy_reset_counter,
                        px4_z_reset_counter_, z_reset_counter,
                        delta_xy_x, delta_xy_y, delta_z,
                        frame_offset_ned_[0], frame_offset_ned_[1], frame_offset_ned_[2]);
                }
                else
                {
                    force_realign_ = true;

                    RCLCPP_WARN(
                        logger_,
                        "vehicle_local_position reset detected (xy:%u->%u z:%u->%u), "
                        "relocking frame alignment",
                        px4_xy_reset_counter_, xy_reset_counter,
                        px4_z_reset_counter_, z_reset_counter);
                }
            }
        }

        px4_xy_reset_counter_ = xy_reset_counter;
        px4_z_reset_counter_ = z_reset_counter;
        px4_ned_ = ned_pos;
        if (std::isfinite(heading_ned))
        {
            px4_heading_ned_ = heading_ned;
            has_px4_heading_ = true;
        }
        has_px4_pos_ = true;
        compute();
    }

    void FrameAligner::reset()
    {
        has_planner_pos_ = false;
        has_planner_yaw_ = false;
        has_px4_pos_ = false;
        has_px4_heading_ = false;
        has_alignment_ = false;
        has_yaw_alignment_ = false;
        force_realign_ = false;
        planner_enu_ = {0.0f, 0.0f, 0.0f};
        planner_yaw_enu_ = 0.0f;
        px4_ned_ = {0.0f, 0.0f, 0.0f};
        px4_heading_ned_ = 0.0f;
        frame_offset_ned_ = {0.0f, 0.0f, 0.0f};
        yaw_offset_ned_ = 0.0f;
        px4_xy_reset_counter_ = 0;
        px4_z_reset_counter_ = 0;
    }

    void FrameAligner::compute()
    {
        if (!has_planner_pos_ || !has_px4_pos_)
        {
            return;
        }

        const auto planner_ned = enuPositionToNed(
            planner_enu_[0], planner_enu_[1], planner_enu_[2]);

        const bool yaw_inputs_valid =
            config_.enable_yaw_alignment &&
            has_planner_yaw_ &&
            has_px4_heading_ &&
            std::isfinite(px4_heading_ned_);

        float target_yaw_offset_ned = yaw_offset_ned_;
        if (yaw_inputs_valid)
        {
            const float planner_yaw_ned = enuYawToNed(planner_yaw_enu_);
            target_yaw_offset_ned = normalizeAngle(px4_heading_ned_ - planner_yaw_ned);
        }

        std::array<float, 2> planner_xy_rotated = {planner_ned[0], planner_ned[1]};
        if (config_.enable_yaw_alignment)
        {
            planner_xy_rotated = rotateNedXY(planner_xy_rotated, target_yaw_offset_ned);
        }

        const std::array<float, 3> target_offset = {
            px4_ned_[0] - planner_xy_rotated[0],
            px4_ned_[1] - planner_xy_rotated[1],
            px4_ned_[2] - planner_ned[2]};

        if (!has_alignment_ || force_realign_)
        {
            const bool is_relock = has_alignment_ && force_realign_;
            frame_offset_ned_ = target_offset;
            yaw_offset_ned_ = target_yaw_offset_ned;
            has_alignment_ = true;
            has_yaw_alignment_ = yaw_inputs_valid || !config_.enable_yaw_alignment;
            force_realign_ = false;

            if (is_relock)
            {
                RCLCPP_INFO(
                    logger_,
                    "frame alignment relocked (NED offset): [%.3f, %.3f, %.3f], yaw offset=%.3f rad",
                    frame_offset_ned_[0], frame_offset_ned_[1], frame_offset_ned_[2], yaw_offset_ned_);
            }
            else
            {
                RCLCPP_INFO(
                    logger_,
                    "frame alignment locked (NED offset): [%.3f, %.3f, %.3f], yaw offset=%.3f rad",
                    frame_offset_ned_[0], frame_offset_ned_[1], frame_offset_ned_[2], yaw_offset_ned_);
            }
            return;
        }

        if (config_.enable_yaw_alignment && yaw_inputs_valid && !has_yaw_alignment_)
        {
            yaw_offset_ned_ = target_yaw_offset_ned;
            frame_offset_ned_ = target_offset;
            has_yaw_alignment_ = true;
            RCLCPP_INFO(
                logger_,
                "frame yaw alignment locked after delayed heading availability: yaw offset=%.3f rad",
                yaw_offset_ned_);
        }

        if (!config_.lock_on_first_valid)
        {
            const float a = config_.alpha;
            for (int i = 0; i < 3; ++i)
            {
                frame_offset_ned_[i] = (1.0f - a) * frame_offset_ned_[i] + a * target_offset[i];
            }
            if (config_.enable_yaw_alignment && yaw_inputs_valid)
            {
                const float yaw_error = normalizeAngle(target_yaw_offset_ned - yaw_offset_ned_);
                yaw_offset_ned_ = normalizeAngle(yaw_offset_ned_ + (a * yaw_error));
            }
        }
    }

} // namespace uav_bridge
