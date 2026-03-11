/**
 * @file optical_flow_bridge_node.cpp
 * @brief Bridge GZ downward camera -> PX4 SensorOpticalFlow via uXRCE-DDS
 *
 * Subscribes to the Gazebo camera image topic from the downward optical_flow
 * camera sensor, computes optical flow using a hybrid estimator
 * (sparse LK + phase-correlation fallback), and publishes
 * px4_msgs::msg::SensorOpticalFlow on /uav/fmu/in/sensor_optical_flow.
 *
 * This replaces the PX4 main-branch gz_plugins/OpticalFlow* + gz_bridge approach
 * with a standalone ROS 2 node that doesn't require modifying PX4 source.
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>

#include <uav_bridge/gz_topic_utils.hpp>

namespace uav_bridge {

class OpticalFlowBridgeNode : public rclcpp::Node {
public:
  OpticalFlowBridgeNode() : Node("optical_flow_bridge_node") {
    // ---------- parameters ----------
    declare_parameter<std::string>("gz_world_name", "test");
    declare_parameter<std::string>("model_name", "uav");
    declare_parameter<std::string>("link_name", "base_link");
    declare_parameter<std::string>("sensor_name", "optical_flow_camera");
    declare_parameter<std::string>("range_sensor_name", "optical_flow_range");
    declare_parameter<std::string>("imu_sensor_name", "imu_sensor");
    declare_parameter<std::string>("gz_topic_override", "");
    declare_parameter<std::string>("gz_range_topic_override", "");
    declare_parameter<std::string>("gz_imu_topic_override", "");
    declare_parameter<std::string>("ros_topic", "/uav/fmu/in/sensor_optical_flow");
    declare_parameter<double>("camera_hfov", 0.25);         // radians
    declare_parameter<int>("image_width", 64);
    declare_parameter<int>("image_height", 64);
    declare_parameter<double>("max_flow_rate", 7.4);        // rad/s
    declare_parameter<double>("min_ground_distance", 0.08); // m
    declare_parameter<double>("max_ground_distance", 8.0);  // m
    declare_parameter<int>("quality_threshold", 20);        // 0-255
    declare_parameter<double>("roi_ratio", 0.8);            // center ROI ratio in [0,1]
    declare_parameter<int>("max_features", 120);
    declare_parameter<int>("min_features", 6);
    declare_parameter<double>("lk_quality_level", 0.001);
    declare_parameter<double>("lk_max_error", 20.0);
    declare_parameter<double>("phase_corr_min_response", 0.08);
    declare_parameter<double>("flow_deadband_px", 0.04);
    declare_parameter<double>("max_integration_time_ms", 80.0);
    declare_parameter<double>("range_sample_timeout_ms", 100.0);
    declare_parameter<double>("range_min_valid_m", 0.08);
    declare_parameter<double>("range_max_valid_m", 8.0);
    declare_parameter<bool>("use_imu_delta_angle", true);
    declare_parameter<bool>("imu_is_flu_frame", true);
    declare_parameter<double>("imu_min_coverage_ratio", 0.7);

    const auto gz_world_name = get_parameter("gz_world_name").as_string();
    const auto model  = get_parameter("model_name").as_string();
    const auto link   = get_parameter("link_name").as_string();
    const auto sensor = get_parameter("sensor_name").as_string();
    const auto range_sensor = get_parameter("range_sensor_name").as_string();
    const auto imu_sensor = get_parameter("imu_sensor_name").as_string();
    auto gz_topic = get_parameter("gz_topic_override").as_string();
    auto gz_range_topic = get_parameter("gz_range_topic_override").as_string();
    auto gz_imu_topic = get_parameter("gz_imu_topic_override").as_string();
    const auto ros_topic = get_parameter("ros_topic").as_string();
    camera_hfov_      = get_parameter("camera_hfov").as_double();
    image_width_      = get_parameter("image_width").as_int();
    image_height_     = get_parameter("image_height").as_int();
    max_flow_rate_ = get_parameter("max_flow_rate").as_double();
    min_ground_dist_ = get_parameter("min_ground_distance").as_double();
    max_ground_dist_ = get_parameter("max_ground_distance").as_double();
    range_sample_timeout_us_ = static_cast<uint64_t>(
      std::max(1.0, get_parameter("range_sample_timeout_ms").as_double()) * 1000.0);
    range_min_valid_m_ = std::max(0.01, get_parameter("range_min_valid_m").as_double());
    range_max_valid_m_ = std::max(
      range_min_valid_m_ + 0.01, get_parameter("range_max_valid_m").as_double());
    quality_threshold_ = get_parameter("quality_threshold").as_int();
    roi_ratio_ = std::clamp(get_parameter("roi_ratio").as_double(), 0.2, 1.0);
    max_features_ = std::max(20, static_cast<int>(get_parameter("max_features").as_int()));
    min_features_ = std::max(4, static_cast<int>(get_parameter("min_features").as_int()));
    min_features_ = std::min(min_features_, max_features_);
    lk_quality_level_ = std::clamp(get_parameter("lk_quality_level").as_double(), 1.0e-5, 0.1);
    lk_max_error_ = std::max(1.0, get_parameter("lk_max_error").as_double());
    phase_corr_min_response_ = std::clamp(
      get_parameter("phase_corr_min_response").as_double(), 0.0, 1.0);
    flow_deadband_px_ = std::max(0.0, get_parameter("flow_deadband_px").as_double());
    max_dt_us_ = static_cast<uint64_t>(
      std::max(5.0, get_parameter("max_integration_time_ms").as_double()) * 1000.0);
    use_imu_delta_angle_ = get_parameter("use_imu_delta_angle").as_bool();
    imu_is_flu_frame_ = get_parameter("imu_is_flu_frame").as_bool();
    imu_min_coverage_ratio_ = std::clamp(
      get_parameter("imu_min_coverage_ratio").as_double(), 0.1, 1.0);

    // Focal length in pixels
    focal_length_px_ = (static_cast<double>(image_width_) / 2.0) /
                       std::tan(camera_hfov_ / 2.0);

    if (gz_topic.empty()) {
      gz_topic = gz_topics::Image(gz_world_name, model, link, sensor);
    }
    if (gz_range_topic.empty()) {
      gz_range_topic = gz_topics::Scan(gz_world_name, model, link, range_sensor);
    }
    if (gz_imu_topic.empty()) {
      gz_imu_topic = gz_topics::Imu(gz_world_name, model, link, imu_sensor);
    }

    pub_ = create_publisher<px4_msgs::msg::SensorOpticalFlow>(
        ros_topic, rclcpp::SensorDataQoS());

    bool ok = gz_node_.Subscribe(gz_topic, &OpticalFlowBridgeNode::OnGzImage, this);
    if (!ok) {
      RCLCPP_FATAL(get_logger(), "Failed to subscribe to GZ topic: %s", gz_topic.c_str());
      throw std::runtime_error("GZ optical flow camera subscribe failed");
    }
    ok = gz_node_.Subscribe(gz_range_topic, &OpticalFlowBridgeNode::OnGzRange, this);
    if (!ok) {
      RCLCPP_FATAL(get_logger(), "Failed to subscribe to GZ range topic: %s", gz_range_topic.c_str());
      throw std::runtime_error("GZ optical flow range subscribe failed");
    }
    if (use_imu_delta_angle_) {
      ok = gz_node_.Subscribe(gz_imu_topic, &OpticalFlowBridgeNode::OnGzImu, this);
      if (!ok) {
        RCLCPP_FATAL(get_logger(), "Failed to subscribe to GZ IMU topic: %s", gz_imu_topic.c_str());
        throw std::runtime_error("GZ IMU subscribe failed");
      }
    }

    RCLCPP_INFO(get_logger(),
                "Optical flow bridge: GZ image [%s] range [%s] IMU [%s] -> ROS [%s] (focal=%.1f px, hfov=%.3f rad)",
                gz_topic.c_str(),
                gz_range_topic.c_str(),
                use_imu_delta_angle_ ? gz_imu_topic.c_str() : "<disabled>",
                ros_topic.c_str(),
                focal_length_px_,
                camera_hfov_);
  }

private:
  static float Median(std::vector<float> &values) {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    auto mid = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), mid, values.end());
    float med = *mid;
    if (values.size() % 2 == 0) {
      auto max_lower = std::max_element(values.begin(), mid);
      med = 0.5f * (med + *max_lower);
    }
    return med;
  }

  cv::Rect BuildRoi(int w, int h) const {
    const int roi_w = std::max(8, static_cast<int>(std::round(w * roi_ratio_)));
    const int roi_h = std::max(8, static_cast<int>(std::round(h * roi_ratio_)));
    const int x0 = std::max(0, (w - roi_w) / 2);
    const int y0 = std::max(0, (h - roi_h) / 2);
    return cv::Rect(x0, y0, std::min(roi_w, w - x0), std::min(roi_h, h - y0));
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<px4_msgs::msg::SensorOpticalFlow>::SharedPtr pub_;

  double camera_hfov_{0.25};
  int image_width_{64};
  int image_height_{64};
  double focal_length_px_{128.0};
  double max_flow_rate_{7.4};
  double min_ground_dist_{0.1};
  double max_ground_dist_{30.0};
  int quality_threshold_{20};
  double roi_ratio_{0.8};
  int max_features_{120};
  int min_features_{6};
  double lk_quality_level_{0.001};
  double lk_max_error_{20.0};
  double phase_corr_min_response_{0.08};
  double flow_deadband_px_{0.04};
  uint64_t max_dt_us_{80000};
  uint64_t range_sample_timeout_us_{100000};
  double range_min_valid_m_{0.08};
  double range_max_valid_m_{8.0};
  bool use_imu_delta_angle_{true};
  bool imu_is_flu_frame_{true};
  double imu_min_coverage_ratio_{0.7};

  struct ImuSample {
    uint64_t stamp_us{0};
    std::array<float, 3> gyro_frd{0.0f, 0.0f, 0.0f};
  };

  struct RangeSample {
    uint64_t stamp_us{0};
    float distance_m{std::numeric_limits<float>::quiet_NaN()};
  };

  // Previous frame state
  std::mutex mu_;
  cv::Mat prev_gray_;
  uint64_t prev_stamp_us_{0};
  cv::Mat hann_window_;
  std::deque<ImuSample> imu_samples_;
  RangeSample last_range_sample_{};

  static float Clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
  }

  uint64_t GzStampToUs(const gz::msgs::Image &msg) const {
    if (msg.has_header() && msg.header().has_stamp()) {
      return static_cast<uint64_t>(msg.header().stamp().sec()) * 1000000ULL +
             static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;
    }
    return 0;
  }

  uint64_t GzStampToUs(const gz::msgs::IMU &msg) const {
    if (msg.has_header() && msg.header().has_stamp()) {
      return static_cast<uint64_t>(msg.header().stamp().sec()) * 1000000ULL +
             static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;
    }
    return 0;
  }

  uint64_t GzStampToUs(const gz::msgs::LaserScan &msg) const {
    if (msg.has_header() && msg.header().has_stamp()) {
      return static_cast<uint64_t>(msg.header().stamp().sec()) * 1000000ULL +
             static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;
    }
    return 0;
  }

  std::array<float, 3> GyroToFrd(const gz::msgs::IMU &msg) const {
    const float wx = static_cast<float>(msg.angular_velocity().x());
    const float wy = static_cast<float>(msg.angular_velocity().y());
    const float wz = static_cast<float>(msg.angular_velocity().z());

    if (!imu_is_flu_frame_) {
      return {wx, wy, wz};
    }
    // Gazebo IMU is typically FLU; PX4 expects FRD.
    return {wx, -wy, -wz};
  }

  void OnGzRange(const gz::msgs::LaserScan &scan_msg) {
    const uint64_t stamp_us = GzStampToUs(scan_msg);
    if (stamp_us == 0) {
      return;
    }

    float distance_m = std::numeric_limits<float>::quiet_NaN();
    if (scan_msg.ranges_size() > 0) {
      const float candidate = static_cast<float>(scan_msg.ranges(0));
      if (std::isfinite(candidate) && candidate >= range_min_valid_m_ && candidate <= range_max_valid_m_) {
        distance_m = candidate;
      }
    }

    std::lock_guard<std::mutex> lock(mu_);
    if (last_range_sample_.stamp_us != 0 && stamp_us <= last_range_sample_.stamp_us) {
      return;
    }

    last_range_sample_.stamp_us = stamp_us;
    last_range_sample_.distance_m = distance_m;
  }

  void OnGzImu(const gz::msgs::IMU &imu_msg) {
    const uint64_t stamp_us = GzStampToUs(imu_msg);
    if (stamp_us == 0) {
      return;
    }

    const auto gyro_frd = GyroToFrd(imu_msg);
    if (!std::isfinite(gyro_frd[0]) || !std::isfinite(gyro_frd[1]) || !std::isfinite(gyro_frd[2])) {
      return;
    }

    std::lock_guard<std::mutex> lock(mu_);
    if (!imu_samples_.empty() && stamp_us <= imu_samples_.back().stamp_us) {
      return;
    }

    imu_samples_.push_back(ImuSample{stamp_us, gyro_frd});

    const uint64_t keep_from_us = (stamp_us > 2000000ULL) ? (stamp_us - 2000000ULL) : 0ULL;
    while (imu_samples_.size() > 600 || (!imu_samples_.empty() && imu_samples_.front().stamp_us < keep_from_us)) {
      imu_samples_.pop_front();
    }
  }

  bool TryGetFreshRangeSampleLocked(uint64_t stamp_us, float &distance_m) const {
    if (last_range_sample_.stamp_us == 0 || stamp_us < last_range_sample_.stamp_us) {
      return false;
    }

    if ((stamp_us - last_range_sample_.stamp_us) > range_sample_timeout_us_) {
      return false;
    }

    if (!std::isfinite(last_range_sample_.distance_m)) {
      return false;
    }

    distance_m = last_range_sample_.distance_m;
    return true;
  }

  bool IntegrateDeltaAngleLocked(
    uint64_t start_us,
    uint64_t end_us,
    std::array<float, 3> &delta_angle_rad,
    float &coverage_ratio) const
  {
    delta_angle_rad = {0.0f, 0.0f, 0.0f};
    coverage_ratio = 0.0f;

    if (end_us <= start_us || imu_samples_.empty()) {
      return false;
    }

    const uint64_t total_us = end_us - start_us;
    const uint64_t first_us = imu_samples_.front().stamp_us;
    const uint64_t last_us = imu_samples_.back().stamp_us;
    if (last_us <= start_us || first_us >= end_us) {
      return false;
    }

    const uint64_t integ_start_us = std::max(start_us, first_us);
    const uint64_t integ_end_us = std::min(end_us, last_us);
    if (integ_end_us <= integ_start_us) {
      return false;
    }

    std::array<float, 3> omega = imu_samples_.front().gyro_frd;
    for (const auto &sample : imu_samples_) {
      if (sample.stamp_us > integ_start_us) {
        break;
      }
      omega = sample.gyro_frd;
    }

    uint64_t cursor_us = integ_start_us;
    for (const auto &sample : imu_samples_) {
      if (sample.stamp_us <= integ_start_us) {
        continue;
      }
      if (sample.stamp_us >= integ_end_us) {
        break;
      }

      const uint64_t next_us = sample.stamp_us;
      const float dt_s = static_cast<float>(next_us - cursor_us) * 1.0e-6f;
      for (size_t axis = 0; axis < 3; ++axis) {
        delta_angle_rad[axis] += omega[axis] * dt_s;
      }
      cursor_us = next_us;
      omega = sample.gyro_frd;
    }

    if (integ_end_us > cursor_us) {
      const float dt_s = static_cast<float>(integ_end_us - cursor_us) * 1.0e-6f;
      for (size_t axis = 0; axis < 3; ++axis) {
        delta_angle_rad[axis] += omega[axis] * dt_s;
      }
    }

    coverage_ratio = static_cast<float>(integ_end_us - integ_start_us) /
                     static_cast<float>(total_us);
    return true;
  }

  void OnGzImage(const gz::msgs::Image &gz_msg) {
    // Convert to grayscale
    cv::Mat gray;
    const int h = static_cast<int>(gz_msg.height());
    const int w = static_cast<int>(gz_msg.width());

    if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
      cv::Mat rgb(h, w, CV_8UC3,
                  const_cast<char *>(gz_msg.data().data()));
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
      gray = cv::Mat(h, w, CV_8UC1,
                     const_cast<char *>(gz_msg.data().data())).clone();
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Unsupported pixel format: %d", static_cast<int>(gz_msg.pixel_format_type()));
      return;
    }

    const uint64_t stamp_us = GzStampToUs(gz_msg);
    if (stamp_us == 0 || gray.empty()) {
      return;
    }

    cv::Mat gray_proc;
    cv::GaussianBlur(
      gray, gray_proc, cv::Size(3, 3), 0.0, 0.0, cv::BORDER_REPLICATE);

    std::lock_guard<std::mutex> lock(mu_);

    if (prev_gray_.empty() || prev_stamp_us_ == 0 || prev_gray_.size() != gray_proc.size()) {
      prev_gray_ = gray_proc.clone();
      prev_stamp_us_ = stamp_us;
      return;
    }

    if (stamp_us <= prev_stamp_us_) {
      prev_gray_ = gray_proc.clone();
      prev_stamp_us_ = stamp_us;
      return;
    }

    const uint64_t dt_us = stamp_us - prev_stamp_us_;
    if (dt_us == 0 || dt_us > max_dt_us_) {
      prev_gray_ = gray_proc.clone();
      prev_stamp_us_ = stamp_us;
      return;
    }

    const double focal_length_px = (static_cast<double>(w) / 2.0) /
                                   std::tan(camera_hfov_ / 2.0);
    const float dt_s = std::max(1.0e-4f, static_cast<float>(dt_us) * 1.0e-6f);
    const float max_disp_px = static_cast<float>(max_flow_rate_ * focal_length_px * dt_s);

    const cv::Rect roi = BuildRoi(w, h);
    const cv::Mat prev_roi = prev_gray_(roi);
    const cv::Mat curr_roi = gray_proc(roi);

    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(
      prev_roi, prev_pts, max_features_, lk_quality_level_, 2.0, cv::Mat(), 3, false, 0.04);

    std::vector<float> dx_valid;
    std::vector<float> dy_valid;
    const int track_candidates = static_cast<int>(prev_pts.size());
    int tracked_valid = 0;

    if (!prev_pts.empty()) {
      std::vector<cv::Point2f> curr_pts;
      std::vector<uint8_t> status;
      std::vector<float> err;

      cv::calcOpticalFlowPyrLK(
        prev_roi, curr_roi, prev_pts, curr_pts, status, err,
        cv::Size(15, 15), 2,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));

      dx_valid.reserve(prev_pts.size());
      dy_valid.reserve(prev_pts.size());

      for (size_t i = 0; i < prev_pts.size(); ++i) {
        if (i >= curr_pts.size() || i >= status.size() || i >= err.size()) {
          continue;
        }
        if (!status[i] || !std::isfinite(err[i]) || err[i] > lk_max_error_) {
          continue;
        }

        const float dx = curr_pts[i].x - prev_pts[i].x;
        const float dy = curr_pts[i].y - prev_pts[i].y;
        if (!std::isfinite(dx) || !std::isfinite(dy)) {
          continue;
        }

        const float disp = std::hypot(dx, dy);
        if (disp > (max_disp_px * 1.5f + 1.0f)) {
          continue;
        }

        dx_valid.push_back(dx);
        dy_valid.push_back(dy);
      }
      tracked_valid = static_cast<int>(dx_valid.size());
    }

    float flow_dx_px = 0.0f;
    float flow_dy_px = 0.0f;
    int inlier_count = 0;
    int quality = 0;
    bool flow_valid = false;

    if (tracked_valid >= min_features_) {
      std::vector<float> dx_tmp = dx_valid;
      std::vector<float> dy_tmp = dy_valid;
      float med_dx = Median(dx_tmp);
      float med_dy = Median(dy_tmp);

      std::vector<float> residuals;
      residuals.reserve(dx_valid.size());
      for (size_t i = 0; i < dx_valid.size(); ++i) {
        residuals.push_back(std::hypot(dx_valid[i] - med_dx, dy_valid[i] - med_dy));
      }
      const float mad = Median(residuals);
      const float inlier_thresh = std::max(0.5f, 3.0f * mad);

      std::vector<float> dx_inliers;
      std::vector<float> dy_inliers;
      dx_inliers.reserve(dx_valid.size());
      dy_inliers.reserve(dx_valid.size());
      for (size_t i = 0; i < dx_valid.size(); ++i) {
        const float r = std::hypot(dx_valid[i] - med_dx, dy_valid[i] - med_dy);
        if (r <= inlier_thresh) {
          dx_inliers.push_back(dx_valid[i]);
          dy_inliers.push_back(dy_valid[i]);
        }
      }

      if (!dx_inliers.empty()) {
        flow_dx_px = Median(dx_inliers);
        flow_dy_px = Median(dy_inliers);
      } else {
        flow_dx_px = med_dx;
        flow_dy_px = med_dy;
      }

      inlier_count = static_cast<int>(dx_inliers.size());

      const float tracked_ratio = static_cast<float>(tracked_valid) /
                                  static_cast<float>(std::max(1, track_candidates));
      const float inlier_ratio = static_cast<float>(inlier_count) /
                                 static_cast<float>(std::max(1, tracked_valid));
      const float consistency = std::exp(-0.6f * std::max(0.0f, mad));
      const float score = Clamp01(0.55f * inlier_ratio + 0.30f * tracked_ratio + 0.15f * consistency);
      quality = static_cast<int>(std::round(255.0f * score));
      quality = std::clamp(quality, 0, 255);
      flow_valid = quality >= quality_threshold_;
    }

    // Low-texture fallback: phase correlation estimates global image translation.
    if (!flow_valid) {
      cv::Mat prev_f;
      cv::Mat curr_f;
      prev_roi.convertTo(prev_f, CV_32F);
      curr_roi.convertTo(curr_f, CV_32F);

      const float prev_mean = static_cast<float>(cv::mean(prev_f)[0]);
      const float curr_mean = static_cast<float>(cv::mean(curr_f)[0]);
      prev_f -= prev_mean;
      curr_f -= curr_mean;

      if (hann_window_.empty() || hann_window_.size() != prev_f.size()) {
        cv::createHanningWindow(hann_window_, prev_f.size(), CV_32F);
      }

      double phase_response = 0.0;
      const cv::Point2d shift = cv::phaseCorrelate(prev_f, curr_f, hann_window_, &phase_response);
      const float disp = std::hypot(static_cast<float>(shift.x), static_cast<float>(shift.y));

      if (std::isfinite(shift.x) &&
          std::isfinite(shift.y) &&
          phase_response >= phase_corr_min_response_ &&
          disp <= (max_disp_px * 1.5f + 1.0f))
      {
        flow_dx_px = static_cast<float>(shift.x);
        flow_dy_px = static_cast<float>(shift.y);

        cv::Scalar mean_luma;
        cv::Scalar stddev_luma;
        cv::meanStdDev(curr_roi, mean_luma, stddev_luma);
        const float texture_score = Clamp01(static_cast<float>(stddev_luma[0]) / 20.0f);
        const float response_score = Clamp01(static_cast<float>(phase_response) * 1.5f);
        quality = static_cast<int>(std::round(255.0f * (0.75f * response_score + 0.25f * texture_score)));
        quality = std::clamp(quality, 0, 255);
        flow_valid = quality >= quality_threshold_;
      }
    }

    if (!flow_valid) {
      flow_dx_px = 0.0f;
      flow_dy_px = 0.0f;
      quality = 0;
    } else if (std::hypot(flow_dx_px, flow_dy_px) < static_cast<float>(flow_deadband_px_)) {
      // Keep quality so EKF can still trust near-zero flow in hover/low-speed cases.
      flow_dx_px = 0.0f;
      flow_dy_px = 0.0f;
    }

    // Convert pixel displacement to integrated angular flow (radians).
    const double flow_x_rad = std::atan2(static_cast<double>(flow_dx_px), focal_length_px);
    const double flow_y_rad = std::atan2(static_cast<double>(flow_dy_px), focal_length_px);

    std::array<float, 3> delta_angle{std::nanf(""), std::nanf(""), std::nanf("")};
    bool delta_angle_available = false;
    if (use_imu_delta_angle_) {
      std::array<float, 3> delta_integrated{};
      float coverage = 0.0f;
      if (IntegrateDeltaAngleLocked(prev_stamp_us_, stamp_us, delta_integrated, coverage) &&
          coverage >= static_cast<float>(imu_min_coverage_ratio_))
      {
        delta_angle = delta_integrated;
        delta_angle_available = true;
      }
    }

    // Publish
    px4_msgs::msg::SensorOpticalFlow out;
    out.timestamp = stamp_us;
    out.timestamp_sample = stamp_us;
    out.pixel_flow[0] = static_cast<float>(flow_x_rad);
    out.pixel_flow[1] = static_cast<float>(flow_y_rad);
    out.delta_angle[0] = delta_angle[0];
    out.delta_angle[1] = delta_angle[1];
    out.delta_angle[2] = delta_angle[2];
    out.delta_angle_available = delta_angle_available;

    float distance_m = 0.0f;
    if (TryGetFreshRangeSampleLocked(stamp_us, distance_m)) {
      out.distance_m = distance_m;
      out.distance_available = true;
    } else {
      out.distance_m = 0.0f;
      out.distance_available = false;
    }

    out.integration_timespan_us = static_cast<uint32_t>(dt_us);
    out.quality = static_cast<uint8_t>(quality);
    out.error_count = static_cast<uint32_t>(std::max(0, track_candidates - inlier_count));
    out.max_flow_rate = static_cast<float>(max_flow_rate_);
    out.min_ground_distance = static_cast<float>(min_ground_dist_);
    out.max_ground_distance = static_cast<float>(max_ground_dist_);
    out.mode = px4_msgs::msg::SensorOpticalFlow::MODE_BRIGHT;

    // Device ID: simulation bus
    out.device_id = (4U << 28) | (2U << 16) | (1U << 8) | 0x51U;

    pub_->publish(out);

    // Store for next iteration
    prev_gray_ = gray_proc.clone();
    prev_stamp_us_ = stamp_us;
  }
};

}  // namespace uav_bridge

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::OpticalFlowBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
