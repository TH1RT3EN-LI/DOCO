#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <uav_visual_landing/msg/landing_error.hpp>
#include <uav_visual_landing/msg/landing_status.hpp>

namespace uav_visual_landing
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode() : Node("aruco_detector_node")
  {
    this->declare_parameter<std::string>("image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/uav/camera/camera_info");
    this->declare_parameter<std::string>("debug_image_topic", "/uav/visual_landing/debug_image");
    this->declare_parameter<std::string>("pixel_error_topic", "/uav/visual_landing/pixel_error");
    this->declare_parameter<std::string>("detected_topic", "/uav/visual_landing/target_detected");
    this->declare_parameter<std::string>("landing_error_topic", "/uav/visual_landing/landing_error");
    this->declare_parameter<std::string>("status_topic", "/uav/visual_landing/status");
    this->declare_parameter<std::string>("velocity_topic", "/uav/control/velocity");
    this->declare_parameter<std::string>("height_distance_topic", "/uav/fmu/in/distance_sensor");
    this->declare_parameter<std::string>("optical_flow_topic", "/uav/fmu/in/sensor_optical_flow");
    this->declare_parameter<int>("target_marker_id", -1);
    this->declare_parameter<double>("camera_fx", 320.0);
    this->declare_parameter<double>("camera_fy", 320.0);
    this->declare_parameter<double>("camera_cx", 320.0);
    this->declare_parameter<double>("camera_cy", 240.0);
    this->declare_parameter<bool>("always_publish_debug", true);
    this->declare_parameter<double>("debug_publish_rate_hz", 0.0);
    this->declare_parameter<int>("detect_max_dimension", 640);
    this->declare_parameter<bool>("roi_search_enabled", true);
    this->declare_parameter<double>("roi_max_age_s", 0.35);
    this->declare_parameter<double>("roi_margin_scale", 2.5);
    this->declare_parameter<int>("roi_min_size_px", 96);
    this->declare_parameter<double>("marker_size_m", 0.20);
    this->declare_parameter<double>("confidence_reproj_scale_px", 4.0);
    this->declare_parameter<double>("confidence_min_marker_span_px", 18.0);
    this->declare_parameter<double>("confidence_full_marker_span_px", 72.0);
    this->declare_parameter<double>("debug_render_scale", 6.0);
    this->declare_parameter<int>("debug_panel_width_px", 420);

    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
    pixel_error_topic_ = this->get_parameter("pixel_error_topic").as_string();
    detected_topic_ = this->get_parameter("detected_topic").as_string();
    landing_error_topic_ = this->get_parameter("landing_error_topic").as_string();
    status_topic_ = this->get_parameter("status_topic").as_string();
    velocity_topic_ = this->get_parameter("velocity_topic").as_string();
    height_distance_topic_ = this->get_parameter("height_distance_topic").as_string();
    optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
    target_marker_id_ = this->get_parameter("target_marker_id").as_int();
    always_publish_debug_ = this->get_parameter("always_publish_debug").as_bool();
    const double debug_publish_rate_hz = this->get_parameter("debug_publish_rate_hz").as_double();
    debug_publish_period_s_ = debug_publish_rate_hz > 1.0e-6 ? (1.0 / debug_publish_rate_hz) : 0.0;
    detect_max_dimension_ = this->get_parameter("detect_max_dimension").as_int();
    roi_search_enabled_ = this->get_parameter("roi_search_enabled").as_bool();
    roi_max_age_s_ = this->get_parameter("roi_max_age_s").as_double();
    roi_margin_scale_ = static_cast<float>(this->get_parameter("roi_margin_scale").as_double());
    roi_min_size_px_ = this->get_parameter("roi_min_size_px").as_int();

    fx_ = static_cast<float>(this->get_parameter("camera_fx").as_double());
    fy_ = static_cast<float>(this->get_parameter("camera_fy").as_double());
    cx_ = static_cast<float>(this->get_parameter("camera_cx").as_double());
    cy_ = static_cast<float>(this->get_parameter("camera_cy").as_double());
    marker_size_m_ = static_cast<float>(this->get_parameter("marker_size_m").as_double());
    confidence_reproj_scale_px_ = static_cast<float>(
      this->get_parameter("confidence_reproj_scale_px").as_double());
    confidence_min_marker_span_px_ = static_cast<float>(
      this->get_parameter("confidence_min_marker_span_px").as_double());
    confidence_full_marker_span_px_ = static_cast<float>(
      this->get_parameter("confidence_full_marker_span_px").as_double());
    debug_render_scale_ = this->get_parameter("debug_render_scale").as_double();
    debug_panel_width_px_ = this->get_parameter("debug_panel_width_px").as_int();

    updateFallbackCameraModel();

    tag_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    tag_params_ = cv::aruco::DetectorParameters::create();
    tag_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    debug_image_pub_ = image_transport::create_publisher(
      this, debug_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    pixel_error_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      pixel_error_topic_, 10);
    detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      detected_topic_, 10);
    landing_error_pub_ = this->create_publisher<uav_visual_landing::msg::LandingError>(
      landing_error_topic_, 10);

    landing_status_sub_ = this->create_subscription<uav_visual_landing::msg::LandingStatus>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const uav_visual_landing::msg::LandingStatus::SharedPtr msg)
      {
        landing_status_.received = true;
        landing_status_.stamp =
          ((msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0)) ?
          rclcpp::Time(msg->header.stamp) : this->now();
        landing_status_.active = msg->active;
        landing_status_.tag_detected = msg->tag_detected;
        landing_status_.state = msg->state;
        landing_status_.current_height_m = msg->current_height_m;
        landing_status_.lost_duration_s = msg->lost_duration_s;
        landing_status_.err_u_norm_filtered = msg->err_u_norm_filtered;
        landing_status_.err_v_norm_filtered = msg->err_v_norm_filtered;
        landing_status_.yaw_err_rad_filtered = msg->yaw_err_rad_filtered;
        landing_status_.detection_confidence = msg->detection_confidence;
        landing_status_.range_available = msg->range_available;
        landing_status_.range_fresh = msg->range_fresh;
        landing_status_.range_in_bounds = msg->range_in_bounds;
        landing_status_.range_consistent = msg->range_consistent;
        landing_status_.height_using_range = msg->height_using_range;
        landing_status_.range_height_m = msg->range_height_m;
        landing_status_.range_age_s = msg->range_age_s;
        landing_status_.range_signal_quality = msg->range_signal_quality;
      });

    velocity_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      velocity_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        velocity_cmd_.received = true;
        velocity_cmd_.stamp = this->now();
        velocity_cmd_.vx = static_cast<float>(msg->linear.x);
        velocity_cmd_.vy = static_cast<float>(msg->linear.y);
        velocity_cmd_.vz = static_cast<float>(msg->linear.z);
        velocity_cmd_.yaw_rate = static_cast<float>(msg->angular.z);
      });

    range_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      height_distance_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
      {
        range_telemetry_.received = true;
        range_telemetry_.stamp = this->now();
        range_telemetry_.current_distance_m = msg->current_distance;
        range_telemetry_.signal_quality = msg->signal_quality;
      });

    optical_flow_sub_ = this->create_subscription<px4_msgs::msg::SensorOpticalFlow>(
      optical_flow_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::SensorOpticalFlow::SharedPtr msg)
      {
        flow_telemetry_.received = true;
        flow_telemetry_.stamp = this->now();
        flow_telemetry_.quality = msg->quality;
        flow_telemetry_.min_ground_distance = msg->min_ground_distance;
        flow_telemetry_.max_ground_distance = msg->max_ground_distance;
      });

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
      {
        onCameraInfo(msg);
      });

    image_sub_ = image_transport::create_subscription(
      this, image_topic_,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg)
      {
        onImage(msg);
      },
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] image=%s dict=DICT_APRILTAG_16h5 target_id=%d marker_size=%.3fm",
      image_topic_.c_str(), target_marker_id_, marker_size_m_);
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] outputs: landing_error=%s pixel_error=%s detected=%s debug=%s",
      landing_error_topic_.c_str(), pixel_error_topic_.c_str(), detected_topic_.c_str(),
      debug_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] debug overlay: status=%s cmd=%s range=%s flow=%s scale=%.1f panel=%d",
      status_topic_.c_str(), velocity_topic_.c_str(), height_distance_topic_.c_str(),
      optical_flow_topic_.c_str(), debug_render_scale_, debug_panel_width_px_);
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] fallback intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      fx_, fy_, cx_, cy_);
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] debug publish rate limit: %.1f Hz (0 = unlimited)",
      debug_publish_rate_hz);
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] detect max dimension: %d px (0 = full resolution)",
      detect_max_dimension_);
    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] roi search: %s age<=%.2fs margin=%.2f min=%dpx",
      roi_search_enabled_ ? "on" : "off", roi_max_age_s_, roi_margin_scale_, roi_min_size_px_);
  }

private:
  struct PoseCandidate
  {
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};
    float yaw_rad{0.0f};
    float reproj_err_px{std::numeric_limits<float>::infinity()};
  };

  struct DetectionEstimate
  {
    bool detected{false};
    bool pose_valid{false};
    int matched_id{-1};
    cv::Point2f center_px{0.0f, 0.0f};
    float pixel_err_u{0.0f};
    float pixel_err_v{0.0f};
    float pixel_err_norm{0.0f};
    float err_u_norm{0.0f};
    float err_v_norm{0.0f};
    float yaw_primary_rad{0.0f};
    float yaw_alt_rad{0.0f};
    float reproj_primary_px{0.0f};
    float reproj_alt_px{0.0f};
    uint8_t solution_count{0};
    float confidence{0.0f};
    float marker_span_px{0.0f};
  };

  struct LandingStatusTelemetry
  {
    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool active{false};
    bool tag_detected{false};
    std::string state{"-"};
    float current_height_m{0.0f};
    float lost_duration_s{0.0f};
    float err_u_norm_filtered{0.0f};
    float err_v_norm_filtered{0.0f};
    float yaw_err_rad_filtered{0.0f};
    float detection_confidence{0.0f};
    bool range_available{false};
    bool range_fresh{false};
    bool range_in_bounds{false};
    bool range_consistent{false};
    bool height_using_range{false};
    float range_height_m{0.0f};
    float range_age_s{-1.0f};
    int8_t range_signal_quality{-1};
  };

  struct VelocityCommandTelemetry
  {
    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
    float yaw_rate{0.0f};
  };

  struct RangeTelemetry
  {
    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    float current_distance_m{0.0f};
    int8_t signal_quality{-1};
  };

  struct FlowTelemetry
  {
    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint8_t quality{0U};
    float min_ground_distance{0.0f};
    float max_ground_distance{0.0f};
  };

  static float clamp01(float value)
  {
    if (value < 0.0f) {
      return 0.0f;
    }
    if (value > 1.0f) {
      return 1.0f;
    }
    return value;
  }

  static const char * boolFlag(bool value)
  {
    return value ? "Y" : "N";
  }

  static float normalizeAngle(float angle)
  {
    while (angle > static_cast<float>(M_PI)) {
      angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
      angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
  }

  static cv::Vec3d matToVec3d(const cv::Mat & mat)
  {
    cv::Mat reshaped = mat.reshape(1, 3);
    cv::Mat numeric;
    reshaped.convertTo(numeric, CV_64F);
    return {
      numeric.at<double>(0, 0),
      numeric.at<double>(1, 0),
      numeric.at<double>(2, 0)};
  }

  static void offsetCorners(std::vector<std::vector<cv::Point2f>> & corners, const cv::Point2f & offset)
  {
    if (std::abs(offset.x) < 1.0e-6f && std::abs(offset.y) < 1.0e-6f) {
      return;
    }

    for (auto & marker_corners : corners) {
      for (auto & point : marker_corners) {
        point.x += offset.x;
        point.y += offset.y;
      }
    }
  }

  static void scaleCorners(std::vector<std::vector<cv::Point2f>> & corners, float scale)
  {
    if (std::abs(scale - 1.0f) < 1.0e-6f) {
      return;
    }

    for (auto & marker_corners : corners) {
      for (auto & point : marker_corners) {
        point.x *= scale;
        point.y *= scale;
      }
    }
  }

  void updateFallbackCameraModel()
  {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = static_cast<double>(fx_);
    camera_matrix_.at<double>(1, 1) = static_cast<double>(fy_);
    camera_matrix_.at<double>(0, 2) = static_cast<double>(cx_);
    camera_matrix_.at<double>(1, 2) = static_cast<double>(cy_);
    dist_coeffs_.assign(5, 0.0);
  }

  rclcpp::Time resolveImageStamp(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
  {
    const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
    return has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
  }

  int findTargetIndex(const std::vector<int> & ids) const
  {
    for (size_t i = 0; i < ids.size(); ++i) {
      if (target_marker_id_ < 0 || ids[i] == target_marker_id_) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  cv::Rect computeRoiRect(const cv::Size & image_size) const
  {
    const int min_side = std::max(roi_min_size_px_, 8);
    const float half_span = std::max(
      0.5f * static_cast<float>(min_side), roi_margin_scale_ * last_target_radius_detect_);

    const int left = std::max(
      0, static_cast<int>(std::floor(last_target_center_detect_.x - half_span)));
    const int top = std::max(
      0, static_cast<int>(std::floor(last_target_center_detect_.y - half_span)));
    const int right = std::min(
      image_size.width, static_cast<int>(std::ceil(last_target_center_detect_.x + half_span)));
    const int bottom = std::min(
      image_size.height, static_cast<int>(std::ceil(last_target_center_detect_.y + half_span)));

    return cv::Rect(left, top, std::max(0, right - left), std::max(0, bottom - top));
  }

  void updateTrackingState(const std::vector<cv::Point2f> & detect_corners, const rclcpp::Time & stamp)
  {
    if (detect_corners.size() != 4) {
      return;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto & point : detect_corners) {
      center += point;
    }
    center *= 0.25f;

    float max_radius = 0.0f;
    for (const auto & point : detect_corners) {
      max_radius = std::max(max_radius, std::hypot(point.x - center.x, point.y - center.y));
    }

    last_target_center_detect_ = center;
    last_target_radius_detect_ = std::max(max_radius, 8.0f);
    last_target_stamp_ = stamp;
    has_last_target_detect_ = true;
  }

  cv::Mat toGray(
    const cv::Mat & frame, const std::string & encoding,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
  {
    namespace enc = sensor_msgs::image_encodings;

    cv::Mat gray;
    if (encoding == enc::MONO8) {
      return frame;
    }
    if (encoding == enc::BGR8) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      return gray;
    }
    if (encoding == enc::RGB8) {
      cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
      return gray;
    }
    if (encoding == enc::BGRA8) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
      return gray;
    }
    if (encoding == enc::RGBA8) {
      cv::cvtColor(frame, gray, cv::COLOR_RGBA2GRAY);
      return gray;
    }

    return cv_bridge::toCvCopy(msg, enc::MONO8)->image;
  }

  bool shouldPublishDebug(const rclcpp::Time & stamp, bool detected)
  {
    if (!(always_publish_debug_ || detected)) {
      return false;
    }

    if (debug_image_pub_.getNumSubscribers() == 0) {
      return false;
    }

    if (debug_publish_period_s_ <= 1.0e-6) {
      return true;
    }

    if (!debug_publish_initialized_) {
      last_debug_publish_time_ = stamp;
      debug_publish_initialized_ = true;
      return true;
    }

    if ((stamp - last_debug_publish_time_).seconds() + 1.0e-9 < debug_publish_period_s_) {
      return false;
    }

    last_debug_publish_time_ = stamp;
    return true;
  }

  std::vector<cv::Point3f> buildMarkerObjectPoints() const
  {
    const float half = 0.5f * marker_size_m_;
    return {
      {-half, half, 0.0f},
      {half, half, 0.0f},
      {half, -half, 0.0f},
      {-half, -half, 0.0f}};
  }

  static float computeMarkerSpanPx(const std::vector<cv::Point2f> & corners)
  {
    if (corners.size() != 4) {
      return 0.0f;
    }

    float perimeter = 0.0f;
    for (size_t i = 0; i < corners.size(); ++i) {
      const auto & a = corners[i];
      const auto & b = corners[(i + 1U) % corners.size()];
      perimeter += std::hypot(b.x - a.x, b.y - a.y);
    }
    return 0.25f * perimeter;
  }

  float computeReprojectionErrorPx(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec) const
  {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

    if (projected_points.size() != image_points.size() || projected_points.empty()) {
      return std::numeric_limits<float>::infinity();
    }

    double sum_sq = 0.0;
    for (size_t i = 0; i < projected_points.size(); ++i) {
      const double dx = static_cast<double>(projected_points[i].x) - image_points[i].x;
      const double dy = static_cast<double>(projected_points[i].y) - image_points[i].y;
      sum_sq += (dx * dx) + (dy * dy);
    }

    const double mean_sq = sum_sq / static_cast<double>(projected_points.size());
    return static_cast<float>(std::sqrt(mean_sq));
  }

  static float computePoseYawRad(const cv::Vec3d & rvec)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    return normalizeAngle(static_cast<float>(std::atan2(
      rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0))));
  }

  float computeDetectionConfidence(bool pose_valid, float reproj_err_px, float marker_span_px) const
  {
    const float span_den = std::max(
      confidence_full_marker_span_px_ - confidence_min_marker_span_px_, 1.0f);
    const float size_conf = clamp01((marker_span_px - confidence_min_marker_span_px_) / span_den);

    if (!pose_valid || !std::isfinite(reproj_err_px)) {
      return 0.35f * size_conf;
    }

    const float reproj_scale = std::max(confidence_reproj_scale_px_, 1.0e-3f);
    const float reproj_conf = std::exp(-reproj_err_px / reproj_scale);
    return clamp01((0.70f * reproj_conf) + (0.30f * size_conf));
  }

  bool estimatePoseCandidates(
    const std::vector<cv::Point2f> & corners,
    std::vector<PoseCandidate> & candidates)
  {
    candidates.clear();

    if (corners.size() != 4 || marker_size_m_ <= 1.0e-6f || fx_ <= 1.0e-6f || fy_ <= 1.0e-6f) {
      return false;
    }

    const std::vector<cv::Point3f> object_points = buildMarkerObjectPoints();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    try {
      cv::solvePnPGeneric(
        object_points, corners, camera_matrix_, dist_coeffs_, rvecs, tvecs,
        false, cv::SOLVEPNP_IPPE_SQUARE);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[apriltag_detector] solvePnPGeneric failed: %s", e.what());
    }

    if (rvecs.empty() || tvecs.empty()) {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      try {
        const bool ok = cv::solvePnP(
          object_points, corners, camera_matrix_, dist_coeffs_, rvec, tvec,
          false, cv::SOLVEPNP_ITERATIVE);
        if (!ok) {
          return false;
        }
      } catch (const cv::Exception & e) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[apriltag_detector] solvePnP fallback failed: %s", e.what());
        return false;
      }

      PoseCandidate candidate;
      candidate.rvec = rvec;
      candidate.tvec = tvec;
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(
        object_points, corners, candidate.rvec, candidate.tvec);
      candidates.push_back(candidate);
      return true;
    }

    const size_t solution_count = std::min(rvecs.size(), tvecs.size());
    for (size_t i = 0; i < solution_count; ++i) {
      PoseCandidate candidate;
      candidate.rvec = matToVec3d(rvecs[i]);
      candidate.tvec = matToVec3d(tvecs[i]);
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(
        object_points, corners, candidate.rvec, candidate.tvec);
      candidates.push_back(candidate);
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const PoseCandidate & lhs, const PoseCandidate & rhs)
      {
        return lhs.reproj_err_px < rhs.reproj_err_px;
      });
    return !candidates.empty();
  }

  std_msgs::msg::Header resolveOutputHeader(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const rclcpp::Time & image_stamp) const
  {
    auto header = msg->header;
    if ((header.stamp.sec == 0) && (header.stamp.nanosec == 0)) {
      header.stamp = image_stamp;
    }
    return header;
  }

  void publishLandingError(
    const std_msgs::msg::Header & header,
    const DetectionEstimate & estimate)
  {
    uav_visual_landing::msg::LandingError landing_error_msg;
    landing_error_msg.header = header;
    landing_error_msg.detected = estimate.detected;
    landing_error_msg.err_u_norm = estimate.err_u_norm;
    landing_error_msg.err_v_norm = estimate.err_v_norm;
    landing_error_msg.yaw_err_rad = estimate.yaw_primary_rad;
    landing_error_msg.yaw_alt_rad = estimate.yaw_alt_rad;
    landing_error_msg.reproj_err_px = estimate.reproj_primary_px;
    landing_error_msg.reproj_err_alt_px = estimate.reproj_alt_px;
    landing_error_msg.solution_count = estimate.solution_count;
    landing_error_msg.confidence = estimate.confidence;
    landing_error_pub_->publish(landing_error_msg);
  }

  double sampleAgeSec(const rclcpp::Time & stamp, bool received) const
  {
    if (!received) {
      return -1.0;
    }
    return std::max(0.0, (this->now() - stamp).seconds());
  }

  static void drawPanelText(cv::Mat & image, int x, int y, const std::string & text, double scale)
  {
    cv::putText(
      image, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
      scale, cv::Scalar(235), 1, cv::LINE_AA);
  }

  cv::Mat composeDebugDisplay(const cv::Mat & annotated_image, const DetectionEstimate & estimate) const
  {
    cv::Mat zoomed_image;
    const double scale = std::max(1.0, debug_render_scale_);
    if (scale > 1.0 + 1.0e-6) {
      cv::resize(
        annotated_image, zoomed_image, cv::Size(), scale, scale,
        scale >= 3.0 ? cv::INTER_NEAREST : cv::INTER_LINEAR);
    } else {
      zoomed_image = annotated_image.clone();
    }

    std::vector<std::string> lines;
    lines.reserve(10);

    char buffer[256];
    const double status_age_s = sampleAgeSec(landing_status_.stamp, landing_status_.received);
    const double cmd_age_s = sampleAgeSec(velocity_cmd_.stamp, velocity_cmd_.received);
    const double range_age_s = sampleAgeSec(range_telemetry_.stamp, range_telemetry_.received);
    const double flow_age_s = sampleAgeSec(flow_telemetry_.stamp, flow_telemetry_.received);

    std::snprintf(
      buffer, sizeof(buffer),
      "TAG: %s id=%d conf=%.2f rep=%.2f span=%.1fpx",
      estimate.detected ? "DETECTED" : "MISSING",
      estimate.matched_id, estimate.confidence,
      estimate.reproj_primary_px, estimate.marker_span_px);
    lines.emplace_back(buffer);

    std::snprintf(
      buffer, sizeof(buffer),
      "ERR: px=(%.1f, %.1f) n=(%.3f, %.3f) yaw=%.2f",
      estimate.pixel_err_u, estimate.pixel_err_v,
      estimate.err_u_norm, estimate.err_v_norm, estimate.yaw_primary_rad);
    lines.emplace_back(buffer);

    if (landing_status_.received) {
      std::snprintf(
        buffer, sizeof(buffer),
        "VL : active=%s stage=%s tag=%s age=%.2fs",
        boolFlag(landing_status_.active), landing_status_.state.c_str(),
        boolFlag(landing_status_.tag_detected), status_age_s);
      lines.emplace_back(buffer);

      std::snprintf(
        buffer, sizeof(buffer),
        "CTRL: ef=(%.3f, %.3f) yaw_f=%.2f lost=%.2fs",
        landing_status_.err_u_norm_filtered, landing_status_.err_v_norm_filtered,
        landing_status_.yaw_err_rad_filtered, landing_status_.lost_duration_s);
      lines.emplace_back(buffer);

      std::snprintf(
        buffer, sizeof(buffer),
        "HGT: est=%.2fm src=%s range=%.2fm q=%d",
        landing_status_.current_height_m,
        landing_status_.height_using_range ? "range" : "odom",
        landing_status_.range_height_m,
        static_cast<int>(landing_status_.range_signal_quality));
      lines.emplace_back(buffer);

      std::snprintf(
        buffer, sizeof(buffer),
        "RNG: avail=%s fresh=%s in=%s cons=%s age=%.2fs",
        boolFlag(landing_status_.range_available),
        boolFlag(landing_status_.range_fresh),
        boolFlag(landing_status_.range_in_bounds),
        boolFlag(landing_status_.range_consistent),
        landing_status_.range_age_s);
      lines.emplace_back(buffer);
    } else {
      lines.emplace_back("VL : status unavailable");
    }

    if (velocity_cmd_.received) {
      std::snprintf(
        buffer, sizeof(buffer),
        "CMD: vx=%.2f vy=%.2f vz=%.2f wz=%.2f age=%.2fs",
        velocity_cmd_.vx, velocity_cmd_.vy, velocity_cmd_.vz,
        velocity_cmd_.yaw_rate, cmd_age_s);
      lines.emplace_back(buffer);
    } else {
      lines.emplace_back("CMD: unavailable");
    }

    if (range_telemetry_.received) {
      std::snprintf(
        buffer, sizeof(buffer),
        "RAW: range=%.2fm q=%d age=%.2fs",
        range_telemetry_.current_distance_m,
        static_cast<int>(range_telemetry_.signal_quality), range_age_s);
      lines.emplace_back(buffer);
    }

    if (flow_telemetry_.received) {
      std::snprintf(
        buffer, sizeof(buffer),
        "FLOW: q=%u min=%.2f max=%.2f age=%.2fs",
        static_cast<unsigned>(flow_telemetry_.quality),
        flow_telemetry_.min_ground_distance,
        flow_telemetry_.max_ground_distance, flow_age_s);
      lines.emplace_back(buffer);
    } else {
      lines.emplace_back("FLOW: unavailable");
    }

    const int panel_width = std::max(debug_panel_width_px_, 260);
    const int top_margin = 26;
    const int line_height = 22;
    const int panel_height = std::max(
      zoomed_image.rows, top_margin + static_cast<int>(lines.size()) * line_height + 14);
    cv::Mat canvas(
      panel_height, zoomed_image.cols + panel_width,
      CV_8UC1, cv::Scalar(16));
    zoomed_image.copyTo(canvas(cv::Rect(0, 0, zoomed_image.cols, zoomed_image.rows)));

    const int panel_x = zoomed_image.cols;
    cv::rectangle(
      canvas, cv::Rect(panel_x, 0, panel_width, panel_height),
      cv::Scalar(28), cv::FILLED);
    cv::line(
      canvas, cv::Point(panel_x, 0), cv::Point(panel_x, panel_height - 1),
      cv::Scalar(100), 1);

    drawPanelText(canvas, panel_x + 10, 16, "VISUAL LANDING DEBUG", 0.55);
    for (size_t index = 0; index < lines.size(); ++index) {
      drawPanelText(
        canvas, panel_x + 10,
        top_margin + static_cast<int>(index) * line_height,
        lines[index], 0.50);
    }

    return canvas;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;
    }

    fx_ = static_cast<float>(msg->k[0]);
    fy_ = static_cast<float>(msg->k[4]);
    cx_ = static_cast<float>(msg->k[2]);
    cy_ = static_cast<float>(msg->k[5]);
    camera_info_received_ = true;

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix_.at<double>(row, col) = msg->k[row * 3 + col];
      }
    }

    dist_coeffs_.clear();
    dist_coeffs_.reserve(msg->d.size());
    for (const double coeff : msg->d) {
      dist_coeffs_.push_back(coeff);
    }
    if (dist_coeffs_.empty()) {
      dist_coeffs_.assign(5, 0.0);
    }

    RCLCPP_INFO(this->get_logger(),
      "[apriltag_detector] CameraInfo received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f dist=%zu",
      fx_, fy_, cx_, cy_, dist_coeffs_.size());
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & frame = cv_ptr->image;
    if (frame.empty()) {
      return;
    }

    const rclcpp::Time image_stamp = resolveImageStamp(msg);
    const std_msgs::msg::Header output_header = resolveOutputHeader(msg, image_stamp);

    const std::string & encoding = cv_ptr->encoding;
    cv::Mat gray;
    try {
      gray = toGray(frame, encoding, msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "gray conversion failed for encoding '%s': %s",
        encoding.c_str(), e.what());
      return;
    }

    cv::Mat detect_img = gray;
    float detect_scale = 1.0f;
    const int frame_max_dim = std::max(gray.cols, gray.rows);
    if (detect_max_dimension_ > 0 && frame_max_dim > detect_max_dimension_) {
      detect_scale = static_cast<float>(detect_max_dimension_) /
        static_cast<float>(frame_max_dim);
      const int detect_width = std::max(1, static_cast<int>(std::lround(gray.cols * detect_scale)));
      const int detect_height = std::max(1, static_cast<int>(std::lround(gray.rows * detect_scale)));
      cv::resize(gray, detect_img, cv::Size(detect_width, detect_height), 0.0, 0.0, cv::INTER_AREA);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    bool used_roi_detection = false;
    if (roi_search_enabled_ && has_last_target_detect_ &&
      (image_stamp - last_target_stamp_).seconds() <= roi_max_age_s_)
    {
      const cv::Rect roi_rect = computeRoiRect(detect_img.size());
      if (roi_rect.width > 0 && roi_rect.height > 0 &&
        (roi_rect.width < detect_img.cols || roi_rect.height < detect_img.rows))
      {
        std::vector<int> roi_ids;
        std::vector<std::vector<cv::Point2f>> roi_corners;
        cv::aruco::detectMarkers(detect_img(roi_rect), tag_dict_, roi_corners, roi_ids, tag_params_);
        offsetCorners(
          roi_corners, cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y)));
        if (findTargetIndex(roi_ids) >= 0) {
          ids = std::move(roi_ids);
          corners = std::move(roi_corners);
          used_roi_detection = true;
        }
      }
    }

    if (!used_roi_detection) {
      cv::aruco::detectMarkers(detect_img, tag_dict_, corners, ids, tag_params_);
    }

    auto debug_corners = corners;
    const cv::Point2f debug_img_center(
      static_cast<float>(detect_img.cols) / 2.0f,
      static_cast<float>(detect_img.rows) / 2.0f);

    const int target_index = findTargetIndex(ids);

    if (target_index >= 0) {
      updateTrackingState(debug_corners[static_cast<size_t>(target_index)], image_stamp);
    } else {
      has_last_target_detect_ = false;
    }

    if (detect_scale < 0.9999f) {
      scaleCorners(corners, 1.0f / detect_scale);
    }

    DetectionEstimate estimate;
    if (target_index >= 0) {
      const auto & target_corners = corners[static_cast<size_t>(target_index)];
      estimate.detected = true;
      estimate.matched_id = ids[static_cast<size_t>(target_index)];
      estimate.center_px = cv::Point2f(
        (target_corners[0].x + target_corners[1].x + target_corners[2].x + target_corners[3].x) * 0.25f,
        (target_corners[0].y + target_corners[1].y + target_corners[2].y + target_corners[3].y) * 0.25f);

      const cv::Point2f img_center(
        static_cast<float>(frame.cols) / 2.0f,
        static_cast<float>(frame.rows) / 2.0f);
      estimate.pixel_err_u = estimate.center_px.x - img_center.x;
      estimate.pixel_err_v = estimate.center_px.y - img_center.y;
      estimate.pixel_err_norm = std::sqrt(
        (estimate.pixel_err_u * estimate.pixel_err_u) +
        (estimate.pixel_err_v * estimate.pixel_err_v));

      const float fx_safe = std::max(fx_, 1.0e-6f);
      const float fy_safe = std::max(fy_, 1.0e-6f);
      estimate.err_u_norm = (estimate.center_px.x - cx_) / fx_safe;
      estimate.err_v_norm = (estimate.center_px.y - cy_) / fy_safe;
      estimate.marker_span_px = computeMarkerSpanPx(target_corners);

      const float edge_dx = target_corners[1].x - target_corners[0].x;
      const float edge_dy = target_corners[1].y - target_corners[0].y;
      estimate.yaw_primary_rad = normalizeAngle(std::atan2(edge_dy, edge_dx));
      estimate.yaw_alt_rad = estimate.yaw_primary_rad;

      std::vector<PoseCandidate> pose_candidates;
      if (estimatePoseCandidates(target_corners, pose_candidates)) {
        estimate.pose_valid = true;
        estimate.solution_count = static_cast<uint8_t>(std::min<size_t>(
          pose_candidates.size(), std::numeric_limits<uint8_t>::max()));
        estimate.yaw_primary_rad = pose_candidates.front().yaw_rad;
        estimate.reproj_primary_px = pose_candidates.front().reproj_err_px;
        estimate.yaw_alt_rad = estimate.yaw_primary_rad;
        estimate.reproj_alt_px = estimate.reproj_primary_px;

        if (pose_candidates.size() > 1U) {
          estimate.yaw_alt_rad = pose_candidates[1].yaw_rad;
          estimate.reproj_alt_px = pose_candidates[1].reproj_err_px;
        }
      }

      estimate.confidence = computeDetectionConfidence(
        estimate.pose_valid, estimate.reproj_primary_px, estimate.marker_span_px);
    }

    std_msgs::msg::Bool detected_msg;
    detected_msg.data = estimate.detected;
    detected_pub_->publish(detected_msg);
    publishLandingError(output_header, estimate);

    if (estimate.detected) {
      geometry_msgs::msg::PointStamped pixel_error_msg;
      pixel_error_msg.header = output_header;
      pixel_error_msg.point.x = static_cast<double>(estimate.pixel_err_u);
      pixel_error_msg.point.y = static_cast<double>(estimate.pixel_err_v);
      pixel_error_msg.point.z = static_cast<double>(estimate.yaw_primary_rad);
      pixel_error_pub_->publish(pixel_error_msg);

      RCLCPP_DEBUG(this->get_logger(),
        "tag id=%d center=(%.1f, %.1f) err_px=(%.1f, %.1f) err_n=(%.4f, %.4f) yaw=%.3f conf=%.2f",
        estimate.matched_id, estimate.center_px.x, estimate.center_px.y,
        estimate.pixel_err_u, estimate.pixel_err_v,
        estimate.err_u_norm, estimate.err_v_norm,
        estimate.yaw_primary_rad, estimate.confidence);
    }

    const bool publish_debug = shouldPublishDebug(image_stamp, estimate.detected);
    if (!publish_debug) {
      return;
    }

    namespace enc = sensor_msgs::image_encodings;
    cv::Mat debug_img = (detect_img.data == gray.data) ? detect_img.clone() : detect_img;

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(debug_img, debug_corners, ids, cv::Scalar(255));
    }

    constexpr int kCross = 12;
    const cv::Scalar kCrossColor(180);
    cv::line(debug_img,
      {static_cast<int>(debug_img_center.x) - kCross, static_cast<int>(debug_img_center.y)},
      {static_cast<int>(debug_img_center.x) + kCross, static_cast<int>(debug_img_center.y)},
      kCrossColor, 2);
    cv::line(debug_img,
      {static_cast<int>(debug_img_center.x), static_cast<int>(debug_img_center.y) - kCross},
      {static_cast<int>(debug_img_center.x), static_cast<int>(debug_img_center.y) + kCross},
      kCrossColor, 2);

    if (estimate.detected) {
      const cv::Point2f debug_target_center(
        estimate.center_px.x * detect_scale,
        estimate.center_px.y * detect_scale);
      cv::circle(debug_img,
        cv::Point(
          static_cast<int>(debug_target_center.x),
          static_cast<int>(debug_target_center.y)),
        6, cv::Scalar(255), -1);
      cv::line(debug_img,
        cv::Point(static_cast<int>(debug_img_center.x), static_cast<int>(debug_img_center.y)),
        cv::Point(static_cast<int>(debug_target_center.x), static_cast<int>(debug_target_center.y)),
        cv::Scalar(255), 2);
    }

    cv::Mat composed_debug = composeDebugDisplay(debug_img, estimate);
    debug_image_pub_.publish(*cv_bridge::CvImage(output_header, enc::MONO8, composed_debug).toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string debug_image_topic_;
  std::string pixel_error_topic_;
  std::string detected_topic_;
  std::string landing_error_topic_;
  std::string status_topic_;
  std::string velocity_topic_;
  std::string height_distance_topic_;
  std::string optical_flow_topic_;
  int target_marker_id_{-1};
  bool always_publish_debug_{true};
  double debug_publish_period_s_{0.0};
  double debug_render_scale_{6.0};
  int detect_max_dimension_{640};
  int debug_panel_width_px_{420};
  bool roi_search_enabled_{true};
  double roi_max_age_s_{0.35};
  float roi_margin_scale_{2.5f};
  int roi_min_size_px_{96};

  float fx_{320.0f};
  float fy_{320.0f};
  float cx_{320.0f};
  float cy_{240.0f};
  float marker_size_m_{0.20f};
  float confidence_reproj_scale_px_{4.0f};
  float confidence_min_marker_span_px_{18.0f};
  float confidence_full_marker_span_px_{72.0f};

  bool camera_info_received_{false};
  cv::Mat camera_matrix_;
  std::vector<double> dist_coeffs_;

  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};
  bool debug_publish_initialized_{false};
  cv::Point2f last_target_center_detect_{0.0f, 0.0f};
  float last_target_radius_detect_{0.0f};
  rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_target_detect_{false};

  LandingStatusTelemetry landing_status_;
  VelocityCommandTelemetry velocity_cmd_;
  RangeTelemetry range_telemetry_;
  FlowTelemetry flow_telemetry_;

  cv::Ptr<cv::aruco::Dictionary> tag_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> tag_params_;

  image_transport::Publisher debug_image_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<uav_visual_landing::msg::LandingStatus>::SharedPtr landing_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr range_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorOpticalFlow>::SharedPtr optical_flow_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::LandingError>::SharedPtr landing_error_pub_;
};

}  // namespace uav_visual_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_visual_landing::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
