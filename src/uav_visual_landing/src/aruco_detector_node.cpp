#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <uav_visual_landing/msg/landing_controller_state.hpp>
#include <uav_visual_landing/msg/target_observation.hpp>

namespace uav_visual_landing
{

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector_node")
  {
    this->declare_parameter<std::string>("image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/uav/camera/camera_info");
    this->declare_parameter<std::string>(
      "target_observation_topic",
      "/uav/visual_landing/target_observation");
    this->declare_parameter<std::string>(
      "controller_state_topic",
      "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>("debug_image_topic", "/uav/visual_landing/debug_image");
    this->declare_parameter<int>("target_marker_id", 0);
    this->declare_parameter<double>("tag_size_m", 0.1625);
    this->declare_parameter<double>("confidence_reproj_scale_px", 4.0);
    this->declare_parameter<double>("confidence_min_marker_span_px", 18.0);
    this->declare_parameter<double>("confidence_full_marker_span_px", 72.0);

    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    target_observation_topic_ = this->get_parameter("target_observation_topic").as_string();
    controller_state_topic_ = this->get_parameter("controller_state_topic").as_string();
    debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
    target_marker_id_ = this->get_parameter("target_marker_id").as_int();
    tag_size_m_ = static_cast<float>(this->get_parameter("tag_size_m").as_double());
    confidence_reproj_scale_px_ =
      static_cast<float>(this->get_parameter("confidence_reproj_scale_px").as_double());
    confidence_min_marker_span_px_ =
      static_cast<float>(this->get_parameter("confidence_min_marker_span_px").as_double());
    confidence_full_marker_span_px_ =
      static_cast<float>(this->get_parameter("confidence_full_marker_span_px").as_double());

    tag_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    tag_params_ = cv::aruco::DetectorParameters::create();
    tag_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    auto observation_qos = rclcpp::SensorDataQoS();
    observation_qos.keep_last(1);
    observation_pub_ = this->create_publisher<uav_visual_landing::msg::TargetObservation>(
      target_observation_topic_, observation_qos);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_, rclcpp::SensorDataQoS());
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
      {
        onCameraInfo(msg);
      });
    controller_state_sub_ =
      this->create_subscription<uav_visual_landing::msg::LandingControllerState>(
      controller_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const uav_visual_landing::msg::LandingControllerState::SharedPtr msg)
      {
        telemetry_.phase = msg->phase;
        telemetry_.height_source = msg->height_source;
        telemetry_.terminal_trigger_source = msg->terminal_trigger_source;
        telemetry_.height_valid = msg->height_valid;
        telemetry_.raw_flow_fresh = msg->raw_flow_fresh;
        telemetry_.height_measurement_m = msg->height_measurement_m;
        telemetry_.control_height_m = msg->control_height_m;
        telemetry_.err_u_norm_filtered = msg->err_u_norm_filtered;
        telemetry_.err_v_norm_filtered = msg->err_v_norm_filtered;
        telemetry_.err_u_rate_norm_s = msg->err_u_rate_norm_s;
        telemetry_.err_v_rate_norm_s = msg->err_v_rate_norm_s;
        telemetry_.lateral_error_valid = msg->lateral_error_valid;
        telemetry_.lateral_error_x_m = msg->lateral_error_x_m;
        telemetry_.lateral_error_y_m = msg->lateral_error_y_m;
        telemetry_.lateral_error_m = msg->lateral_error_m;
        telemetry_.lateral_error_rate_x_mps = msg->lateral_error_rate_x_mps;
        telemetry_.lateral_error_rate_y_mps = msg->lateral_error_rate_y_mps;
        telemetry_.active_max_vxy = msg->active_max_vxy;
        telemetry_.align_enter_lateral_m = msg->align_enter_lateral_m;
        telemetry_.align_exit_lateral_m = msg->align_exit_lateral_m;
        telemetry_.z_target_height_m = msg->z_target_height_m;
        telemetry_.z_error_m = msg->z_error_m;
        telemetry_.xy_control_mode = msg->xy_control_mode;
      });
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg)
      {
        onImage(msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "aruco_detector_node: image=%s camera_info=%s target_observation=%s debug_image=%s marker_id=%d tag_size=%.3f",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      target_observation_topic_.c_str(),
      debug_image_topic_.c_str(),
      target_marker_id_,
      tag_size_m_);
  }

private:
  struct PoseCandidate
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    float yaw_rad{0.0f};
    float reproj_err_px{std::numeric_limits<float>::infinity()};
    float tag_depth_m{0.0f};
  };

  struct ControllerTelemetry
  {
    std::string phase{"READY"};
    std::string height_source{"ODOM"};
    std::string terminal_trigger_source{"NONE"};
    bool height_valid{false};
    bool raw_flow_fresh{false};
    float height_measurement_m{0.0f};
    float control_height_m{0.0f};
    float err_u_norm_filtered{0.0f};
    float err_v_norm_filtered{0.0f};
    float err_u_rate_norm_s{0.0f};
    float err_v_rate_norm_s{0.0f};
    bool lateral_error_valid{false};
    float lateral_error_x_m{0.0f};
    float lateral_error_y_m{0.0f};
    float lateral_error_m{0.0f};
    float lateral_error_rate_x_mps{0.0f};
    float lateral_error_rate_y_mps{0.0f};
    float active_max_vxy{0.0f};
    float align_enter_lateral_m{0.08f};
    float align_exit_lateral_m{0.05f};
    float z_target_height_m{0.0f};
    float z_error_m{0.0f};
    std::string xy_control_mode{"vision_pd_metric"};
  };

  static float clamp01(float value)
  {
    return std::max(0.0f, std::min(value, 1.0f));
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
    return {numeric.at<double>(0, 0), numeric.at<double>(1, 0), numeric.at<double>(2, 0)};
  }

  std::vector<cv::Point3f> buildMarkerObjectPoints() const
  {
    const float half = tag_size_m_ * 0.5f;
    return {
      {-half, half, 0.0f},
      {half, half, 0.0f},
      {half, -half, 0.0f},
      {-half, -half, 0.0f}};
  }

  static float computePoseYawRad(const cv::Vec3d & rvec)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    return normalizeAngle(
      static_cast<float>(std::atan2(
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0))));
  }

  static float computeTagDepthFromTvec(const cv::Vec3d & tvec)
  {
    return static_cast<float>(tvec[2]);
  }

  float computeTagDepthFromSpanPx(float marker_span_px) const
  {
    const float focal_mean = 0.5f * (fx_ + fy_);
    if (marker_span_px <= 1.0e-6f || focal_mean <= 1.0e-6f || tag_size_m_ <= 1.0e-6f) {
      return 0.0f;
    }
    return (tag_size_m_ * focal_mean) / marker_span_px;
  }

  static bool isPositiveFiniteDepth(float depth_m)
  {
    return std::isfinite(depth_m) && depth_m > 0.0f;
  }

  float computeSpanConfidence(float marker_span_px) const
  {
    const float span_den = std::max(
      confidence_full_marker_span_px_ - confidence_min_marker_span_px_, 1.0f);
    return clamp01((marker_span_px - confidence_min_marker_span_px_) / span_den);
  }

  float computeReprojectionErrorPx(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & corners,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec) const
  {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);
    if (projected_points.size() != corners.size() || projected_points.empty()) {
      return std::numeric_limits<float>::infinity();
    }

    double sum_sq = 0.0;
    for (size_t index = 0; index < corners.size(); ++index) {
      const double dx = static_cast<double>(projected_points[index].x - corners[index].x);
      const double dy = static_cast<double>(projected_points[index].y - corners[index].y);
      sum_sq += (dx * dx) + (dy * dy);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(corners.size())));
  }

  float computeMarkerSpanPx(const std::vector<cv::Point2f> & corners) const
  {
    if (corners.size() != 4) {
      return 0.0f;
    }
    float total = 0.0f;
    for (size_t index = 0; index < corners.size(); ++index) {
      const auto & a = corners[index];
      const auto & b = corners[(index + 1U) % corners.size()];
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return 0.25f * total;
  }

  float computeDetectionConfidence(bool pose_valid, float reproj_err_px, float marker_span_px) const
  {
    const float size_conf = computeSpanConfidence(marker_span_px);
    if (!pose_valid || !std::isfinite(reproj_err_px)) {
      return 0.35f * size_conf;
    }
    const float reproj_scale = std::max(confidence_reproj_scale_px_, 1.0e-3f);
    const float reproj_conf = std::exp(-reproj_err_px / reproj_scale);
    return clamp01((0.70f * reproj_conf) + (0.30f * size_conf));
  }

  bool estimatePose(
    const std::vector<cv::Point2f> & corners,
    std::vector<PoseCandidate> & candidates)
  {
    candidates.clear();
    if (corners.size() != 4 || tag_size_m_ <= 1.0e-6f || !camera_info_received_) {
      return false;
    }

    const auto object_points = buildMarkerObjectPoints();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    try {
      cv::solvePnPGeneric(
        object_points, corners, camera_matrix_, dist_coeffs_, rvecs, tvecs,
        false, cv::SOLVEPNP_IPPE_SQUARE);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "solvePnPGeneric failed: %s", e.what());
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
          this->get_logger(),
          *this->get_clock(), 2000, "solvePnP failed: %s", e.what());
        return false;
      }
      PoseCandidate candidate;
      candidate.rvec = rvec;
      candidate.tvec = tvec;
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(object_points, corners, rvec, tvec);
      candidate.tag_depth_m = computeTagDepthFromTvec(candidate.tvec);
      candidates.push_back(candidate);
      return true;
    }

    for (size_t index = 0; index < std::min(rvecs.size(), tvecs.size()); ++index) {
      PoseCandidate candidate;
      candidate.rvec = matToVec3d(rvecs[index]);
      candidate.tvec = matToVec3d(tvecs[index]);
      candidate.yaw_rad = computePoseYawRad(candidate.rvec);
      candidate.reproj_err_px = computeReprojectionErrorPx(
        object_points, corners, candidate.rvec,
        candidate.tvec);
      candidate.tag_depth_m = computeTagDepthFromTvec(candidate.tvec);
      candidates.push_back(candidate);
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const PoseCandidate & lhs, const PoseCandidate & rhs)
      {
        const bool lhs_valid = isPositiveFiniteDepth(lhs.tag_depth_m);
        const bool rhs_valid = isPositiveFiniteDepth(rhs.tag_depth_m);
        if (lhs_valid != rhs_valid) {
          return lhs_valid > rhs_valid;
        }
        return lhs.reproj_err_px < rhs.reproj_err_px;
      });
    return !candidates.empty();
  }

  int findTargetIndex(const std::vector<int> & ids) const
  {
    for (size_t index = 0; index < ids.size(); ++index) {
      if (target_marker_id_ < 0 || ids[index] == target_marker_id_) {
        return static_cast<int>(index);
      }
    }
    return -1;
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
    dist_coeffs_.assign(msg->d.begin(), msg->d.end());

    RCLCPP_INFO(
      this->get_logger(),
      "CameraInfo received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      fx_, fy_, cx_, cy_);
  }

  void publishDebugImage(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const cv::Mat & gray_frame,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    int target_index,
    const uav_visual_landing::msg::TargetObservation & observation)
  {
    if (debug_image_pub_->get_subscription_count() == 0U) {
      return;
    }

    cv::Mat debug_bgr;
    cv::cvtColor(gray_frame, debug_bgr, cv::COLOR_GRAY2BGR);
    if (!corners.empty()) {
      cv::aruco::drawDetectedMarkers(debug_bgr, corners, ids);
    }

    const cv::Point image_center(
      static_cast<int>(std::lround(cx_)),
      static_cast<int>(std::lround(cy_)));
    cv::drawMarker(debug_bgr, image_center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 28, 2);

    if (target_index >= 0) {
      const auto & target_corners = corners[static_cast<size_t>(target_index)];
      cv::Point target_center(0, 0);
      for (const auto & point : target_corners) {
        target_center.x += static_cast<int>(std::lround(point.x));
        target_center.y += static_cast<int>(std::lround(point.y));
      }
      target_center.x /= 4;
      target_center.y /= 4;
      cv::circle(debug_bgr, target_center, 6, cv::Scalar(0, 255, 0), 2);
      cv::line(debug_bgr, image_center, target_center, cv::Scalar(255, 255, 0), 2);
    }

    int y = 26;
    const int line_height = 22;
    const double font_scale = 0.60;
    const int thickness = 2;
    auto drawLine = [&](const std::string & text)
      {
        cv::putText(
          debug_bgr, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
          font_scale, cv::Scalar(16, 16, 16), thickness + 2, cv::LINE_AA);
        cv::putText(
          debug_bgr, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX,
          font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
        y += line_height;
      };

    drawLine("VISUAL LANDING DEBUG");
    drawLine("PHASE: " + telemetry_.phase);
    drawLine(observation.detected ? "TAG: DETECTED" : "TAG: MISSING");
    drawLine(
      cv::format(
        "TAG_Z=%.2f src=%s conf=%.2f",
        observation.tag_depth_m,
        observation.tag_depth_source.c_str(),
        observation.tag_depth_confidence));
    drawLine(
      cv::format(
        "H_CTRL=%.2f src=%s",
        telemetry_.control_height_m,
        telemetry_.height_source.c_str()));
    drawLine(
      cv::format(
        "FLOW=%.2f valid=%s fresh=%s",
        telemetry_.height_measurement_m,
        telemetry_.height_valid ? "Y" : "N",
        telemetry_.raw_flow_fresh ? "Y" : "N"));
    drawLine("TERM: " + telemetry_.terminal_trigger_source);
    drawLine(cv::format("ERR n=(%.4f, %.4f)", observation.err_u_norm, observation.err_v_norm));
    drawLine(
      cv::format(
        "ERR_F=(%.4f, %.4f)", telemetry_.err_u_norm_filtered,
        telemetry_.err_v_norm_filtered));
    drawLine(
      cv::format(
        "DERR=(%.4f, %.4f)", telemetry_.err_u_rate_norm_s,
        telemetry_.err_v_rate_norm_s));
    drawLine(
      cv::format(
        "LERR=%.3f xy=(%.3f, %.3f)",
        telemetry_.lateral_error_m,
        telemetry_.lateral_error_x_m,
        telemetry_.lateral_error_y_m));
    drawLine(
      cv::format(
        "TH=%.2f/%.2f VXY_MAX=%.3f",
        telemetry_.align_enter_lateral_m,
        telemetry_.align_exit_lateral_m,
        telemetry_.active_max_vxy));
    drawLine(cv::format("ZTG=%.2f ZE=%.2f", telemetry_.z_target_height_m, telemetry_.z_error_m));
    drawLine(
      cv::format(
        "YAW=%.3f rep=%.2f span=%.1f", observation.yaw_err_rad,
        observation.reproj_err_px, observation.marker_span_px));

    const auto output_msg = cv_bridge::CvImage(
      msg->header, sensor_msgs::image_encodings::BGR8,
      debug_bgr).toImageMsg();
    debug_image_pub_->publish(*output_msg);
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for CameraInfo on %s before publishing target observations",
        camera_info_topic_.c_str());
      return;
    }

    cv::Mat gray_frame;
    try {
      gray_frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "cv_bridge failed: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray_frame, tag_dict_, corners, ids, tag_params_);

    uav_visual_landing::msg::TargetObservation observation;
    observation.header = msg->header;

    const int target_index = findTargetIndex(ids);
    if (target_index < 0) {
      observation_pub_->publish(observation);
      publishDebugImage(msg, gray_frame, corners, ids, target_index, observation);
      return;
    }

    const auto & target_corners = corners[static_cast<size_t>(target_index)];
    const cv::Point2f center_px(
      (target_corners[0].x + target_corners[1].x + target_corners[2].x + target_corners[3].x) *
      0.25f,
      (target_corners[0].y + target_corners[1].y + target_corners[2].y + target_corners[3].y) *
      0.25f);

    observation.detected = true;
    observation.pixel_err_u = center_px.x - cx_;
    observation.pixel_err_v = center_px.y - cy_;
    observation.err_u_norm = observation.pixel_err_u / std::max(fx_, 1.0e-6f);
    observation.err_v_norm = observation.pixel_err_v / std::max(fy_, 1.0e-6f);
    observation.marker_span_px = computeMarkerSpanPx(target_corners);

    const float edge_dx = target_corners[1].x - target_corners[0].x;
    const float edge_dy = target_corners[1].y - target_corners[0].y;
    observation.yaw_err_rad = normalizeAngle(std::atan2(edge_dy, edge_dx));

    const float span_conf = computeSpanConfidence(observation.marker_span_px);
    const float span_tag_depth_m = computeTagDepthFromSpanPx(observation.marker_span_px);

    std::vector<PoseCandidate> candidates;
    if (estimatePose(target_corners, candidates)) {
      observation.pose_valid = true;
      observation.yaw_err_rad = candidates.front().yaw_rad;
      observation.reproj_err_px = candidates.front().reproj_err_px;
      if (isPositiveFiniteDepth(candidates.front().tag_depth_m)) {
        observation.tag_depth_valid = true;
        observation.tag_depth_m = candidates.front().tag_depth_m;
        observation.tag_depth_source = "PNP_Z";
        observation.tag_depth_confidence = computeDetectionConfidence(
          true,
          observation.reproj_err_px,
          observation.marker_span_px);
      } else {
        observation.tag_depth_valid = false;
        observation.tag_depth_m = 0.0f;
        observation.tag_depth_source = "NONE";
        observation.tag_depth_confidence = 0.0f;
      }
    } else if (span_tag_depth_m > 0.0f) {
      observation.tag_depth_valid = true;
      observation.tag_depth_m = span_tag_depth_m;
      observation.tag_depth_source = "SPAN_Z";
      observation.tag_depth_confidence = 0.60f * span_conf;
    } else {
      observation.tag_depth_valid = false;
      observation.tag_depth_m = 0.0f;
      observation.tag_depth_source = "NONE";
      observation.tag_depth_confidence = 0.0f;
    }

    observation.confidence = computeDetectionConfidence(
      observation.pose_valid, observation.reproj_err_px, observation.marker_span_px);

    observation_pub_->publish(observation);
    publishDebugImage(msg, gray_frame, corners, ids, target_index, observation);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string target_observation_topic_;
  std::string controller_state_topic_;
  std::string debug_image_topic_;
  int target_marker_id_{0};
  float tag_size_m_{0.1625f};
  float fx_{0.0f};
  float fy_{0.0f};
  float cx_{0.0f};
  float cy_{0.0f};
  float confidence_reproj_scale_px_{4.0f};
  float confidence_min_marker_span_px_{18.0f};
  float confidence_full_marker_span_px_{72.0f};
  bool camera_info_received_{false};
  cv::Mat camera_matrix_;
  std::vector<double> dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> tag_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> tag_params_;
  ControllerTelemetry telemetry_{};
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<uav_visual_landing::msg::LandingControllerState>::SharedPtr
    controller_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<uav_visual_landing::msg::TargetObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

}  // namespace uav_visual_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_visual_landing::ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
