#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

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

      image_topic_ = this->get_parameter("image_topic").as_string();
      camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
      debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
      pixel_error_topic_ = this->get_parameter("pixel_error_topic").as_string();
      detected_topic_ = this->get_parameter("detected_topic").as_string();
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

      tag_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
      tag_params_ = cv::aruco::DetectorParameters::create();

      debug_image_pub_ = image_transport::create_publisher(
          this, debug_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
      pixel_error_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
          pixel_error_topic_, 10);
      detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
          detected_topic_, 10);

      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic_, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
          {
            onCameraInfo(msg);
          });

      image_sub_ = image_transport::create_subscription(
          this, image_topic_,
          [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg)
          {
            onImage(msg);
          },
          "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

      RCLCPP_INFO(this->get_logger(),
                  "[apriltag_detector] image=%s  dict=DICT_APRILTAG_16h5  target_id=%d",
                  image_topic_.c_str(), target_marker_id_);
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
    static void offsetCorners(std::vector<std::vector<cv::Point2f>> &corners, const cv::Point2f &offset)
    {
      if (std::abs(offset.x) < 1.0e-6f && std::abs(offset.y) < 1.0e-6f)
      {
        return;
      }

      for (auto &marker_corners : corners)
      {
        for (auto &point : marker_corners)
        {
          point.x += offset.x;
          point.y += offset.y;
        }
      }
    }

    static void scaleCorners(std::vector<std::vector<cv::Point2f>> &corners, float scale)
    {
      if (std::abs(scale - 1.0f) < 1.0e-6f)
      {
        return;
      }

      for (auto &marker_corners : corners)
      {
        for (auto &point : marker_corners)
        {
          point.x *= scale;
          point.y *= scale;
        }
      }
    }

    rclcpp::Time resolveImageStamp(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const
    {
      const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
      return has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
    }

    int findTargetIndex(const std::vector<int> &ids) const
    {
      for (size_t i = 0; i < ids.size(); ++i)
      {
        if (target_marker_id_ < 0 || ids[i] == target_marker_id_)
        {
          return static_cast<int>(i);
        }
      }
      return -1;
    }

    cv::Rect computeRoiRect(const cv::Size &image_size) const
    {
      const int min_side = std::max(roi_min_size_px_, 8);
      const float half_span = std::max(0.5f * static_cast<float>(min_side), roi_margin_scale_ * last_target_radius_detect_);

      const int left = std::max(0, static_cast<int>(std::floor(last_target_center_detect_.x - half_span)));
      const int top = std::max(0, static_cast<int>(std::floor(last_target_center_detect_.y - half_span)));
      const int right = std::min(image_size.width, static_cast<int>(std::ceil(last_target_center_detect_.x + half_span)));
      const int bottom = std::min(image_size.height, static_cast<int>(std::ceil(last_target_center_detect_.y + half_span)));

      return cv::Rect(left, top, std::max(0, right - left), std::max(0, bottom - top));
    }

    void updateTrackingState(const std::vector<cv::Point2f> &detect_corners, const rclcpp::Time &stamp)
    {
      if (detect_corners.size() != 4)
      {
        return;
      }

      cv::Point2f center(0.0f, 0.0f);
      for (const auto &point : detect_corners)
      {
        center += point;
      }
      center *= 0.25f;

      float max_radius = 0.0f;
      for (const auto &point : detect_corners)
      {
        max_radius = std::max(max_radius, std::hypot(point.x - center.x, point.y - center.y));
      }

      last_target_center_detect_ = center;
      last_target_radius_detect_ = std::max(max_radius, 8.0f);
      last_target_stamp_ = stamp;
      has_last_target_detect_ = true;
    }

    cv::Mat toGray(const cv::Mat &frame, const std::string &encoding,
                   const sensor_msgs::msg::Image::ConstSharedPtr &msg) const
    {
      namespace enc = sensor_msgs::image_encodings;

      cv::Mat gray;
      if (encoding == enc::MONO8)
      {
        return frame;
      }
      if (encoding == enc::BGR8)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        return gray;
      }
      if (encoding == enc::RGB8)
      {
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        return gray;
      }
      if (encoding == enc::BGRA8)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
        return gray;
      }
      if (encoding == enc::RGBA8)
      {
        cv::cvtColor(frame, gray, cv::COLOR_RGBA2GRAY);
        return gray;
      }

      return cv_bridge::toCvCopy(msg, enc::MONO8)->image;
    }

    bool shouldPublishDebug(const rclcpp::Time &stamp, bool detected)
    {
      if (!(always_publish_debug_ || detected))
      {
        return false;
      }

      if (debug_image_pub_.getNumSubscribers() == 0)
      {
        return false;
      }

      if (debug_publish_period_s_ <= 1.0e-6)
      {
        return true;
      }

      if (!debug_publish_initialized_)
      {
        last_debug_publish_time_ = stamp;
        debug_publish_initialized_ = true;
        return true;
      }

      if ((stamp - last_debug_publish_time_).seconds() + 1.0e-9 < debug_publish_period_s_)
      {
        return false;
      }

      last_debug_publish_time_ = stamp;
      return true;
    }

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      if (camera_info_received_)
        return;

      fx_ = static_cast<float>(msg->k[0]);
      fy_ = static_cast<float>(msg->k[4]);
      cx_ = static_cast<float>(msg->k[2]);
      cy_ = static_cast<float>(msg->k[5]);
      camera_info_received_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "[apriltag_detector] CameraInfo received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                  fx_, fy_, cx_, cy_);
    }

    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvShare(msg);
      }
      catch (const cv_bridge::Exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      const cv::Mat &frame = cv_ptr->image;
      if (frame.empty())
        return;

      const rclcpp::Time image_stamp = resolveImageStamp(msg);

      const std::string &encoding = cv_ptr->encoding;
      cv::Mat gray;
      try
      {
        gray = toGray(frame, encoding, msg);
      }
      catch (const cv_bridge::Exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "gray conversion failed for encoding '%s': %s",
                    encoding.c_str(), e.what());
        return;
      }

      cv::Mat detect_img = gray;
      float detect_scale = 1.0f;
      const int frame_max_dim = std::max(gray.cols, gray.rows);
      if (detect_max_dimension_ > 0 && frame_max_dim > detect_max_dimension_)
      {
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
          offsetCorners(roi_corners, cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y)));
          if (findTargetIndex(roi_ids) >= 0)
          {
            ids = std::move(roi_ids);
            corners = std::move(roi_corners);
            used_roi_detection = true;
          }
        }
      }

      if (!used_roi_detection)
      {
        cv::aruco::detectMarkers(detect_img, tag_dict_, corners, ids, tag_params_);
      }

      auto debug_corners = corners;
      const cv::Point2f debug_img_center(
          static_cast<float>(detect_img.cols) / 2.0f,
          static_cast<float>(detect_img.rows) / 2.0f);

      const int target_index = findTargetIndex(ids);

      if (target_index >= 0)
      {
        updateTrackingState(debug_corners[static_cast<size_t>(target_index)], image_stamp);
      }
      else
      {
        has_last_target_detect_ = false;
      }

      if (detect_scale < 0.9999f)
      {
        scaleCorners(corners, 1.0f / detect_scale);
      }

      std::optional<cv::Point2f> target_center;
      double target_yaw = 0.0;
      int matched_id = -1;

      if (target_index >= 0)
      {
        const auto &c = corners[static_cast<size_t>(target_index)];
        const cv::Point2f center(
            (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f,
            (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f);
        target_center = center;
        const float dx = c[1].x - c[0].x;
        const float dy = c[1].y - c[0].y;
        target_yaw = std::atan2(dy, dx);
        matched_id = ids[static_cast<size_t>(target_index)];
      }

      const cv::Point2f img_center(
          static_cast<float>(frame.cols) / 2.0f,
          static_cast<float>(frame.rows) / 2.0f);

      const bool detected = target_center.has_value();
      const bool publish_debug = shouldPublishDebug(image_stamp, detected);
      float err_u = 0.0f;
      float err_v = 0.0f;
      float err_norm = 0.0f;

      std_msgs::msg::Bool detected_msg;
      detected_msg.data = detected;
      detected_pub_->publish(detected_msg);

      if (detected)
      {
        err_u = target_center->x - img_center.x;
        err_v = target_center->y - img_center.y;
        err_norm = std::sqrt(err_u * err_u + err_v * err_v);

        geometry_msgs::msg::PointStamped pe;
        pe.header = msg->header;
        pe.point.x = static_cast<double>(err_u);
        pe.point.y = static_cast<double>(err_v);
        pe.point.z = target_yaw;
        pixel_error_pub_->publish(pe);

        RCLCPP_DEBUG(this->get_logger(),
                     "tag id=%d  center=(%.1f, %.1f)  err=(%.1f, %.1f)px",
                     matched_id, target_center->x, target_center->y, err_u, err_v);
      }

      if (!publish_debug)
      {
        return;
      }

      namespace enc = sensor_msgs::image_encodings;
      cv::Mat debug_img = (detect_img.data == gray.data) ? detect_img.clone() : detect_img;

      if (!ids.empty())
      {
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

      if (detected)
      {
        const cv::Point2f debug_target_center(
            target_center->x * detect_scale,
            target_center->y * detect_scale);
        cv::circle(debug_img,
                   cv::Point(static_cast<int>(debug_target_center.x), static_cast<int>(debug_target_center.y)),
                   6, cv::Scalar(255), -1);
        cv::line(debug_img,
                 cv::Point(static_cast<int>(debug_img_center.x), static_cast<int>(debug_img_center.y)),
                 cv::Point(static_cast<int>(debug_target_center.x), static_cast<int>(debug_target_center.y)),
                 cv::Scalar(255), 2);

        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "ID:%d  err=(%.1f, %.1f)px  |e|=%.1f", matched_id, err_u, err_v, err_norm);
        cv::putText(debug_img, buf, cv::Point(8, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 2);
      }
      else
      {
        cv::putText(debug_img, "NO TARGET", cv::Point(8, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255), 2);
      }

      debug_image_pub_.publish(*cv_bridge::CvImage(msg->header, enc::MONO8, debug_img).toImageMsg());
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string debug_image_topic_;
    std::string pixel_error_topic_;
    std::string detected_topic_;
    int target_marker_id_{-1};
    bool always_publish_debug_{true};
    double debug_publish_period_s_{0.0};
    int detect_max_dimension_{640};
    bool roi_search_enabled_{true};
    double roi_max_age_s_{0.35};
    float roi_margin_scale_{2.5f};
    int roi_min_size_px_{96};

    float fx_{320.0f};
    float fy_{320.0f};
    float cx_{320.0f};
    float cy_{240.0f};
    bool camera_info_received_{false};
    rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};
    bool debug_publish_initialized_{false};
    cv::Point2f last_target_center_detect_{0.0f, 0.0f};
    float last_target_radius_detect_{0.0f};
    rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};
    bool has_last_target_detect_{false};

    cv::Ptr<cv::aruco::Dictionary> tag_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> tag_params_;

    image_transport::Publisher debug_image_pub_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  };

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_visual_landing::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
