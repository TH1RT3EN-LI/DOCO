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

      image_topic_ = this->get_parameter("image_topic").as_string();
      camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
      debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
      pixel_error_topic_ = this->get_parameter("pixel_error_topic").as_string();
      detected_topic_ = this->get_parameter("detected_topic").as_string();
      target_marker_id_ = this->get_parameter("target_marker_id").as_int();
      always_publish_debug_ = this->get_parameter("always_publish_debug").as_bool();

      fx_ = static_cast<float>(this->get_parameter("camera_fx").as_double());
      fy_ = static_cast<float>(this->get_parameter("camera_fy").as_double());
      cx_ = static_cast<float>(this->get_parameter("camera_cx").as_double());
      cy_ = static_cast<float>(this->get_parameter("camera_cy").as_double());

      tag_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
      tag_params_ = cv::aruco::DetectorParameters::create();

      debug_image_pub_ = image_transport::create_publisher(this, debug_image_topic_);
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
    }

  private:
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
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      }
      catch (const cv_bridge::Exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      const cv::Mat &frame = cv_ptr->image;
      if (frame.empty())
        return;

      cv::Mat debug_img = frame.clone();

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      cv::aruco::detectMarkers(frame, tag_dict_, corners, ids, tag_params_, rejected);

      if (!ids.empty())
      {
        cv::aruco::drawDetectedMarkers(debug_img, corners, ids);
      }

      std::optional<cv::Point2f> target_center;
      double target_yaw = 0.0;
      int matched_id = -1;

      for (size_t i = 0; i < ids.size(); ++i)
      {
        if (target_marker_id_ < 0 || ids[i] == target_marker_id_)
        {
          const auto &c = corners[i];
          const cv::Point2f center(
              (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f,
              (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f);
          target_center = center;
          const float dx = c[1].x - c[0].x;
          const float dy = c[1].y - c[0].y;
          target_yaw = std::atan2(dy, dx);
          matched_id = ids[i];
          break;
        }
      }

      const cv::Point2f img_center(
          static_cast<float>(frame.cols) / 2.0f,
          static_cast<float>(frame.rows) / 2.0f);

      constexpr int kCross = 12;
      const cv::Scalar kYellow(0, 255, 255);
      cv::line(debug_img,
               {static_cast<int>(img_center.x) - kCross, static_cast<int>(img_center.y)},
               {static_cast<int>(img_center.x) + kCross, static_cast<int>(img_center.y)},
               kYellow, 2);
      cv::line(debug_img,
               {static_cast<int>(img_center.x), static_cast<int>(img_center.y) - kCross},
               {static_cast<int>(img_center.x), static_cast<int>(img_center.y) + kCross},
               kYellow, 2);

      const bool detected = target_center.has_value();

      std_msgs::msg::Bool detected_msg;
      detected_msg.data = detected;
      detected_pub_->publish(detected_msg);

      if (detected)
      {
        const float err_u = target_center->x - img_center.x;
        const float err_v = target_center->y - img_center.y;
        const float err_norm = std::sqrt(err_u * err_u + err_v * err_v);


        cv::circle(debug_img,
                   cv::Point(static_cast<int>(target_center->x), static_cast<int>(target_center->y)),
                   6, cv::Scalar(0, 0, 255), -1);
        cv::line(debug_img,
                 cv::Point(static_cast<int>(img_center.x), static_cast<int>(img_center.y)),
                 cv::Point(static_cast<int>(target_center->x), static_cast<int>(target_center->y)),
                 cv::Scalar(0, 0, 255), 2);

        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "ID:%d  err=(%.1f, %.1f)px  |e|=%.1f", matched_id, err_u, err_v, err_norm);
        cv::putText(debug_img, buf, cv::Point(8, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

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
      else
      {
        cv::putText(debug_img, "NO TARGET", cv::Point(8, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      }

      if (always_publish_debug_ || detected)
      {
        debug_image_pub_.publish(*cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg());
      }
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string debug_image_topic_;
    std::string pixel_error_topic_;
    std::string detected_topic_;
    int target_marker_id_{-1};
    bool always_publish_debug_{true};

    float fx_{320.0f};
    float fy_{320.0f};
    float cx_{320.0f};
    float cy_{240.0f};
    bool camera_info_received_{false};

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
