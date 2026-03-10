#include "ugv_controller_emulator/controller_emulator_core.hpp"

#include <algorithm>
#include <array>
#include <errno.h>
#include <functional>
#include <poll.h>
#include <pty.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ugv_controller_emulator
{

namespace
{

std::chrono::nanoseconds timer_period_from_hz(double hz)
{
  const double safe_hz = std::max(1.0, hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_hz));
}

void close_file_descriptor(int & fd) noexcept
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

std::string errno_message(const char * prefix)
{
  return std::string(prefix) + ": " + std::strerror(errno);
}

}

class ControllerEmulatorNode : public rclcpp::Node
{
public:
  ControllerEmulatorNode()
  : Node("ugv_controller_emulator")
  {
    pty_link_path_ = this->declare_parameter<std::string>("pty_link_path", "/tmp/ugv_controller");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/ugv/sim/odom");
    cmd_vel_out_topic_ = this->declare_parameter<std::string>(
      "cmd_vel_out_topic", "/ugv/cmd_vel_sim");

    config_.cmd_timeout = this->declare_parameter<double>("cmd_timeout", 0.5);
    config_.cmd_pub_hz = this->declare_parameter<double>("cmd_pub_hz", 50.0);
    config_.feedback_hz = this->declare_parameter<double>("feedback_hz", 200.0);
    config_.max_vx = this->declare_parameter<double>("max_vx", 0.6);
    config_.max_vy = this->declare_parameter<double>("max_vy", 0.6);
    config_.max_wz = this->declare_parameter<double>("max_wz", 2.0);
    config_.acc_lim_xy = this->declare_parameter<double>("acc_lim_xy", 2.0);
    config_.acc_lim_wz = this->declare_parameter<double>("acc_lim_wz", 3.2);
    config_.battery_voltage = this->declare_parameter<double>("battery_voltage", 24.0);

    last_applied_time_ = SteadyClock::now();
    applied_twist_.stamp = last_applied_time_;

    create_pty_link();

    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, 10);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(20),
      std::bind(&ControllerEmulatorNode::handle_odom, this, std::placeholders::_1));

    command_timer_ = this->create_wall_timer(
      timer_period_from_hz(config_.cmd_pub_hz),
      std::bind(&ControllerEmulatorNode::handle_command_timer, this));
    feedback_timer_ = this->create_wall_timer(
      timer_period_from_hz(config_.feedback_hz),
      std::bind(&ControllerEmulatorNode::handle_feedback_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "controller emulator started: pty_link_path='%s', odom_topic='%s', cmd_vel_out_topic='%s'",
      pty_link_path_.c_str(),
      odom_topic_.c_str(),
      cmd_vel_out_topic_.c_str());
  }

  ~ControllerEmulatorNode() override
  {
    if (command_timer_) {
      command_timer_->cancel();
    }
    if (feedback_timer_) {
      feedback_timer_->cancel();
    }

    rx_thread_stop_.store(true, std::memory_order_release);
    close_file_descriptor(master_fd_);
    close_file_descriptor(slave_fd_);

    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }

    remove_pty_link();
  }

private:
  void create_pty_link()
  {
    int master_fd = -1;
    int slave_fd = -1;
    if (::openpty(&master_fd, &slave_fd, nullptr, nullptr, nullptr) != 0) {
      throw std::runtime_error(errno_message("failed to create PTY"));
    }

    struct termios slave_termios {};
    if (::tcgetattr(slave_fd, &slave_termios) != 0) {
      close_file_descriptor(master_fd);
      close_file_descriptor(slave_fd);
      throw std::runtime_error(errno_message("failed to query PTY termios"));
    }
    ::cfmakeraw(&slave_termios);
    if (::tcsetattr(slave_fd, TCSANOW, &slave_termios) != 0) {
      close_file_descriptor(master_fd);
      close_file_descriptor(slave_fd);
      throw std::runtime_error(errno_message("failed to configure PTY raw mode"));
    }

    char * slave_name_cstr = ::ttyname(slave_fd);
    if (slave_name_cstr == nullptr) {
      close_file_descriptor(master_fd);
      close_file_descriptor(slave_fd);
      throw std::runtime_error(errno_message("failed to resolve PTY slave name"));
    }
    slave_name_ = slave_name_cstr;

    try {
      const std::filesystem::path link_path(pty_link_path_);
      if (link_path.has_parent_path()) {
        std::filesystem::create_directories(link_path.parent_path());
      }
      if (std::filesystem::is_symlink(link_path) || std::filesystem::exists(link_path)) {
        std::filesystem::remove(link_path);
      }
      std::filesystem::create_symlink(slave_name_, link_path);
    } catch (const std::filesystem::filesystem_error & exc) {
      close_file_descriptor(master_fd);
      close_file_descriptor(slave_fd);
      throw std::runtime_error(
              "failed to create pty link path '" + pty_link_path_ + "': " + exc.what());
    }

    master_fd_ = master_fd;
    slave_fd_ = slave_fd;

    RCLCPP_INFO(
      this->get_logger(),
      "PTY ready: slave='%s' -> link='%s'",
      slave_name_.c_str(),
      pty_link_path_.c_str());

    rx_thread_ = std::thread(&ControllerEmulatorNode::run_receive_loop, this);
  }

  void remove_pty_link() noexcept
  {
    try {
      const std::filesystem::path link_path(pty_link_path_);
      if (std::filesystem::is_symlink(link_path)) {
        std::filesystem::remove(link_path);
      }
    } catch (const std::filesystem::filesystem_error &) {
    }
  }

  void run_receive_loop()
  {
    CommandParser parser;
    std::array<uint8_t, 256> read_buf{};

    while (rclcpp::ok() && !rx_thread_stop_.load(std::memory_order_acquire)) {
      const int fd = master_fd_;
      if (fd < 0) {
        break;
      }

      struct pollfd poll_fd
      {
        fd, POLLIN | POLLERR | POLLHUP, 0
      };
      const int poll_rc = ::poll(&poll_fd, 1, 100);
      if (poll_rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (!rx_thread_stop_.load(std::memory_order_acquire)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        continue;
      }
      if (poll_rc == 0) {
        continue;
      }
      if ((poll_fd.revents & POLLIN) == 0) {
        if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
          if (rx_thread_stop_.load(std::memory_order_acquire)) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        continue;
      }

      const ssize_t n = ::read(fd, read_buf.data(), read_buf.size());
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (rx_thread_stop_.load(std::memory_order_acquire)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (n == 0) {
        continue;
      }

      const auto stamp = SteadyClock::now();
      for (ssize_t i = 0; i < n; ++i) {
        auto parsed = parser.push_byte(read_buf[static_cast<std::size_t>(i)], stamp);
        if (!parsed.has_value()) {
          continue;
        }
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_twist_ = *parsed;
      }
    }
  }

  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    TwistState odom{};
    odom.vx = static_cast<double>(msg->twist.twist.linear.x);
    odom.vy = static_cast<double>(msg->twist.twist.linear.y);
    odom.wz = static_cast<double>(msg->twist.twist.angular.z);
    odom.stamp = SteadyClock::now();

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_state_ = odom;
  }

  void handle_command_timer()
  {
    TwistState target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target = target_twist_;
    }

    const auto now = SteadyClock::now();
    TwistState applied;
    {
      std::lock_guard<std::mutex> lock(applied_mutex_);
      applied_twist_ = step_twist(applied_twist_, target, config_, now, last_applied_time_);
      last_applied_time_ = now;
      applied = applied_twist_;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = applied.vx;
    msg.linear.y = applied.vy;
    msg.angular.z = applied.wz;
    command_publisher_->publish(msg);
  }

  void handle_feedback_timer()
  {
    const int fd = master_fd_;
    if (fd < 0) {
      return;
    }

    TwistState applied;
    {
      std::lock_guard<std::mutex> lock(applied_mutex_);
      applied = applied_twist_;
    }

    std::optional<TwistState> odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = odom_state_;
    }

    const auto frame = build_feedback_frame(config_, applied, odom, SteadyClock::now());
    const ssize_t written = ::write(fd, frame.data(), frame.size());
    (void)written;
  }

  EmulatorParams config_{};

  std::string pty_link_path_;
  std::string slave_name_;
  std::string odom_topic_;
  std::string cmd_vel_out_topic_;

  int master_fd_{-1};
  int slave_fd_{-1};

  std::atomic<bool> rx_thread_stop_{false};
  std::thread rx_thread_;

  std::mutex target_mutex_;
  TwistState target_twist_{};

  std::mutex applied_mutex_;
  TwistState applied_twist_{};
  SteadyTimePoint last_applied_time_{};

  std::mutex odom_mutex_;
  std::optional<TwistState> odom_state_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<ugv_controller_emulator::ControllerEmulatorNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ugv_controller_emulator"),
      "failed to start controller emulator: %s",
      exc.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
