#include "motor_control/motor_driver.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace motor_driver
{

namespace
{
speed_t baud_to_speed_t(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
  }
}
}  // namespace

// ---------------------------------------------------------------------------
// SerialPort
// ---------------------------------------------------------------------------

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & port, int baud_rate, double timeout_sec)
{
  timeout_sec_ = timeout_sec;

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    throw std::runtime_error(
            "could not open port " + port + ": " + std::strerror(errno));
  }

  // Switch back to blocking-style descriptor; we manage our own timeout via select().
  fcntl(fd_, F_SETFL, 0);

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    throw std::runtime_error("tcgetattr failed on " + port + ": " + std::strerror(errno));
  }

  speed_t speed = baud_to_speed_t(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no flow control, raw mode (mirrors pyserial's defaults).
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;  // We implement our own timeout via select() in read_until_cr().

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("tcsetattr failed on " + port + ": " + std::strerror(errno));
  }

  // Give the Nano time to finish its DTR-triggered reset after the port opens,
  // same reason pyserial connections often need a brief pause post-open.
  usleep(2000000);
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    throw std::runtime_error("write() called on a closed serial port");
  }
  ssize_t written = ::write(fd_, data.c_str(), data.size());
  if (written < 0 || static_cast<size_t>(written) != data.size()) {
    throw std::runtime_error(std::string("serial write failed: ") + std::strerror(errno));
  }
}

std::string SerialPort::read_until_cr()
{
  std::string result;
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec_);

  char ch;
  while (std::chrono::steady_clock::now() < deadline) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    auto remaining = deadline - std::chrono::steady_clock::now();
    auto remaining_us =
      std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
    if (remaining_us < 0) {remaining_us = 0;}
    timeval tv{};
    tv.tv_sec = remaining_us / 1000000;
    tv.tv_usec = remaining_us % 1000000;

    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(fd_, &read_fds)) {
      ssize_t n = ::read(fd_, &ch, 1);
      if (n == 1) {
        if (ch == '\r') {
          return result;
        }
        result += ch;
      }
    } else {
      break;  // timeout or select() error
    }
  }
  return "";  // timeout: caller treats empty string the same as Python's `None`
}

// ---------------------------------------------------------------------------
// MotorDriver
// ---------------------------------------------------------------------------

MotorDriver::MotorDriver(const rclcpp::NodeOptions & options)
: Node("motor_driver", options)
{
  // Required, non-zero parameters (mirrors the Python node's hard failure on 0).
  this->declare_parameter<int64_t>("encoder_cpr", 0);
  encoder_cpr_ = this->get_parameter("encoder_cpr").as_int();
  if (encoder_cpr_ == 0) {
    RCLCPP_ERROR(this->get_logger(), "Encoder CPR must be set to a non-zero value!");
    throw std::invalid_argument("Encoder CPR is required and must be non-zero.");
  }

  this->declare_parameter<double>("loop_rate", 0.0);
  loop_rate_ = this->get_parameter("loop_rate").as_double();
  if (loop_rate_ == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Loop rate must be set to a non-zero value!");
    throw std::invalid_argument("Loop rate is required and must be non-zero.");
  }

  // Serial parameters
  this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  serial_port_ = this->get_parameter("serial_port").as_string();
  this->declare_parameter<int64_t>("baud_rate", 57600);
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  this->declare_parameter<bool>("serial_debug", false);
  debug_serial_cmds_ = this->get_parameter("serial_debug").as_bool();

  if (debug_serial_cmds_) {
    RCLCPP_INFO(this->get_logger(), "Serial debug enabled");
  }

  // Kinematic parameters
  this->declare_parameter<double>("wheel_diameter", 0.065);
  wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
  this->declare_parameter<double>("wheel_separation", 0.17);
  wheel_separation_ = this->get_parameter("wheel_separation").as_double();
  wheel_radius_ = wheel_diameter_ / 2.0;

  // Replaces the Python node's "-robot_name_value" argparse argument with a
  // standard ROS2 parameter.
  this->declare_parameter<std::string>("robot_name", "fastbot_X");
  robot_name_ = this->get_parameter("robot_name").as_string();

  // Publishers / subscriber, all sharing a reentrant callback group so the
  // timer and the cmd_vel callback can run concurrently, same as the Python node.
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&MotorDriver::cmd_vel_callback, this, std::placeholders::_1),
    sub_options);

  motor_vels_pub_ = this->create_publisher<serial_motor_msgs::msg::MotorVels>("motor_vels", 10);
  encoder_pub_ = this->create_publisher<serial_motor_msgs::msg::EncoderVals>("encoder_vals", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MotorDriver::timer_callback, this),
    callback_group_);

  last_enc_read_time_ = this->now();
  last_time_ = this->now();

  // Serial connection
  try {
    RCLCPP_INFO(
      this->get_logger(), "Connecting to port %s at %ld.",
      serial_port_.c_str(), static_cast<long>(baud_rate_));
    conn_.open(serial_port_, static_cast<int>(baud_rate_), 1.0);
    RCLCPP_INFO(this->get_logger(), "Connected to %s", serial_port_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to connect to %s: %s", serial_port_.c_str(), e.what());
    throw;
  }
}

MotorDriver::~MotorDriver()
{
  if (conn_.is_open()) {
    conn_.close();
    RCLCPP_INFO(this->get_logger(), "Serial connection closed.");
  }
}

void MotorDriver::send_pwm_motor_command(double mot_1_pwm, double mot_2_pwm)
{
  send_command(
    "o " + std::to_string(static_cast<int>(mot_1_pwm)) + " " +
    std::to_string(static_cast<int>(mot_2_pwm)));
}

void MotorDriver::send_feedback_motor_command(double mot_1_ct_per_loop, double mot_2_ct_per_loop)
{
  send_command(
    "m " + std::to_string(static_cast<int>(mot_1_ct_per_loop)) + " " +
    std::to_string(static_cast<int>(mot_2_ct_per_loop)));
}

std::optional<std::vector<int64_t>> MotorDriver::send_encoder_read_command()
{
  auto resp = send_command("e");
  if (resp) {
    std::vector<int64_t> values;
    std::istringstream iss(*resp);
    std::string token;
    try {
      while (iss >> token) {
        values.push_back(std::stoll(token));
      }
      return values;
    } catch (const std::exception &) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse encoder values.");
    }
  }
  return std::nullopt;
}

std::optional<std::string> MotorDriver::send_command(const std::string & cmd_string)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    std::string cmd = cmd_string + "\r";
    conn_.write(cmd);
    if (debug_serial_cmds_) {
      RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.c_str());
    }

    std::string response = conn_.read_until_cr();
    if (response.empty()) {
      RCLCPP_WARN(this->get_logger(), "Serial timeout on command: %s", cmd_string.c_str());
      return std::nullopt;
    }
    if (debug_serial_cmds_) {
      RCLCPP_INFO(this->get_logger(), "Received: %s", response.c_str());
    }
    return response;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
    return std::nullopt;
  }
}

void MotorDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear_vel = msg->linear.x;
  double angular_vel = msg->angular.z;

  double left_wheel_speed = linear_vel - (angular_vel * wheel_separation_) / 2.0;
  double right_wheel_speed = linear_vel + (angular_vel * wheel_separation_) / 2.0;

  if (wheel_radius_ == 0.0) {
    RCLCPP_ERROR(
      this->get_logger(), "Wheel radius is zero, cannot calculate angular velocities.");
    return;
  }

  double left_wheel_rad_per_sec = left_wheel_speed / wheel_radius_;
  double right_wheel_rad_per_sec = right_wheel_speed / wheel_radius_;

  RCLCPP_INFO(
    this->get_logger(),
    "cmd_vel -> Linear: %.3f m/s, Angular: %.3f rad/s | Left wheel: %.2f rad/s, "
    "Right wheel: %.2f rad/s",
    linear_vel, angular_vel, left_wheel_rad_per_sec, right_wheel_rad_per_sec);

  double scaler = (1.0 / (2.0 * M_PI)) * static_cast<double>(encoder_cpr_) * (1.0 / loop_rate_);
  double mot_1_ct_per_loop = left_wheel_rad_per_sec * scaler;
  double mot_2_ct_per_loop = right_wheel_rad_per_sec * scaler;

  if (std::isfinite(mot_1_ct_per_loop) && std::isfinite(mot_2_ct_per_loop)) {
    send_feedback_motor_command(mot_1_ct_per_loop, mot_2_ct_per_loop);
  } else {
    RCLCPP_WARN(this->get_logger(), "Non-finite motor count detected, skipping command send.");
  }
}

void MotorDriver::timer_callback()
{
  check_encoders();
}

void MotorDriver::check_encoders()
{
  auto resp = send_encoder_read_command();
  if (resp && resp->size() >= 2) {
    rclcpp::Time new_time = this->now();
    double time_diff = (new_time - last_enc_read_time_).seconds();
    last_enc_read_time_ = new_time;

    int64_t m1_diff = (*resp)[0] - last_m1_enc_;
    last_m1_enc_ = (*resp)[0];
    int64_t m2_diff = (*resp)[1] - last_m2_enc_;
    last_m2_enc_ = (*resp)[1];

    double rads_per_ct = 2.0 * M_PI / static_cast<double>(encoder_cpr_);
    if (time_diff > 0.0) {
      m1_spd_ = static_cast<double>(m1_diff) * rads_per_ct / time_diff;
      m2_spd_ = static_cast<double>(m2_diff) * rads_per_ct / time_diff;
    }

    serial_motor_msgs::msg::MotorVels spd_msg;
    spd_msg.mot_1_rad_sec = m1_spd_;
    spd_msg.mot_2_rad_sec = m2_spd_;
    motor_vels_pub_->publish(spd_msg);

    serial_motor_msgs::msg::EncoderVals enc_msg;
    enc_msg.mot_1_enc_val = last_m1_enc_;
    enc_msg.mot_2_enc_val = last_m2_enc_;
    encoder_pub_->publish(enc_msg);

    // publish_odometry();  // disabled -- mirrors the commented-out call in the Python node
  }
}

void MotorDriver::publish_odometry()
{
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  if (dt == 0.0) {
    return;
  }

  double left_distance = m1_spd_ * dt * wheel_radius_;
  double right_distance = m2_spd_ * dt * wheel_radius_;

  double avg_distance = (left_distance + right_distance) / 2.0;
  double d_theta = (right_distance - left_distance) / wheel_separation_;

  theta_ = std::fmod(theta_ + d_theta, 2.0 * M_PI);
  if (theta_ < 0) {
    theta_ += 2.0 * M_PI;
  }
  x_ += avg_distance * std::cos(theta_);
  y_ += avg_distance * std::sin(theta_);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = robot_name_ + "_odom";
  odom_msg.child_frame_id = robot_name_ + "_base_link";

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  auto quat = euler_to_quaternion(0.0, 0.0, theta_);
  odom_msg.pose.pose.orientation.x = quat[0];
  odom_msg.pose.pose.orientation.y = quat[1];
  odom_msg.pose.pose.orientation.z = quat[2];
  odom_msg.pose.pose.orientation.w = quat[3];

  odom_msg.twist.twist.linear.x = avg_distance / dt;
  odom_msg.twist.twist.angular.z = d_theta / dt;

  odom_pub_->publish(odom_msg);
}

std::array<double, 4> MotorDriver::euler_to_quaternion(double roll, double pitch, double yaw)
{
  double qx = std::sin(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) -
    std::cos(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  double qy = std::cos(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2) +
    std::sin(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2);
  double qz = std::cos(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2) -
    std::sin(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2);
  double qw = std::cos(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) +
    std::sin(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  return {qx, qy, qz, qw};
}

}  // namespace motor_driver
