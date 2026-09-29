#ifndef MOTOR_DRIVER__MOTOR_DRIVER_HPP_
#define MOTOR_DRIVER__MOTOR_DRIVER_HPP_

#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "serial_motor_msgs/msg/motor_vels.hpp"
#include "serial_motor_msgs/msg/encoder_vals.hpp"

namespace motor_driver
{

/// Minimal POSIX serial port wrapper. Replaces pyserial's Serial class --
/// no external serial library dependency required.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  /// Opens the given port at the given baud rate.
  /// Throws std::runtime_error on failure (mirrors pyserial raising SerialException).
  void open(const std::string & port, int baud_rate, double timeout_sec);

  bool is_open() const {return fd_ >= 0;}

  void close();

  /// Writes a string to the serial port. Throws std::runtime_error on failure.
  void write(const std::string & data);

  /// Reads bytes until '\r' is seen or the configured timeout elapses.
  /// Returns the response with the trailing '\r' stripped, or "" on timeout
  /// (mirrors the Python node's `response = conn.read_until(b'\r')...` behavior).
  std::string read_until_cr();

private:
  int fd_ = -1;
  double timeout_sec_ = 1.0;
};

class MotorDriver : public rclcpp::Node
{
public:
  explicit MotorDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotorDriver() override;

private:
  // --- Serial command helpers ------------------------------------------
  void send_pwm_motor_command(double mot_1_pwm, double mot_2_pwm);
  void send_feedback_motor_command(double mot_1_ct_per_loop, double mot_2_ct_per_loop);
  std::optional<std::vector<int64_t>> send_encoder_read_command();
  std::optional<std::string> send_command(const std::string & cmd_string);

  // --- ROS callbacks -----------------------------------------------------
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();
  void check_encoders();
  void publish_odometry();

  // --- Utility -------------------------------------------------------
  static std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw);

  // Parameters
  int64_t encoder_cpr_ = 0;
  int loop_rate_ = 0.0;
  std::string serial_port_;
  int64_t baud_rate_ = 57600;
  bool debug_serial_cmds_ = false;
  double wheel_diameter_ = 0.065;
  double wheel_separation_ = 0.17;
  double wheel_radius_ = 0.0;
  // ROS2-parameter replacement for the Python node's "-robot_name_value" argparse flag.
  std::string robot_name_ = "fastbot_X";

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<serial_motor_msgs::msg::MotorVels>::SharedPtr motor_vels_pub_;
  rclcpp::Publisher<serial_motor_msgs::msg::EncoderVals>::SharedPtr encoder_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Serial connection
  SerialPort conn_;
  std::mutex serial_mutex_;

  // Encoder / speed tracking
  rclcpp::Time last_enc_read_time_;
  int64_t last_m1_enc_ = 0;
  int64_t last_m2_enc_ = 0;
  double m1_spd_ = 0.0;
  double m2_spd_ = 0.0;

  // Odometry state
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  rclcpp::Time last_time_;
};

}  // namespace motor_driver

#endif  // MOTOR_DRIVER__MOTOR_DRIVER_HPP_

