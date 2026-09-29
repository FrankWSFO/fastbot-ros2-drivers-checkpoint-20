#include <memory>

#include "motor_control/motor_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{   
  rclcpp::init(argc, argv);

  int result = 0;
  try {
    auto motor_driver = std::make_shared<motor_driver::MotorDriver>();

    // Two threads, matching the Python node's MultiThreadedExecutor(num_threads=2).
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(motor_driver);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_driver"), "Fatal error: %s", e.what());
    result = 1;
  }

  rclcpp::shutdown();
  return result;
}
