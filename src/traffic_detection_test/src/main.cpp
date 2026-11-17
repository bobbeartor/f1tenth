#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "traffic_detection_test/traffic_detection_test_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node =
      std::make_shared<traffic_detection_test::TrafficDetectionTestNode>(
      options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("traffic_detection_test"),
      "Traffic detection preview terminated: %s", exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
