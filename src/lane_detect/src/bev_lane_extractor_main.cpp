#include <exception>
#include <memory>

#include "lane_mask/bev_lane_extractor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node = std::make_shared<lane_mask::BevLaneExtractorNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("bev_lane_extractor"),
      "BEV lane extractor terminated: %s", exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
