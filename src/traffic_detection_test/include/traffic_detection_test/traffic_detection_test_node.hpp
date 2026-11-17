#ifndef TRAFFIC_DETECTION_TEST__TRAFFIC_DETECTION_TEST_NODE_HPP_
#define TRAFFIC_DETECTION_TEST__TRAFFIC_DETECTION_TEST_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "traffic_detection_test/visibility_control.hpp"

namespace traffic_detection_test
{

class TRAFFIC_DETECTION_TEST_PUBLIC TrafficDetectionTestNode
  : public rclcpp::Node
{
public:
  explicit TrafficDetectionTestNode(const rclcpp::NodeOptions & options);
  ~TrafficDetectionTestNode() override;

  TrafficDetectionTestNode(const TrafficDetectionTestNode &) = delete;
  TrafficDetectionTestNode & operator=(const TrafficDetectionTestNode &) =
    delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace traffic_detection_test

#endif  // TRAFFIC_DETECTION_TEST__TRAFFIC_DETECTION_TEST_NODE_HPP_
