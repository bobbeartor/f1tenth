#ifndef LINE_DETACTOR__LINE_DETACTOR_NODE_HPP_
#define LINE_DETACTOR__LINE_DETACTOR_NODE_HPP_

#include <memory>

#include "line_detactor/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace line_detactor
{

class LINE_DETACTOR_PUBLIC LineDetactorNode : public rclcpp::Node
{
public:
  explicit LineDetactorNode(const rclcpp::NodeOptions & options);
  ~LineDetactorNode() override;

  LineDetactorNode(const LineDetactorNode &) = delete;
  LineDetactorNode & operator=(const LineDetactorNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace line_detactor

#endif  // LINE_DETACTOR__LINE_DETACTOR_NODE_HPP_
