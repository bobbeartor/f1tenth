#ifndef LANE_MASK__LANE_MASK_NODE_HPP_
#define LANE_MASK__LANE_MASK_NODE_HPP_

#include <memory>

#include "lane_mask/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lane_mask
{

class LANE_MASK_PUBLIC LaneMaskNode : public rclcpp::Node
{
public:
  explicit LaneMaskNode(const rclcpp::NodeOptions & options);
  ~LaneMaskNode() override;

  LaneMaskNode(const LaneMaskNode &) = delete;
  LaneMaskNode & operator=(const LaneMaskNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lane_mask

#endif  // LANE_MASK__LANE_MASK_NODE_HPP_
