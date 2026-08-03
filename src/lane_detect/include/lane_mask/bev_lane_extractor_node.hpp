#ifndef LANE_MASK__BEV_LANE_EXTRACTOR_NODE_HPP_
#define LANE_MASK__BEV_LANE_EXTRACTOR_NODE_HPP_

#include <memory>

#include "lane_mask/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lane_mask
{

class LANE_MASK_PUBLIC BevLaneExtractorNode : public rclcpp::Node
{
public:
  explicit BevLaneExtractorNode(const rclcpp::NodeOptions & options);
  ~BevLaneExtractorNode() override;

  BevLaneExtractorNode(const BevLaneExtractorNode &) = delete;
  BevLaneExtractorNode & operator=(const BevLaneExtractorNode &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lane_mask

#endif  // LANE_MASK__BEV_LANE_EXTRACTOR_NODE_HPP_
