#include "auto_control/obstacle_worker.hpp"
#include "auto_control/avoidance_worker.hpp"
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace auto_control
{
// Detection belongs to auto_control, but runs separately from its control loop.
// Loaded in the existing camera/BEV container to share depth without serialization.
class ObstacleDetectorNode final : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options)
  : Node("auto_obstacles",options)
  {
    avoidance_=std::make_unique<AvoidanceWorker>(*this);
    // Component load order need not block on the one-shot BEV measurement.
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {connect();});
  }
private:
  void connect()
  {
    if (worker_) {return;}
    const auto reference=bev_handoff::obstacleReference();
    if (!reference) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for shared BEV startup calibration");
      return;
    }
    bev_processor::RectifiedCameraModel camera{reference->fx,reference->fy,reference->cx,reference->cy,
      reference->source_width,reference->source_height,reference->position_vehicle_m,
      reference->rotation_vehicle_from_camera};
    bev_processor::BevConfig bev{reference->x_min,reference->x_max,reference->y_min,reference->y_max,
      reference->meter_per_pixel,reference->width,reference->height};
    worker_=std::make_unique<ObstacleWorker>(*this,camera,bev,reference->frame_id);
    subscription_=create_subscription<camera_driver::msg::BevInput>(reference->input_topic,
      rclcpp::SensorDataQoS().keep_last(1),[this,reference](camera_driver::msg::BevInput::ConstSharedPtr input) {
        if (input->source_width!=static_cast<unsigned>(reference->source_width) ||
          input->source_height!=static_cast<unsigned>(reference->source_height)) {return;}
        const auto now=std::chrono::steady_clock::now();
        if (first_input_==std::chrono::steady_clock::time_point{}) {first_input_=now;}
        if (std::chrono::duration<double>(now-first_input_).count()<reference->settle_sec) {return;}
        worker_->observe(*input);
      });
    timer_->cancel();
    RCLCPP_INFO(get_logger(),"auto_control obstacle detection connected; output=/auto/obstacles; BEV=%.2fx%.2fm",
      reference->y_max-reference->y_min,reference->x_max-reference->x_min);
  }
  std::unique_ptr<ObstacleWorker> worker_;
  std::unique_ptr<AvoidanceWorker> avoidance_;
  rclcpp::Subscription<camera_driver::msg::BevInput>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point first_input_;
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(auto_control::ObstacleDetectorNode)
