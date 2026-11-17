#ifndef LINE_DETACTOR__SPEED_LIMIT_PANEL_HPP_
#define LINE_DETACTOR__SPEED_LIMIT_PANEL_HPP_
#include <cstdint>
#include <optional>
#include <string>
#include <opencv2/core.hpp>

namespace line_detactor
{
struct SpeedLimitSnapshot
{
  double minimum{0.0}, ceiling{0.0}, applied{0.0};
  std::uint64_t revision{0U};
  bool valid() const;
};

// All methods run on the preview thread, including the HighGUI callback.
// ROS callbacks only supply a mutex-protected snapshot to update().
class SpeedLimitPanel
{
public:
  void update(const SpeedLimitSnapshot & state, bool fresh, double now);
  cv::Mat append(const cv::Mat & bev, bool themed);
  std::string take_action(){auto action=action_;action_.clear();return action;}
  void mouse(int event, int x, int y, int flags);
  std::optional<double> take_request(double now);
  static void mouse_callback(int event, int x, int y, int flags, void * userdata);
  std::string display_key() const;
  double selected() const {return selected_;}
  bool available() const {return available_;}
private:
  double value_at(int y) const;
  void select(int y);
  SpeedLimitSnapshot state_;
  bool available_{false}, initialized_{false}, dragging_{false};
  bool dirty_{false}, pending_{false}, released_{false};
  double selected_{0.0}, now_{0.0}, last_sent_{-100.0}, failed_until_{-1.0};
  cv::Rect hit_;
  cv::Rect metrics_button_,parameters_button_,stop_button_;
  std::string action_;
  int track_top_{0}, track_bottom_{0}, track_x_{0};
};
}  // namespace line_detactor
#endif
