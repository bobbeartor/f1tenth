#pragma once
#include <chrono>

namespace auto_control
{
// All-model readiness must be refreshed by the startup monitor heartbeat.
class ModelStartupGate
{
public:
  using Clock = std::chrono::steady_clock;
  void observe(bool ready, Clock::time_point received_at)
  {
    ready_ = ready;
    received_at_ = received_at;
    seen_ = true;
  }
  bool ready(Clock::time_point now) const
  {
    return seen_ && ready_ && now >= received_at_ &&
      now - received_at_ <= std::chrono::seconds(3);
  }
private:
  bool seen_{false}, ready_{false};
  Clock::time_point received_at_{};
};
}
