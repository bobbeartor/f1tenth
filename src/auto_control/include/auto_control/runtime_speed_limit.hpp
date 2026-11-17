#ifndef AUTO_CONTROL__RUNTIME_SPEED_LIMIT_HPP_
#define AUTO_CONTROL__RUNTIME_SPEED_LIMIT_HPP_
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace auto_control
{
// Configured speed range is immutable for this run; UI only selects within it.
class RuntimeSpeedLimit
{
public:
  RuntimeSpeedLimit(double minimum, double ceiling)
  : minimum_(minimum), ceiling_(ceiling), applied_(ceiling)
  {
    if (!std::isfinite(minimum) || !std::isfinite(ceiling) || minimum <= 0.0 || ceiling < minimum) {
      throw std::invalid_argument("Invalid runtime speed range");
    }
  }
  bool request(double speed)
  {
    if (!std::isfinite(speed) || speed < minimum_ || speed > ceiling_) {return false;}
    if (speed != applied_) {applied_ = speed; ++revision_;}
    return true;
  }
  double minimum() const {return minimum_;}
  double ceiling() const {return ceiling_;}
  double applied() const {return applied_;}
  std::uint64_t revision() const {return revision_;}
private:
  double minimum_, ceiling_, applied_;
  std::uint64_t revision_{0U};
};
}  // namespace auto_control
#endif
