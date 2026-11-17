#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ir_camera_driver
{
// IR sensors already run at 30Hz. At full rate, consume each new capture once;
// do not resample a jittery 30Hz stream with another host-clock 30Hz deadline.
class IrFrameRateGate
{
public:
  bool accept(double captured, double fps)
  {
    if (!std::isfinite(captured) || !std::isfinite(fps) || fps < 0 || fps > 30) {
      throw std::invalid_argument("invalid IR frame rate/time");
    }
    if (fps != fps_) {fps_ = fps; next_ = captured;}
    if (captured <= last_ || fps == 0) {return false;}
    last_ = captured;
    if (fps == 30) {return true;}
    if (captured + 1e-9 < next_) {return false;}
    next_ += 1.0 / fps;
    if (next_ < captured) {next_ = captured + 1.0 / fps;}
    return true;
  }
private:
  double fps_{-1}, next_{0}, last_{-std::numeric_limits<double>::infinity()};
};
}
