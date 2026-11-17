#pragma once
#include <cmath>
#include <stdexcept>

namespace line_detactor
{
// A source-independent monotonic deadline. Never queues or replays frames.
class FrameRateGate
{
public:
  bool accept(double now, double fps)
  {
    if (!std::isfinite(now) || !std::isfinite(fps) || fps<0) {
      throw std::invalid_argument("invalid frame rate/time");
    }
    if (fps==0) {return true;}
    if (fps!=fps_) {fps_=fps; next_=now;}
    if (now+1e-9<next_) {return false;}
    next_+=1.0/fps;
    if (next_<now) {next_=now+1.0/fps;}
    return true;
  }
private:
  double fps_{-1},next_{0};
};
}
