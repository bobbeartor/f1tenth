#pragma once
#include <array>
#include <cstdint>
#include <string_view>

namespace auto_control
{
// Original capture timestamps are ordered within a camera, not across cameras
// with different exposure/transport delays. Capture-age checks remain separate.
class LaneCaptureOrder
{
public:
  bool accept(std::int64_t stamp, std::string_view source = {})
  {
    std::size_t index = 0;
    if (source == "RGB") {index = 1;}
    else if (source == "IR") {index = 2;}
    else if (!source.empty()) {return false;}
    if (stamp <= last_[index]) {return false;}
    last_[index] = stamp;
    return true;
  }
private:
  std::array<std::int64_t, 3> last_{};
};
}
