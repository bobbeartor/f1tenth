#pragma once
#include <cmath>

namespace multiclass_yolo {
// Visibility is unsupported by some GTK backends. AUTOSIZE is also a
// window-presence probe: zero is a valid value for WINDOW_NORMAL.
class PreviewExit {
 public:
  const char * reason(int key, double visible, double autosize) {
    if (key == 27 || key == 'q' || key == 'Q') return "Q/ESC pressed";
    const bool visible_valid = std::isfinite(visible) && visible >= 0;
    const bool presence_valid = std::isfinite(autosize) && autosize >= 0;
    if (visible_valid && visible >= 1) seen_visible_ = true;
    if (presence_valid) seen_presence_ = true;
    if ((seen_visible_ && (!visible_valid || visible < 1)) ||
        (seen_presence_ && !presence_valid)) return "preview window closed";
    return nullptr;
  }
 private:
  bool seen_visible_{false};
  bool seen_presence_{false};
};
}  // namespace multiclass_yolo
