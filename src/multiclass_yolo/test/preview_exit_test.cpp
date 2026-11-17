#include "multiclass_yolo/preview_exit.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>
void check(bool value) { if (!value) throw std::runtime_error("Preview exit regression"); }
int main() {
  using multiclass_yolo::PreviewExit;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  PreviewExit gtk;
  for (int i=0; i<100; ++i) check(gtk.reason(-1,-1,0)==nullptr);
  check(gtk.reason(-1,-1,-1)!=nullptr); // GTK window was present, now missing
  PreviewExit starting;
  check(starting.reason(-1,-1,-1)==nullptr); // not mapped / unsupported
  check(starting.reason(-1,0,-1)==nullptr); // not yet visible
  check(starting.reason(-1,1,0)==nullptr);
  check(starting.reason(-1,0,0)!=nullptr); // visible, then closed/hidden
  PreviewExit unsupported;
  check(unsupported.reason(-1,nan,nan)==nullptr);
  check(unsupported.reason(-1,-1,-1)==nullptr);
  check(unsupported.reason('q',-1,-1)!=nullptr);
  check(unsupported.reason('Q',nan,nan)!=nullptr);
  check(unsupported.reason(27,-1,-1)!=nullptr);
  PreviewExit normal;
  check(normal.reason(-1,1,0)==nullptr);
  check(normal.reason(-1,1,0)==nullptr);
  check(normal.reason(-1,-1,-1)!=nullptr);
  std::cout << "GTK startup, unsupported properties, close and keyboard tests passed\n";
}
