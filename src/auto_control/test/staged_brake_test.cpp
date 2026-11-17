#include <cmath>
#include <cstdlib>
#include <iostream>

#include "auto_control/control_core.hpp"

namespace
{
bool expect_current(double actual, double expected, const char * message)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1.0e-10) {
    std::cerr << "FAILED: " << message << ": " << actual << " != " << expected << '\n';
    return false;
  }
  return true;
}
}  // namespace

int main()
{
  using auto_control::staged_brake_current;
  bool passed = true;
  // Default speed gain preserves the previous default Kp=1.0 brake behavior.
  passed &= expect_current(staged_brake_current(0.2, 0.0, 1.0, 0.5, 2.5), 0.5,
    "normal corner overspeed");
  passed &= expect_current(staged_brake_current(0.2, 0.4, 1.0, 0.5, 2.5), 1.0,
    "traffic stop includes missing deceleration");
  passed &= expect_current(staged_brake_current(0.2, 0.4, 0.5, 0.5, 2.5), 0.75,
    "brake speed gain changes only speed contribution");
  passed &= expect_current(staged_brake_current(0.0, 0.4, 2.0, 0.5, 2.5), 0.5,
    "missing deceleration contribution remains independent");
  passed &= expect_current(staged_brake_current(0.1, 0.0, 1.0, 0.5, 2.5), 0.25,
    "zero-target approach preserves proportional braking before hold");
  passed &= expect_current(staged_brake_current(2.0, 3.0, 1.0, 0.5, 2.5), 2.5,
    "combined request respects current cap");
  passed &= expect_current(staged_brake_current(0.0, 0.0, 1.0, 0.5, 2.5), 0.0,
    "no unconditional brake floor");
  passed &= expect_current(staged_brake_current(-0.2, -0.4, 1.0, 0.5, 2.5), 0.0,
    "negative deficits do not create negative brake current");
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
