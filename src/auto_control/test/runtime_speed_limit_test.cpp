#include "auto_control/runtime_speed_limit.hpp"
#include <cassert>
#include <limits>
#include <initializer_list>

int main()
{
  auto_control::RuntimeSpeedLimit limit(.8, 5.);
  assert(limit.applied() == 5. && limit.revision() == 0);
  assert(limit.request(3.) && limit.applied() == 3. && limit.revision() == 1);
  assert(limit.request(3.) && limit.revision() == 1);
  for (double invalid : {0., .79, 5.01, -1., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    assert(!limit.request(invalid));
    assert(limit.applied() == 3. && limit.revision() == 1);
  }
  assert(limit.request(.8) && limit.applied() == .8);
  assert(limit.request(5.) && limit.applied() == 5.);
  auto_control::RuntimeSpeedLimit fixed(1.8, 1.8);
  assert(fixed.request(1.8) && !fixed.request(1.75));
  bool threw = false;
  try {auto_control::RuntimeSpeedLimit invalid(2., 1.);} catch (const std::invalid_argument &) {threw = true;}
  assert(threw);
}
