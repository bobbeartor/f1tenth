#include "line_detactor/frame_rate_gate.hpp"
#include <limits>
#include <stdexcept>

void require(bool condition, const char * description)
{
  // Release builds define NDEBUG: checks must not disappear like assert().
  if (!condition) {throw std::runtime_error(description);}
}

int main()
{
  line_detactor::FrameRateGate gate;
  int count=0;
  for (int i=0;i<800;++i) {count+=gate.accept(i/80.0,5.0);}
  require(count==50,"80Hz source must produce 5Hz over ten seconds");
  require(gate.accept(10.,80.),"Rate transition must be immediate");
  require(gate.accept(10.0125,80.),"Next 80Hz frame must be accepted");
  require(!gate.accept(10.013,80.),"Early frame must be rejected");
  require(gate.accept(20.,5.),"First frame after a gap must be accepted");
  require(!gate.accept(20.001,5.),"Gap must not cause a backlog burst");
  require(gate.accept(20.002,0.),"Unlimited mode must accept frames");
  bool rejected=false;
  try {gate.accept(21.,std::numeric_limits<double>::infinity());}
  catch (const std::invalid_argument &) {rejected=true;}
  require(rejected,"Nonfinite rate must be rejected");
}
