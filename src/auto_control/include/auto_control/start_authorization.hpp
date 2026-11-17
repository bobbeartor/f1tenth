#pragma once
#include <string>
namespace auto_control
{
// One explicit operator decision per controller/critical-readiness session.
// A false critical input revokes approval; restoring it cannot start the car.
class StartAuthorization
{
public:
  void observe(bool critical_ready, bool manual, const std::string & reason = "")
  {
    if (!critical_ready || manual) {
      if (approved_) {last_reason_=manual ? "Manual mode" :
        (reason.empty() ? "Critical readiness lost" : reason);}
      approved_=false;
    }
    critical_ready_=critical_ready; manual_=manual;
  }
  bool request(bool all_checks_ready)
  {
    if (!critical_ready_ || manual_ || !all_checks_ready) {return false;}
    approved_=true; last_reason_.clear(); return true;
  }
  void revoke(const std::string & reason = "Critical readiness lost") {
    if (approved_) {last_reason_=reason;}
    approved_=false;
  }
  const std::string & last_reason() const {return last_reason_;}
  bool approved() const {return approved_;}
private:
  std::string last_reason_;
  bool approved_{false}, critical_ready_{false}, manual_{false};
};
}
