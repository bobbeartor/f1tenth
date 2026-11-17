#include "camera_driver/rgb_exposure_recovery.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace std::chrono_literals;
using Policy=camera_driver::RgbExposureRecovery;
using Phase=camera_driver::RgbTunnelPhase;
using Command=Policy::Command;
using Time=Policy::Clock::time_point;
void require(bool value,const char * why) {if (!value) {throw std::runtime_error(why);}}
Time at(int milliseconds) {return Time{}+std::chrono::milliseconds(milliseconds);}
Policy::Sample sample(int milliseconds,long exposure=2000,int iso=200)
{return {at(milliseconds),exposure,iso};}
void prepare(Policy & p)
{
  for (int i=1000;i<=1800;i+=20) {
    const auto action=p.step(Phase::outside,at(i+20),sample(i));
    require(action.command==Command::none,"outside must remain AE");
  }
  // Dark entrance in the last 200ms must not overwrite the earlier stable AE.
  for (int i=1820;i<=1980;i+=20) {p.step(Phase::outside,at(i+10),sample(i,12000,800));}
  p.step(Phase::entering,at(2000),sample(1990,12000,800));
  require(p.saved() && p.saved()->exposure_us==2000 && p.saved()->iso==200,"freeze stable exterior metadata before entry");
  p.step(Phase::inside,at(2200),sample(2180,12000,800));
  require(p.saved() && p.saved()->exposure_us==2000,"INSIDE cannot overwrite cached exterior exposure");
}
int main()
{
  Policy p; prepare(p);
  const auto action=p.step(Phase::exiting,at(3000),sample(2990,12000,800));
  require(action.command==Command::manual && action.exposure_us==2000 && action.iso==200,"exit should issue one lowering pulse");
  require(p.step(Phase::exiting,at(3002),sample(2990,2000,200)).command==Command::none,"old capture cannot confirm application");
  require(p.step(Phase::exiting,at(3020),sample(3010)).command==Command::none,"first matching frame retains short pulse");
  require(p.step(Phase::exiting,at(3022),sample(3010)).command==Command::none,"duplicate cannot count twice");
  require(p.step(Phase::exiting,at(3040),sample(3030)).command==Command::automatic,"two new matched frames must return to AE");
  require(p.step(Phase::exiting,at(3042),std::nullopt).command==Command::automatic,"AE send must remain pending until acknowledged");
  p.autoSent();
  require(!p.restoring(),"AE success clears pulse");
  require(p.step(Phase::exiting,at(3050),sample(3040,3000,300)).command==Command::none,"same EXITING must not restart manual exposure");
  require(p.step(Phase::outside,at(3060),sample(3050,4000,300)).command==Command::none,"normal outside auto exposure is untouched");

  Policy no_frames; prepare(no_frames);
  no_frames.step(Phase::exiting,at(3000),sample(2990,12000,800));
  require(no_frames.step(Phase::exiting,at(3199),std::nullopt).command==Command::none,"no forced hold before maximum");
  require(no_frames.step(Phase::exiting,at(3200),std::nullopt).command==Command::automatic,"capture loss cannot keep manual exposure locked");

  Policy cancelled; prepare(cancelled);
  cancelled.step(Phase::exiting,at(3000),sample(2990,12000,800));
  require(cancelled.step(Phase::outside,at(3001),std::nullopt).command==Command::automatic,"early RGB readiness restores AE immediately");
  Policy reverse; prepare(reverse);
  reverse.step(Phase::exiting,at(3000),sample(2990,12000,800));
  require(reverse.step(Phase::entering,at(3001),std::nullopt).command==Command::automatic,"reverse entry must release manual pulse");

  Policy failed; prepare(failed);
  failed.step(Phase::exiting,at(3000),sample(2990,12000,800));
  failed.requestAuto();
  require(failed.step(Phase::exiting,at(3002),std::nullopt).command==Command::automatic,"uncertain manual send must recover through AE");
  failed.autoSent();
  require(failed.step(Phase::exiting,at(3004),std::nullopt).command==Command::none,"failed pulse must not be retried");

  Policy missing;
  require(missing.step(Phase::inside,at(2000),sample(1990,12000,800)).command==Command::none,"inside stays AE");
  require(missing.step(Phase::exiting,at(3000),sample(2990,12000,800)).command==Command::none,"no exterior history means no manual override");
  Policy dark; prepare(dark);
  require(dark.step(Phase::exiting,at(3000),sample(2990,1000,100)).command==Command::none,"never brighten RGB that's already recovered");
  Policy stale; prepare(stale);
  require(stale.step(Phase::exiting,at(3000),sample(2200,12000,800)).command==Command::none,"stale actual metadata cannot trigger pulse");

  Policy unstable;
  for (int i=1000;i<1990;i+=20) {
    unstable.step(Phase::outside,at(i+5),sample(i,(i/20)%2 ? 2000 : 8000));
  }
  unstable.step(Phase::entering,at(2000),std::nullopt);
  require(!unstable.saved(),"unstable outside exposure must not become a baseline");
  require(unstable.step(Phase::exiting,at(3000),sample(2990,12000,800)).command==Command::none,"uncertain baseline keeps AE");

  // A second visit must learn the new outside lighting after AE resumes.
  p.step(Phase::outside,at(4000),sample(3990,1000,100));
  for (int i=4020;i<5000;i+=20) {p.step(Phase::outside,at(i+5),sample(i,1000,100));}
  p.step(Phase::entering,at(5000),std::nullopt);
  require(p.saved() && p.saved()->exposure_us==1000 && p.saved()->iso==100,"next tunnel must use a new exterior snapshot");
  require(p.step(Phase::exiting,at(6000),sample(5990,9000,800)).exposure_us==1000,"second exit restores current exterior lighting");

  Policy changed; prepare(changed);
  changed.step(Phase::exiting,at(3000),sample(2990,12000,800));
  changed.step(Phase::exiting,at(3010),sample(3005));
  changed.step(Phase::exiting,at(3020),sample(3015,9000,800));
  require(changed.step(Phase::exiting,at(3030),sample(3025)).command==Command::none,"matching confirmation must be consecutive");
  require(changed.step(Phase::exiting,at(3040),sample(3035)).command==Command::automatic,"valid consecutive confirmation returns to AE");

  for (double value : {0.,-.1,.51,std::numeric_limits<double>::quiet_NaN()}) {
    bool invalid=false;
    try {Policy invalid_policy(value);} catch (const std::invalid_argument &) {invalid=true;}
    require(invalid,"invalid hold must be rejected");
  }
  std::cout << "rgb_exposure_recovery_test passed\n";
}
