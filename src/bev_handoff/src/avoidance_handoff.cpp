#include "bev_handoff/avoidance_handoff.hpp"
#include <atomic>

namespace bev_handoff
{
namespace
{
std::atomic<bool> enabled{false};
std::atomic<bool> control_requested{false};
std::atomic<bool> deformation_only{true};
std::shared_ptr<const PlanningLane> lane;
std::shared_ptr<const AvoidancePreview> plan;
}
bool avoidancePreviewEnabled() {return enabled.load();}
bool avoidanceControlRequested() {return control_requested.load();}
bool avoidanceDeformationOnly() {return deformation_only.load();}
void setAvoidanceDeformationOnly(bool value) {deformation_only.store(value);}
void setAvoidanceControlRequested(bool value) {control_requested.store(value);}
void setAvoidancePreviewEnabled(bool value)
{
  enabled.store(value);
  if (!value) {
    std::atomic_store(&lane, std::shared_ptr<const PlanningLane>{});
    std::atomic_store(&plan, std::shared_ptr<const AvoidancePreview>{});
  }
}
void publishPlanningLane(std::shared_ptr<const PlanningLane> value)
{std::atomic_store(&lane, std::move(value));}
std::shared_ptr<const PlanningLane> latestPlanningLane() {return std::atomic_load(&lane);}
void publishAvoidancePreview(std::shared_ptr<const AvoidancePreview> value)
{std::atomic_store(&plan, std::move(value));}
std::shared_ptr<const AvoidancePreview> latestAvoidancePreview() {return std::atomic_load(&plan);}
}
