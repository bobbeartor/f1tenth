#pragma once
// Compatibility API; implementation shared with depth_lidar.
#include "oak_startup/startup_attitude.hpp"
namespace bev_processor
{
using oak_startup::StartupAttitudeSource;
using oak_startup::StartupAttitudeSelection;
using oak_startup::parseStartupAttitudeSource;
using oak_startup::startupAttitudeSourceName;
using oak_startup::attitudeUpVector;
using oak_startup::selectStartupAttitude;
}
