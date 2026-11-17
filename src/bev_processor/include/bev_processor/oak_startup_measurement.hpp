#pragma once
#include "bev_processor/startup_attitude.hpp"
// Compatibility API; implementation shared with depth_lidar.
#include "oak_startup/oak_startup_measurement.hpp"
namespace bev_processor
{
using oak_startup::OakStartupMeasurementConfig;
using oak_startup::OakStartupMeasurement;
using oak_startup::measureOakStartupExtrinsics;
}
