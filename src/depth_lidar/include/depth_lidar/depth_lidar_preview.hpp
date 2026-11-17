#pragma once

#include "depth_lidar/depth_lidar_geometry.hpp"
#include "depth_lidar/depth_lidar_grid.hpp"

#include <opencv2/core.hpp>
#include <string>

namespace depth_lidar
{

// Fixed vehicle-frame scan points (before grid filtering) and/or cluster outlines.
// mode: points, clusters, both. Blue points / orange outlines; gray = held.
// Leaves the bottom 40 pixels available for receiver/processing metrics.
cv::Mat makeRadarPreview(const ScanResult & scan, const GridResult & detection, const ProjectionConfig & config, int width_px, bool ready = true,
  const std::string & mode = "both");

// Draw the actual depth ROI scaled into rectified left/right images. Right is
// the depth reference; the left rectangle is a guide, not a correspondence.
cv::Mat makeStereoPreview(const cv::Mat & left, const cv::Mat & right,
  const RoiRect & depth_roi, int depth_width, int depth_height);

} // namespace depth_lidar
