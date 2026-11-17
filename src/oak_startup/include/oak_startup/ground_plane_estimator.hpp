#ifndef OAK_STARTUP__GROUND_PLANE_ESTIMATOR_HPP_
#define OAK_STARTUP__GROUND_PLANE_ESTIMATOR_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace oak_startup
{

struct GroundPlaneFitConfig
{
  int ransac_iterations{200};
  double inlier_threshold_m{0.008};
  std::size_t minimum_inliers{1000U};
  double minimum_inlier_ratio{0.70};
  double maximum_residual_mad_m{0.005};
  double maximum_reference_angle_deg{15.0};
  double minimum_height_m{0.10};
  double maximum_height_m{1.00};
};

struct GroundPlaneEstimate
{
  cv::Vec3d up_camera{0.0, -1.0, 0.0};
  double height_m{0.0};
  double roll_rad{0.0};
  double pitch_down_rad{0.0};
  std::size_t point_count{0U};
  std::size_t inlier_count{0U};
  double inlier_ratio{0.0};
  double residual_mad_m{0.0};
  double reference_angle_deg{0.0};
};

std::optional<GroundPlaneEstimate> fitGroundPlane(
  const std::vector<cv::Vec3d> & points_camera_m,
  const cv::Vec3d & reference_up_camera,
  const GroundPlaneFitConfig & config,
  std::string * rejection = nullptr);

}  // namespace oak_startup

#endif  // OAK_STARTUP__GROUND_PLANE_ESTIMATOR_HPP_
