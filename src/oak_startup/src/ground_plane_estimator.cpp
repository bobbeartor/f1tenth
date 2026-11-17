#include "oak_startup/ground_plane_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace oak_startup
{

namespace
{

constexpr double kRadiansToDegrees =
  180.0 / 3.141592653589793238462643383279502884;

bool finiteVector(const cv::Vec3d & value)
{
  return
    std::isfinite(value[0]) &&
    std::isfinite(value[1]) &&
    std::isfinite(value[2]);
}

cv::Vec3d normalized(const cv::Vec3d & value)
{
  const double norm = cv::norm(value);
  if (!finiteVector(value) || !std::isfinite(norm) || norm <= 1.0e-12) {
    throw std::invalid_argument("ground-plane vector must be finite/non-zero");
  }
  return value / norm;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::invalid_argument("median requires at least one value");
  }
  const auto middle =
    values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if ((values.size() % 2U) != 0U) {
    return *middle;
  }
  const double upper = *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
}

bool validConfig(const GroundPlaneFitConfig & config)
{
  return
    config.ransac_iterations > 0 &&
    std::isfinite(config.inlier_threshold_m) &&
    config.inlier_threshold_m > 0.0 &&
    config.minimum_inliers >= 3U &&
    std::isfinite(config.minimum_inlier_ratio) &&
    config.minimum_inlier_ratio > 0.0 &&
    config.minimum_inlier_ratio <= 1.0 &&
    std::isfinite(config.maximum_residual_mad_m) &&
    config.maximum_residual_mad_m > 0.0 &&
    std::isfinite(config.maximum_reference_angle_deg) &&
    config.maximum_reference_angle_deg > 0.0 &&
    config.maximum_reference_angle_deg < 90.0 &&
    std::isfinite(config.minimum_height_m) &&
    config.minimum_height_m > 0.0 &&
    std::isfinite(config.maximum_height_m) &&
    config.maximum_height_m > config.minimum_height_m;
}

std::vector<std::uint8_t> classifyInliers(
  const std::vector<cv::Vec3d> & points,
  const cv::Vec3d & normal,
  const double offset,
  const double threshold,
  std::size_t * count,
  double * residual_sum)
{
  std::vector<std::uint8_t> mask(points.size(), 0U);
  std::size_t local_count = 0U;
  double local_residual_sum = 0.0;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const double residual = std::abs(normal.dot(points[index]) + offset);
    if (residual <= threshold) {
      mask[index] = 1U;
      ++local_count;
      local_residual_sum += residual;
    }
  }
  if (count != nullptr) {
    *count = local_count;
  }
  if (residual_sum != nullptr) {
    *residual_sum = local_residual_sum;
  }
  return mask;
}

bool refinePlane(
  const std::vector<cv::Vec3d> & points,
  const std::vector<std::uint8_t> & inlier_mask,
  const cv::Vec3d & reference_up,
  cv::Vec3d * normal,
  double * offset)
{
  cv::Vec3d centroid(0.0, 0.0, 0.0);
  std::size_t count = 0U;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (inlier_mask[index] == 0U) {
      continue;
    }
    centroid += points[index];
    ++count;
  }
  if (count < 3U) {
    return false;
  }
  centroid *= 1.0 / static_cast<double>(count);

  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (inlier_mask[index] == 0U) {
      continue;
    }
    const cv::Vec3d delta = points[index] - centroid;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) += delta[row] * delta[column];
      }
    }
  }
  covariance *= 1.0 / static_cast<double>(count);

  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors)) {
    return false;
  }
  cv::Vec3d refined_normal(
    eigenvectors.at<double>(2, 0),
    eigenvectors.at<double>(2, 1),
    eigenvectors.at<double>(2, 2));
  refined_normal = normalized(refined_normal);
  if (refined_normal.dot(reference_up) < 0.0) {
    refined_normal *= -1.0;
  }

  *normal = refined_normal;
  *offset = -refined_normal.dot(centroid);
  return std::isfinite(*offset);
}

void reject(std::string * rejection, const char * reason)
{
  if (rejection != nullptr) {
    *rejection = reason;
  }
}

}  // namespace

std::optional<GroundPlaneEstimate> fitGroundPlane(
  const std::vector<cv::Vec3d> & points_camera_m,
  const cv::Vec3d & reference_up_camera,
  const GroundPlaneFitConfig & config,
  std::string * rejection)
{
  if (!validConfig(config)) {
    throw std::invalid_argument("invalid ground-plane fit configuration");
  }

  const cv::Vec3d reference_up = normalized(reference_up_camera);
  std::vector<cv::Vec3d> points;
  points.reserve(points_camera_m.size());
  for (const auto & point : points_camera_m) {
    if (finiteVector(point)) {
      points.push_back(point);
    }
  }
  if (points.size() < config.minimum_inliers) {
    reject(rejection, "insufficient finite points for ground-plane fitting");
    return std::nullopt;
  }

  std::mt19937 random_engine(
    static_cast<std::uint32_t>(0x6D2B79F5U ^ points.size()));
  std::uniform_int_distribution<std::size_t> index_distribution(
    0U, points.size() - 1U);

  std::vector<std::uint8_t> best_mask;
  std::size_t best_count = 0U;
  double best_residual_sum = std::numeric_limits<double>::infinity();
  for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    const std::size_t first_index = index_distribution(random_engine);
    const std::size_t second_index = index_distribution(random_engine);
    const std::size_t third_index = index_distribution(random_engine);
    if (
      first_index == second_index ||
      first_index == third_index ||
      second_index == third_index)
    {
      continue;
    }

    const cv::Vec3d first_edge =
      points[second_index] - points[first_index];
    const cv::Vec3d second_edge =
      points[third_index] - points[first_index];
    cv::Vec3d normal = first_edge.cross(second_edge);
    const double normal_norm = cv::norm(normal);
    if (!std::isfinite(normal_norm) || normal_norm <= 1.0e-9) {
      continue;
    }
    normal *= 1.0 / normal_norm;
    if (normal.dot(reference_up) < 0.0) {
      normal *= -1.0;
    }
    if (angleDegrees(normal, reference_up) >
      config.maximum_reference_angle_deg)
    {
      continue;
    }

    const double offset = -normal.dot(points[first_index]);
    if (
      offset < config.minimum_height_m ||
      offset > config.maximum_height_m)
    {
      continue;
    }

    std::size_t inlier_count = 0U;
    double residual_sum = 0.0;
    auto mask = classifyInliers(
      points, normal, offset, config.inlier_threshold_m,
      &inlier_count, &residual_sum);
    if (
      inlier_count > best_count ||
      (inlier_count == best_count && residual_sum < best_residual_sum))
    {
      best_count = inlier_count;
      best_residual_sum = residual_sum;
      best_mask = std::move(mask);
    }
  }

  if (best_count < config.minimum_inliers || best_mask.empty()) {
    reject(rejection, "RANSAC did not find enough ground-plane inliers");
    return std::nullopt;
  }

  cv::Vec3d normal;
  double offset = 0.0;
  if (!refinePlane(points, best_mask, reference_up, &normal, &offset)) {
    reject(rejection, "ground-plane PCA refinement failed");
    return std::nullopt;
  }

  std::size_t refined_count = 0U;
  auto refined_mask = classifyInliers(
    points, normal, offset, config.inlier_threshold_m,
    &refined_count, nullptr);
  if (refined_count < config.minimum_inliers) {
    reject(rejection, "refined ground plane has too few inliers");
    return std::nullopt;
  }
  if (!refinePlane(points, refined_mask, reference_up, &normal, &offset)) {
    reject(rejection, "final ground-plane PCA refinement failed");
    return std::nullopt;
  }

  std::size_t final_count = 0U;
  const auto final_mask = classifyInliers(
    points, normal, offset, config.inlier_threshold_m,
    &final_count, nullptr);
  const double inlier_ratio =
    static_cast<double>(final_count) / static_cast<double>(points.size());
  if (
    final_count < config.minimum_inliers ||
    inlier_ratio < config.minimum_inlier_ratio)
  {
    reject(rejection, "ground-plane inlier ratio is below the limit");
    return std::nullopt;
  }

  std::vector<double> residuals;
  residuals.reserve(final_count);
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (final_mask[index] != 0U) {
      residuals.push_back(std::abs(normal.dot(points[index]) + offset));
    }
  }
  const double residual_mad_m = median(std::move(residuals));
  if (residual_mad_m > config.maximum_residual_mad_m) {
    reject(rejection, "ground-plane residual MAD is above the limit");
    return std::nullopt;
  }

  const double reference_angle_deg = angleDegrees(normal, reference_up);
  if (reference_angle_deg > config.maximum_reference_angle_deg) {
    reject(rejection, "ground-plane normal disagrees with IMU reference");
    return std::nullopt;
  }
  if (
    offset < config.minimum_height_m ||
    offset > config.maximum_height_m)
  {
    reject(rejection, "ground-plane height is outside the allowed range");
    return std::nullopt;
  }

  return GroundPlaneEstimate{
    normal,
    offset,
    std::atan2(-normal[0], -normal[1]),
    std::atan2(-normal[2], std::hypot(normal[0], normal[1])),
    points.size(),
    final_count,
    inlier_ratio,
    residual_mad_m,
    reference_angle_deg};
}

}  // namespace oak_startup
