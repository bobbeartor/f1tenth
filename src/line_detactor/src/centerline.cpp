#include "line_detactor/centerline.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{
using Point = cv::Point2d;
using Path = std::vector<Point>;
constexpr double pi = 3.14159265358979323846;
double norm(const Point & p) {return std::hypot(p.x, p.y);}
Point unit(const Point & p) {return p * (1.0 / std::max(norm(p), 1.0e-9));}
double radians(double degrees) {return degrees * pi / 180.0;}

std::vector<double> arc_lengths(const Path & p)
{
  std::vector<double> arc(p.size(), 0.0);
  for (std::size_t i = 1; i < p.size(); ++i) {arc[i] = arc[i - 1] + norm(p[i] - p[i - 1]);}
  return arc;
}
Path resample(const Path & p, double step)
{
  if (p.size() < 2U) {return p;}
  const auto arc = arc_lengths(p);
  const int count = std::max(2, static_cast<int>(std::ceil(arc.back() / step)) + 1);
  Path out;
  std::size_t j = 1U;
  for (int i = 0; i < count; ++i) {
    const double s = arc.back() * i / (count - 1);
    while (j + 1U < p.size() && arc[j] < s) {++j;}
    const double t = (s - arc[j - 1]) / std::max(arc[j] - arc[j - 1], 1.0e-9);
    out.push_back(p[j - 1] + (p[j] - p[j - 1]) * t);
  }
  return out;
}
Path gaussian(const Path & p, double sigma_samples)
{
  if (sigma_samples <= 0.0 || p.size() < 2U) {return p;}
  Path out(p.size());
  const int radius = std::min(static_cast<int>(p.size()),
    static_cast<int>(std::ceil(3.0 * sigma_samples)));
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    double total = 0.0;
    for (int d = -radius; d <= radius; ++d) {
      const int j = std::clamp(i + d, 0, static_cast<int>(p.size()) - 1);
      const double w = std::exp(-0.5 * d * d / (sigma_samples * sigma_samples));
      out[i] += p[j] * w;
      total += w;
    }
    out[i] *= 1.0 / total;
  }
  return out;
}
Path tangents(const Path & p, int half_window)
{
  Path result;
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    result.push_back(unit(p[std::min(i + half_window, static_cast<int>(p.size()) - 1)] -
      p[std::max(0, i - half_window)]));
  }
  return result;
}

// Metric spatial bins keep clearance and graph-neighbor queries local.
class SpatialIndex
{
public:
  SpatialIndex(const Path & points, double cell) : points_(points), cell_(cell)
  {
    for (int i = 0; i < static_cast<int>(points_.size()); ++i) {bins_[key(points_[i])].push_back(i);}
  }
  std::vector<int> nearby(const Point & p, double radius) const
  {
    std::vector<int> ids;
    const auto lo = key(p - Point(radius, radius));
    const auto hi = key(p + Point(radius, radius));
    for (int y = lo.second; y <= hi.second; ++y) {
      for (int x = lo.first; x <= hi.first; ++x) {
        const auto found = bins_.find({x, y});
        if (found == bins_.end()) {continue;}
        for (int id : found->second) {
          if (norm(points_[id] - p) <= radius) {ids.push_back(id);}
        }
      }
    }
    return ids;
  }
  bool clear(const Point & p, double radius, ProcessingProfile * profile) const
  {
    if (profile) {profile->add(ProfileCounter::clearance_queries);}
    const auto lo = key(p - Point(radius, radius));
    const auto hi = key(p + Point(radius, radius));
    for (int y = lo.second; y <= hi.second; ++y) {
      for (int x = lo.first; x <= hi.first; ++x) {
        const auto found = bins_.find({x, y});
        if (found == bins_.end()) {continue;}
        for (int id : found->second) {
          if (profile) {profile->add(ProfileCounter::clearance_pixel_tests);}
          if (norm(points_[id] - p) <= radius) {return false;}
        }
      }
    }
    return true;
  }
private:
  std::pair<int, int> key(const Point & p) const
  {
    return {static_cast<int>(std::floor(p.x / cell_)), static_cast<int>(std::floor(p.y / cell_))};
  }
  const Path & points_;
  double cell_;
  std::map<std::pair<int, int>, std::vector<int>> bins_;
};

struct Geometry
{
  double width;
  double height;
  double margin;
  double clearance;
  double check_step;
  const SpatialIndex & boundary;
  ProcessingProfile * profile;
  bool point_ok(const Point & p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= -margin &&
           p.x <= width + margin && p.y >= 0.0 && p.y < height && boundary.clear(p, clearance, profile);
  }
  bool segment_ok(const Point & a, const Point & b) const
  {
    if (profile) {profile->add(ProfileCounter::segment_checks);}
    const int steps = std::max(1, static_cast<int>(std::ceil(norm(b - a) / check_step)));
    for (int i = 0; i <= steps; ++i) {
      if (!point_ok(a + (b - a) * (static_cast<double>(i) / steps))) {return false;}
    }
    return true;
  }
  bool path_ok(const Path & p) const
  {
    if (profile) {profile->add(ProfileCounter::path_checks);}
    for (std::size_t i = 1; i < p.size(); ++i) {
      if (!segment_ok(p[i - 1], p[i])) {return false;}
    }
    return p.size() >= 2U;
  }
};

Path bounded_smooth(const Path & original, const Path & proposal, double max_shift,
  const Geometry & geometry)
{
  double largest = 0.0;
  for (std::size_t i = 0; i < original.size(); ++i) {
    largest = std::max(largest, norm(proposal[i] - original[i]));
  }
  double factor = std::min(1.0, max_shift / std::max(largest, 1.0e-9));
  for (int attempt = 0; attempt < 9; ++attempt, factor *= 0.5) {
    Path out = original;
    for (std::size_t i = 1; i + 1U < out.size(); ++i) {
      out[i] += (proposal[i] - original[i]) * factor;
    }
    bool forward = true;
    for (std::size_t i = 1; i < out.size(); ++i) {
      if ((out[i] - out[i - 1]).dot(original[i] - original[i - 1]) <= 0.0) {forward = false;}
    }
    if (forward && geometry.path_ok(out)) {return out;}
  }
  return original;
}

// Local quadratic least squares on metric arc coordinates: Savitzky-Golay style
// smoothing without adding Python/SciPy to the ROS runtime.
Path quadratic(const Path & p, double window_m)
{
  const auto arc = arc_lengths(p);
  Path out = p;
  const double half = window_m / 2.0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const double middle = std::clamp(arc[i], std::min(half, arc.back() / 2.0),
      std::max(arc.back() - half, arc.back() / 2.0));
    cv::Matx33d a = cv::Matx33d::zeros();
    cv::Vec3d bx(0, 0, 0), by(0, 0, 0);
    int count = 0;
    for (std::size_t j = 0; j < p.size(); ++j) {
      if (std::abs(arc[j] - middle) > half) {continue;}
      const double t = (arc[j] - arc[i]) / half;
      const cv::Vec3d row(1.0, t, t * t);
      for (int r = 0; r < 3; ++r) {
        bx[r] += row[r] * p[j].x;
        by[r] += row[r] * p[j].y;
        for (int c = 0; c < 3; ++c) {a(r, c) += row[r] * row[c];}
      }
      ++count;
    }
    cv::Vec3d x, y;
    if (count >= 5 && cv::solve(a, bx, x, cv::DECOMP_SVD) && cv::solve(a, by, y, cv::DECOMP_SVD)) {
      out[i] = Point(x[0], y[0]);
    }
  }
  return out;
}
Path smooth_centerline(const Path & path, const Geometry & geometry, const CenterlineConfig & cfg)
{
  if (!cfg.smoothing_enabled || cfg.smoothing_strength == 0.0 || path.size() < 7U) {return path;}
  const double ds = arc_lengths(path).back() / (path.size() - 1U);
  auto local = gaussian(path, cfg.smoothing_sigma_m / ds);
  for (std::size_t i = 0; i < path.size(); ++i) {
    local[i] = path[i] + (local[i] - path[i]) * cfg.smoothing_strength;
  }
  auto base = bounded_smooth(path, local, cfg.smoothing_max_shift_m, geometry);
  auto broad = quadratic(base, cfg.smoothing_window_m);
  const auto directions = tangents(broad, 1);
  const int span = std::max(1, static_cast<int>(std::round(cfg.turn_window_m / (2.0 * ds))));
  Path weights(base.size());
  for (int i = 0; i < static_cast<int>(base.size()); ++i) {
    const auto a = directions[std::max(0, i - span)];
    const auto b = directions[std::min(static_cast<int>(base.size()) - 1, i + span)];
    const double turn = std::acos(std::clamp(a.dot(b), -1.0, 1.0));
    weights[i].x = 1.0 - std::clamp((turn - radians(cfg.straight_turn_deg)) /
      radians(cfg.corner_turn_deg - cfg.straight_turn_deg), 0.0, 1.0);
  }
  weights = gaussian(weights, 0.05 / ds);
  for (std::size_t i = 0; i < base.size(); ++i) {
    const double taper = std::min(1.0, std::min(i, base.size() - 1U - i) * ds / 0.12);
    broad[i] = base[i] + (broad[i] - base[i]) * (weights[i].x * taper * cfg.smoothing_strength);
  }
  return bounded_smooth(base, broad, cfg.smoothing_max_shift_m, geometry);
}

struct Fragment
{
  int side;
  Path points;
  Path directions;
  std::vector<double> arc;
  Path outer_centers;
  Path outer_directions;
  std::vector<double> outer_confidence;
  std::vector<double> outward_confidence;
};

// Classify geometric outer side, not whichever fragment happens to be longest.
// Image coordinates are x-right/y-down: left turn has negative signed angle,
// hence right boundary (side 1) is outer; right turn uses side 0.
void prepare_outer_reference(Fragment & f, const CenterlineConfig & cfg)
{
  const auto count = f.points.size();
  f.outer_confidence.assign(count, 0.0);
  f.outward_confidence.assign(count, 0.0);
  f.outer_centers = f.points;
  f.outer_directions = f.directions;
  if (!cfg.corner_outer_enabled || count < 5U || f.arc.back() < cfg.corner_outer_min_length_m) {return;}
  const double ds = f.arc.back() / (count - 1U);
  const auto filtered = gaussian(f.points, 0.04 / ds);
  const int tangent_half = std::max(1, static_cast<int>(std::round(
    cfg.corner_outer_tangent_window_m / (2.0 * ds))));
  const auto direction = tangents(filtered, tangent_half);
  // Smooth direction, but retain the actual observed boundary anchor positions.
  for (std::size_t i = 0; i < count; ++i) {
    const Point normal = Point(-direction[i].y, direction[i].x) * (f.side == 0 ? 1.0 : -1.0);
    f.outer_centers[i] = f.points[i] + normal * (cfg.lane_width_m / 2.0);
  }
  f.outer_directions = tangents(f.outer_centers, tangent_half);
  Path weights(count);
  const double half_window = cfg.corner_outer_window_m / 2.0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto lo = static_cast<std::size_t>(std::lower_bound(
      f.arc.begin(), f.arc.end(), f.arc[i] - half_window) - f.arc.begin());
    const auto high = std::upper_bound(f.arc.begin(), f.arc.end(), f.arc[i] + half_window);
    const auto hi = static_cast<std::size_t>(high - f.arc.begin() - 1);
    const double observed_span = f.arc[hi] - f.arc[lo];
    if (observed_span < cfg.corner_outer_min_length_m) {continue;}
    double signed_turn = 0.0;
    double absolute_turn = 0.0;
    for (std::size_t j = lo + 1U; j <= hi; ++j) {
      const auto & a = direction[j - 1U];
      const auto & b = direction[j];
      const double turn = std::atan2(a.x * b.y - a.y * b.x, a.dot(b));
      signed_turn += turn;
      absolute_turn += std::abs(turn);
    }
    const bool outer = f.side == 1 ? signed_turn < 0.0 : signed_turn > 0.0;
    if (!outer) {continue;}
    const double turn_strength = std::clamp((std::abs(signed_turn) -
      radians(cfg.corner_outer_min_turn_deg)) / radians(
      cfg.corner_outer_full_turn_deg - cfg.corner_outer_min_turn_deg), 0.0, 1.0);
    // Oscillating/S-shaped observations have less evidence of one turn direction.
    const double consistency = std::abs(signed_turn) / std::max(absolute_turn, 1.0e-9);
    const double coverage_ratio = std::clamp((observed_span - cfg.corner_outer_min_length_m) /
      (cfg.corner_outer_window_m - cfg.corner_outer_min_length_m), 0.0, 1.0);
    const double coverage = coverage_ratio * coverage_ratio * (3.0 - 2.0 * coverage_ratio);
    const double endpoint = std::min(1.0, std::min(f.arc[i], f.arc.back() - f.arc[i]) /
      (cfg.corner_outer_tangent_window_m / 2.0));
    weights[i].x = turn_strength * consistency * coverage * endpoint;
  }
  weights = gaussian(weights, 0.05 / ds);
  std::vector<bool> outer_usable(count, false);
  for (std::size_t i = 0; i < count; ++i) {
    // Outer offset can fold at an excessively tight radius. Never prefer it.
    const auto a = f.outer_centers[i == 0U ? 0U : i - 1U];
    const auto b = f.outer_centers[std::min(i + 1U, count - 1U)];
    if ((b - a).dot(direction[i]) <= 0.001 || f.outer_directions[i].dot(direction[i]) < 0.5) {continue;}
    outer_usable[i] = true;
    const double endpoint = std::min(1.0, std::min(f.arc[i], f.arc.back() - f.arc[i]) /
      (cfg.corner_outer_tangent_window_m / 2.0));
    f.outer_confidence[i] = std::clamp(weights[i].x * endpoint, 0.0, 1.0);
  }
  // Extend observed corner evidence toward the vehicle along this fragment.
  // Use only the original local weights: recursive propagation would shift
  // the entire visible straight. Shape blending keeps its local confidence.
  f.outward_confidence = f.outer_confidence;
  if (cfg.corner_entry_distance_m <= 0.0 || cfg.corner_outward_offset_m <= 0.0) {return;}
  for (std::size_t i = 0; i < count; ++i) {
    if (!outer_usable[i]) {continue;}
    for (std::size_t j = i + 1U; j < count; ++j) {
      const double distance = f.arc[j] - f.arc[i];
      if (distance >= cfg.corner_entry_distance_m) {break;}
      // Reach the full observed confidence before the bend, then hold it.
      // Zero hold distance reproduces the original ramp exactly.
      const double ratio = cfg.corner_entry_full_offset_distance_m == 0.0 ?
        1.0 - distance / cfg.corner_entry_distance_m :
        1.0 - std::clamp(
        (distance - cfg.corner_entry_full_offset_distance_m) /
        (cfg.corner_entry_distance_m - cfg.corner_entry_full_offset_distance_m), 0.0, 1.0);
      const double ramp = ratio * ratio * (3.0 - 2.0 * ratio);
      f.outward_confidence[i] = std::max(
        f.outward_confidence[i], f.outer_confidence[j] * ramp);
    }
  }
}
struct Candidate
{
  Point point;
  Point direction;
  std::uint8_t support;
  bool paired;
};
}  // namespace

void validate_centerline(const CenterlineConfig & c)
{
  const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  if (!positive(c.output_spacing_m) || c.output_spacing_m < 0.005 || c.output_spacing_m > 0.10) {
    throw std::invalid_argument("centerline_output_spacing_m must be within 0.005..0.10 metres");
  }
  if (!positive(c.clearance_check_spacing_m) ||
    c.clearance_check_spacing_m < 0.001 || c.clearance_check_spacing_m > 0.05)
  {
    throw std::invalid_argument(
      "centerline_clearance_check_spacing_m must be within 0.001..0.05 metres");
  }
  const double values[] = {c.lane_width_m, c.bev_width_m, c.bev_height_m, c.sample_spacing_m,
    c.min_fragment_length_m, c.tangent_window_m, c.width_tolerance_m,
    c.pair_along_tolerance_m, c.max_gap_m, c.max_start_distance_m, c.min_clearance_m,
    c.smoothing_window_m, c.smoothing_max_shift_m, c.turn_window_m,
    c.corner_outer_window_m, c.corner_outer_tangent_window_m, c.corner_outer_min_length_m};
  for (double v : values) {
    if (!positive(v)) {throw std::invalid_argument("Centerline distances must be finite and positive");}
  }
  if (!std::isfinite(c.corner_outward_offset_m) || c.corner_outward_offset_m < 0.0 ||
    c.corner_outward_offset_m >= c.lane_width_m / 2.0 - c.min_clearance_m)
  {
    throw std::invalid_argument(
      "corner outward offset must be nonnegative and less than half lane width minus clearance");
  }
  if (!std::isfinite(c.corner_entry_distance_m) || c.corner_entry_distance_m < 0.0 ||
    c.corner_entry_distance_m > 5.0)
  {throw std::invalid_argument("corner entry distance must be finite and within 0..5 metres");}
  if (!std::isfinite(c.corner_entry_full_offset_distance_m) ||
    c.corner_entry_full_offset_distance_m < 0.0 ||
    (c.corner_entry_full_offset_distance_m > 0.0 &&
    c.corner_entry_full_offset_distance_m >= c.corner_entry_distance_m))
  {
    throw std::invalid_argument(
      "corner entry full offset distance must be 0 or positive and smaller than entry distance");
  }
  if (!std::isfinite(c.corner_outer_weight) || c.corner_outer_weight < 0.0 || c.corner_outer_weight > 1.0 ||
    !positive(c.corner_outer_min_turn_deg) || !positive(c.corner_outer_full_turn_deg) ||
    c.corner_outer_full_turn_deg <= c.corner_outer_min_turn_deg || c.corner_outer_full_turn_deg >= 180.0 ||
    c.corner_outer_window_m > 5.0 || c.corner_outer_tangent_window_m > c.corner_outer_window_m ||
    c.corner_outer_min_length_m >= c.corner_outer_window_m ||
    c.corner_outer_tangent_window_m < 2.0 * c.sample_spacing_m ||
    c.corner_outer_min_length_m < 3.0 * c.sample_spacing_m ||
    !std::isfinite(c.outside_margin_m) || c.outside_margin_m < 0.0 ||
    !std::isfinite(c.smoothing_sigma_m) || c.smoothing_sigma_m < 0.0 ||
    !std::isfinite(c.smoothing_strength) || c.smoothing_strength < 0.0 || c.smoothing_strength > 1.0 ||
    !positive(c.pair_heading_tolerance_deg) || c.pair_heading_tolerance_deg >= 90.0 ||
    !std::isfinite(c.straight_turn_deg) || c.straight_turn_deg < 0.0 ||
    !positive(c.corner_turn_deg) || c.corner_turn_deg <= c.straight_turn_deg || c.corner_turn_deg >= 180.0 ||
    c.bev_width_m < 0.1 || c.bev_height_m < 0.1 || c.lane_width_m < 0.1 || c.lane_width_m > 5.0 ||
    c.min_clearance_m < 0.005 || c.max_gap_m > 2.0 || c.tangent_window_m > 5.0 ||
    c.sample_spacing_m < 0.005 || c.sample_spacing_m > 0.10 ||
    c.tangent_window_m < c.sample_spacing_m || c.max_gap_m < 2.0 * c.sample_spacing_m ||
    c.min_clearance_m >= c.lane_width_m / 2.0 || c.width_tolerance_m >= c.lane_width_m ||
    c.smoothing_window_m < 5.0 * c.sample_spacing_m || c.bev_width_m > 20.0 || c.bev_height_m > 20.0 ||
    c.smoothing_window_m > 5.0 || c.smoothing_sigma_m > 1.0 || c.turn_window_m > 5.0 ||
    c.outside_margin_m > 2.0 || c.max_samples < 32 || c.max_samples > 5000 ||
    c.line_width_px < 1 || c.line_width_px > 10)
  {throw std::invalid_argument("Invalid centerline geometry, smoothing or resource parameters");}
}

CenterlineResult generate_centerline(
  const cv::Mat & labels, const ObservedLanePaths & observed_paths,
  int source_width, int padding, const CenterlineConfig & cfg,
  const bool render_mask, ProcessingProfile * profile)
{
  ProfileTimer timer(profile, ProfileStage::centerline);
  CenterlineResult result;
  if (render_mask) {result.mask = cv::Mat::zeros(labels.size(), CV_8UC1);}
  if (!cfg.enabled) {return result;}
  if (labels.empty() || labels.type() != CV_8UC1 || source_width <= 0 || padding < 0 ||
    labels.cols != source_width + 2 * padding)
  {throw std::invalid_argument("Centerline requires mono8 labels and matching source/padding dimensions");}
  const double sx = cfg.bev_width_m / source_width;
  const double sy = cfg.bev_height_m / labels.rows;
  const auto metric = [=](const cv::Point2f & p) {return Point((p.x - padding) * sx, p.y * sy);};
  const Point ego(cfg.bev_width_m / 2.0, cfg.bev_height_m);
  ProfileTimer boundary_timer(profile, ProfileStage::boundary_index);
  Path boundary;
  std::vector<cv::Point> pixels;
  cv::findNonZero((labels == 1) | (labels == 2), pixels);
  for (const auto & p : pixels) {boundary.push_back(metric(cv::Point2f(p)));}
  if (profile) {profile->add(ProfileCounter::boundary_pixels, boundary.size());}
  if (boundary.empty()) {return result;}
  const SpatialIndex boundary_index(boundary, cfg.min_clearance_m);
  // Include pixel-cell half diagonal and half sampling step for a conservative
  // sampled clearance check. This measures observed pixels, not drivable space.
  const double check_step = cfg.clearance_check_spacing_m;
  const Geometry geometry{cfg.bev_width_m, cfg.bev_height_m,
    std::min(cfg.outside_margin_m, std::max(0, padding - cfg.line_width_px) * sx),
    cfg.min_clearance_m + 0.5 * std::hypot(sx, sy) + check_step / 2.0, check_step, boundary_index, profile};
  boundary_timer.stop();
  ProfileTimer fragment_timer(profile, ProfileStage::fragment_prepare);
  std::vector<Fragment> fragments;
  std::size_t samples = 0U;
  for (int side = 0; side < 2; ++side) {
    for (const auto & curve : observed_paths[side]) {
      Path points;
      for (const auto & p : curve) {points.push_back(metric(p));}
      const auto raw_arc = arc_lengths(points);
      if (raw_arc.back() < cfg.min_fragment_length_m) {continue;}
      const auto required = static_cast<std::size_t>(std::ceil(raw_arc.back() / cfg.sample_spacing_m)) + 1U;
      if (samples + required > static_cast<std::size_t>(cfg.max_samples)) {
        result.sample_limit_reached = true;
        return result;
      }
      if (norm(points.front() - ego) > norm(points.back() - ego)) {std::reverse(points.begin(), points.end());}
      points = gaussian(resample(points, cfg.sample_spacing_m), 1.0);
      const int half = std::max(1, static_cast<int>(std::round(cfg.tangent_window_m / (2.0 * cfg.sample_spacing_m))));
      Fragment f{side, points, tangents(points, half), arc_lengths(points), {}, {}, {}, {}};
      samples += points.size();
      if (profile) {
        profile->add(ProfileCounter::samples, points.size());
        profile->add(ProfileCounter::fragments);
      }
      fragments.push_back(std::move(f));
    }
  }
  fragment_timer.stop();
  ProfileTimer outer_timer(profile, ProfileStage::outer_reference);
  std::array<Path, 2> side_points;
  std::array<std::vector<std::pair<std::size_t, std::size_t>>, 2> side_references;
  for (std::size_t fragment_index = 0; fragment_index < fragments.size(); ++fragment_index) {
    auto & fragment = fragments[fragment_index];
    prepare_outer_reference(fragment, cfg);
    for (std::size_t point_index = 0; point_index < fragment.points.size(); ++point_index) {
      side_points[fragment.side].push_back(fragment.points[point_index]);
      side_references[fragment.side].emplace_back(fragment_index, point_index);
    }
  }
  outer_timer.stop();
  ProfileTimer pairing_timer(profile, ProfileStage::pairing_index);
  const double pairing_radius = std::hypot(
    cfg.lane_width_m + cfg.width_tolerance_m, cfg.pair_along_tolerance_m);
  std::array<std::unique_ptr<SpatialIndex>, 2> side_indices;
  for (int side = 0; side < 2; ++side) {
    if (!side_points[side].empty()) {
      side_indices[side] = std::make_unique<SpatialIndex>(side_points[side], pairing_radius);
    }
  }
  pairing_timer.stop();
  ProfileTimer candidate_timer(profile, ProfileStage::candidates);
  std::vector<Candidate> candidates;
  for (const auto & f : fragments) {
    Path offsets;
    Path preferred_directions;
    std::vector<std::uint8_t> support;
    std::vector<bool> paired;
    for (std::size_t i = 0; i < f.points.size(); ++i) {
      const auto & p = f.points[i];
      const auto & t = f.directions[i];
      const Point normal = Point(-t.y, t.x) * (f.side == 0 ? 1.0 : -1.0);
      Point center = p + normal * (cfg.lane_width_m / 2.0);
      Point preferred_direction = t;
      const Fragment * counterpart = nullptr;
      std::size_t counterpart_index = 0U;
      std::uint8_t mode = 1U;
      double best = std::numeric_limits<double>::infinity();
      const int opposite_side = 1 - f.side;
      if (side_indices[opposite_side]) {
        for (const int nearby_index : side_indices[opposite_side]->nearby(p, pairing_radius)) {
          if (profile) {profile->add(ProfileCounter::pairing_neighbors);}
          const auto [fragment_index, j] =
            side_references[opposite_side][static_cast<std::size_t>(nearby_index)];
          const auto & other = fragments[fragment_index];
          if (other.arc[j] < 0.025 || other.arc.back() - other.arc[j] < 0.025) {continue;}
          const Point delta = other.points[j] - p;
          const double across = delta.dot(normal);
          const double along = std::abs(delta.dot(t));
          if (std::abs(across - cfg.lane_width_m) > cfg.width_tolerance_m ||
            along > cfg.pair_along_tolerance_m ||
            other.directions[j].dot(t) < std::cos(radians(cfg.pair_heading_tolerance_deg))) {continue;}
          const double score = 3.0 * along + std::abs(across - cfg.lane_width_m);
          if (score < best) {
            best = score;
            center = (p + other.points[j]) * 0.5;
            mode = 2U;
            counterpart = &other;
            counterpart_index = j;
          }
        }
      }
      // Both sides' candidates converge toward the same outer-reference geometry,
      // reducing jumps when the graph changes its source side in a corner.
      const double own_outer = f.outer_confidence[i];
      const double other_outer = counterpart ? counterpart->outer_confidence[counterpart_index] : 0.0;
      const Fragment * reference = nullptr;
      std::size_t reference_index = i;
      // Conflicting evidence reduces the blend continuously to zero at a tie.
      // Avoid binary switches caused by tiny opposite-turn fluctuations.
      if (own_outer > other_outer) {reference = &f;}
      if (other_outer > own_outer) {
        reference = counterpart;
        reference_index = counterpart_index;
      }
      if (reference) {
        const double confidence = std::abs(own_outer - other_outer);
        const double weight = cfg.corner_outer_weight * confidence;
        const auto target = reference->outer_centers[reference_index];
        const auto direction = reference->outer_directions[reference_index];
        const auto blended = center + (target - center) * weight;
        if (weight > 1.0e-3 && direction.dot(t) > 0.5 &&
          geometry.point_ok(target) && geometry.point_ok(blended))
        {
          center = blended;
          preferred_direction = unit(t * (1.0 - weight) + direction * weight);
          mode = 4U;
        }
      }
      // The upcoming corner can bias a still-straight entry segment. Select
      // its outer side independently of the LOCAL curvature blend above, and
      // translate along the CURRENT boundary normal, not the future tangent.
      const double own_outward = f.outward_confidence[i];
      const double other_outward = counterpart ?
        counterpart->outward_confidence[counterpart_index] : 0.0;
      const Fragment * offset_reference = nullptr;
      std::size_t offset_index = i;
      if (own_outward > other_outward) {offset_reference = &f;}
      if (other_outward > own_outward) {
        offset_reference = counterpart;
        offset_index = counterpart_index;
      }
      if (offset_reference) {
        const double requested_offset = cfg.corner_outward_offset_m *
          std::abs(own_outward - other_outward);
        if (requested_offset > 1.0e-6 && geometry.point_ok(center)) {
          const Point outward = unit(offset_reference->points[offset_index] -
            offset_reference->outer_centers[offset_index]);
          const Point displacement = outward * requested_offset;
          double fraction = 1.0;
          if (!geometry.segment_ok(center, center + displacement)) {
            double lo = 0.0;
            double hi = 1.0;
            for (int iteration = 0; iteration < 8; ++iteration) {
              const double mid = (lo + hi) / 2.0;
              if (geometry.segment_ok(center, center + displacement * mid)) {lo = mid;}
              else {hi = mid;}
            }
            fraction = lo;
          }
          if (requested_offset * fraction > 1.0e-6) {
            center += displacement * fraction;
            mode = 4U;
          }
        }
      }
      paired.push_back(counterpart != nullptr);
      offsets.push_back(center);
      preferred_directions.push_back(preferred_direction);
      support.push_back(mode);
    }
    offsets = gaussian(offsets, 1.2);
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      const auto a = offsets[i == 0U ? i : i - 1U];
      const auto b = offsets[std::min(i + 1U, offsets.size() - 1U)];
      if ((b - a).dot(f.directions[i]) <= 0.001 ||
        (b - a).dot(preferred_directions[i]) <= 0.001 || !geometry.point_ok(offsets[i])) {continue;}
      candidates.push_back({offsets[i], preferred_directions[i], support[i], paired[i]});
    }
  }
  if (profile) {profile->add(ProfileCounter::candidates, candidates.size());}
  candidate_timer.stop();
  if (candidates.size() < 2U) {return result;}
  ProfileTimer search_timer(profile, ProfileStage::path_search);
  Path positions;
  int start = 0;
  double nearest = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    positions.push_back(candidates[i].point);
    const double d = norm(candidates[i].point - ego);
    if (d < nearest) {nearest = d; start = i;}
  }
  if (nearest > cfg.max_start_distance_m) {return result;}
  const SpatialIndex neighbors(positions, cfg.max_gap_m);
  std::vector<double> cost(positions.size(), std::numeric_limits<double>::infinity());
  std::vector<int> previous(positions.size(), -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  cost[start] = 0.0;
  queue.emplace(0.0, start);
  while (!queue.empty()) {
    const auto [distance, i] = queue.top();
    queue.pop();
    if (distance > cost[i]) {continue;}
    for (int j : neighbors.nearby(positions[i], cfg.max_gap_m)) {
      if (profile) {profile->add(ProfileCounter::graph_neighbor_visits);}
      const auto delta = positions[j] - positions[i];
      const double length = norm(delta);
      if (length < 0.006) {continue;}
      const auto direction = delta * (1.0 / length);
      const double aligned = direction.dot(candidates[i].direction);
      if (aligned < 0.5 || direction.dot(candidates[j].direction) < 0.5 ||
        candidates[i].direction.dot(candidates[j].direction) < 0.5) {continue;}
      double next = distance + length * (1.0 + (candidates[j].paired ? 0.0 : 0.6) +
        3.0 * (1.0 - aligned));
      next += std::max(0.0, length - 0.065) * 3.0;
      // Clearance is independent of graph cost: do not check edges that
      // cannot improve the best known route to this candidate.
      if (!(next < cost[j])) {
        if (profile) {profile->add(ProfileCounter::graph_cost_rejections);}
        continue;
      }
      if (!geometry.segment_ok(positions[i], positions[j])) {continue;}
      cost[j] = next;
      previous[j] = i;
      queue.emplace(next, j);
    }
  }
  int end = start;
  for (int i = 0; i < static_cast<int>(cost.size()); ++i) {
    if (std::isfinite(cost[i]) && norm(positions[i] - ego) > norm(positions[end] - ego)) {end = i;}
  }
  std::vector<int> ids;
  for (int i = end; i >= 0; i = previous[i]) {ids.push_back(i);}
  std::reverse(ids.begin(), ids.end());
  if (ids.size() < 2U) {return result;}
  search_timer.stop();
  ProfileTimer output_timer(profile, ProfileStage::path_output);
  Path raw;
  for (int id : ids) {raw.push_back(positions[id]);}
  const auto raw_arc = arc_lengths(raw);
  Path path = resample(raw, cfg.output_spacing_m);
  const double length = raw_arc.back();
  std::vector<std::uint8_t> provenance;
  std::size_t j = 1U;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double s = length * i / (path.size() - 1U);
    while (j + 1U < raw.size() && raw_arc[j] < s) {++j;}
    provenance.push_back(i == 0U ? candidates[ids.front()].support :
      (raw_arc[j] - raw_arc[j - 1U] > 0.065 ? 3U : candidates[ids[j]].support));
  }
  {
    ProfileTimer check_timer(profile, ProfileStage::path_check_before);
    if (!geometry.path_ok(path)) {return result;}
  }
  // Match smooth_centerline's no-op conditions; the unchanged path was
  // already validated above. Keep the second validation for changed paths.
  if (cfg.smoothing_enabled && cfg.smoothing_strength != 0.0 && path.size() >= 7U) {
    {
      ProfileTimer smoothing_timer(profile, ProfileStage::smoothing);
      path = smooth_centerline(path, geometry, cfg);
    }
    ProfileTimer check_timer(profile, ProfileStage::path_check_after);
    if (!geometry.path_ok(path)) {return result;}
  }
  result.support = std::move(provenance);
  for (const auto & p : path) {
    result.points.emplace_back(static_cast<float>(p.x / sx + padding), static_cast<float>(p.y / sy));
  }
  if (profile) {profile->add(ProfileCounter::output_points, result.points.size());}
  if (render_mask) {
    for (std::size_t i = 1; i < result.points.size(); ++i) {
      cv::line(
        result.mask, result.points[i - 1], result.points[i],
        cv::Scalar(255), cfg.line_width_px, cv::LINE_8);
    }
  }
  return result;
}
}  // namespace line_detactor
