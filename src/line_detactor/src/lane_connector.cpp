#include "line_detactor/lane_connector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{
using Point = cv::Point2f;
constexpr double kPi = 3.14159265358979323846;

double length(const Point & p) {return std::hypot(p.x, p.y);}
double cross(const Point & a, const Point & b) {return a.x * b.y - a.y * b.x;}
Point unit(const Point & p)
{
  const double size = length(p);
  return size > 1.0e-6 ? p * static_cast<float>(1.0 / size) : Point();
}

double arc_length(const std::vector<Point> & points)
{
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += length(points[i] - points[i - 1]);
  }
  return total;
}

// Zhang-Suen thinning. Pad the ROI so border-touching endpoints are retained.
cv::Mat thin(const cv::Mat & mask, ProcessingProfile * profile)
{
  ProfileTimer timer(profile, ProfileStage::thinning);
  cv::Mat image;
  cv::copyMakeBorder(mask, image, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
  image /= 255;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      if (profile) {
        profile->add(ProfileCounter::thinning_passes);
        profile->add(ProfileCounter::thinning_pixel_visits,
          static_cast<std::uint64_t>(image.rows - 2) * (image.cols - 2));
      }
      std::vector<cv::Point> remove;
      for (int y = 1; y < image.rows - 1; ++y) {
        for (int x = 1; x < image.cols - 1; ++x) {
          if (!image.at<std::uint8_t>(y, x)) {continue;}
          const int n[8] = {
            image.at<std::uint8_t>(y - 1, x), image.at<std::uint8_t>(y - 1, x + 1),
            image.at<std::uint8_t>(y, x + 1), image.at<std::uint8_t>(y + 1, x + 1),
            image.at<std::uint8_t>(y + 1, x), image.at<std::uint8_t>(y + 1, x - 1),
            image.at<std::uint8_t>(y, x - 1), image.at<std::uint8_t>(y - 1, x - 1)};
          int neighbors = 0;
          int transitions = 0;
          for (int k = 0; k < 8; ++k) {
            neighbors += n[k];
            transitions += n[k] == 0 && n[(k + 1) % 8] != 0;
          }
          const bool removable = pass == 0 ?
            n[0] * n[2] * n[4] == 0 && n[2] * n[4] * n[6] == 0 :
            n[0] * n[2] * n[6] == 0 && n[0] * n[4] * n[6] == 0;
          if (neighbors >= 2 && neighbors <= 6 && transitions == 1 && removable) {
            remove.emplace_back(x, y);
          }
        }
      }
      for (const auto & point : remove) {image.at<std::uint8_t>(point) = 0;}
      changed = changed || !remove.empty();
    }
  }
  return image(cv::Rect(1, 1, mask.cols, mask.rows)).clone();
}

// Keep a single ordered skeleton path per component; side branches are not lanes.
std::vector<Point> component_path(
  const cv::Mat & mask, const cv::Point & origin, const int downsample_factor,
  ProcessingProfile * profile)
{
  if (downsample_factor > 1) {
    const cv::Size reduced_size(
      (mask.cols + downsample_factor - 1) / downsample_factor,
      (mask.rows + downsample_factor - 1) / downsample_factor);
    cv::Mat reduced;
    // Retain any occupied area so thin lane pixels are not lost by nearest-neighbor sampling.
    cv::resize(mask, reduced, reduced_size, 0.0, 0.0, cv::INTER_AREA);
    reduced = reduced > 0;
    auto path = component_path(reduced, cv::Point(0, 0), 1, profile);
    if (path.size() >= 2U) {
      const float sx = static_cast<float>(mask.cols) / reduced.cols;
      const float sy = static_cast<float>(mask.rows) / reduced.rows;
      for (auto & point : path) {
        // Map pixel centers using actual ROI dimensions, including partial edge cells.
        point.x = origin.x + (point.x + 0.5F) * sx - 0.5F;
        point.y = origin.y + (point.y + 0.5F) * sy - 0.5F;
      }
      return path;
    }
    // Tiny or degenerate reduced components retain the original extraction behavior.
  }
  const cv::Mat skeleton = thin(mask, profile);
  ProfileTimer graph_timer(profile, ProfileStage::skeleton_graph);
  std::vector<cv::Point> pixels;
  cv::findNonZero(skeleton, pixels);
  if (profile) {profile->add(ProfileCounter::skeleton_pixels, pixels.size());}
  if (pixels.size() < 2U) {return {};}
  cv::Mat indices(mask.size(), CV_32S, cv::Scalar(-1));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    indices.at<int>(pixels[i]) = static_cast<int>(i);
  }
  std::vector<std::vector<int>> adjacent(pixels.size());
  std::vector<int> endpoints;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    const auto p = pixels[i];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const cv::Point q = p + cv::Point(dx, dy);
        if ((dx == 0 && dy == 0) || q.x < 0 || q.y < 0 ||
          q.x >= indices.cols || q.y >= indices.rows) {continue;}
        const int neighbor = indices.at<int>(q);
        // Avoid diagonal shortcuts around a 4-connected corner.
        if (dx && dy && (indices.at<int>(p.y, q.x) >= 0 ||
          indices.at<int>(q.y, p.x) >= 0)) {continue;}
        if (neighbor >= 0) {adjacent[i].push_back(neighbor);}
      }
    }
    if (adjacent[i].size() == 1U) {endpoints.push_back(static_cast<int>(i));}
  }
  if (endpoints.size() < 2U) {return {};}
  const int start = *std::max_element(endpoints.begin(), endpoints.end(), [&](int a, int b) {
    return pixels[a].y < pixels[b].y;
  });
  std::vector<double> distance(pixels.size(), std::numeric_limits<double>::infinity());
  std::vector<int> previous(pixels.size(), -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  distance[start] = 0.0;
  queue.emplace(0.0, start);
  while (!queue.empty()) {
    const auto [cost, i] = queue.top();
    queue.pop();
    if (cost > distance[i]) {continue;}
    for (const int j : adjacent[i]) {
      const double next = cost + cv::norm(pixels[i] - pixels[j]);
      if (next < distance[j]) {
        distance[j] = next;
        previous[j] = i;
        queue.emplace(next, j);
      }
    }
  }
  int end = start;
  for (const int candidate : endpoints) {
    if (std::isfinite(distance[candidate]) && distance[candidate] > distance[end]) {
      end = candidate;
    }
  }
  std::vector<Point> path;
  for (int i = end; i >= 0; i = previous[i]) {
    path.emplace_back(pixels[i] + origin);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

Point tangent(const std::vector<Point> & path, const bool at_end, const double window)
{
  std::vector<Point> sample;
  double walked = 0.0;
  for (std::size_t j = 0; j < path.size(); ++j) {
    const std::size_t i = at_end ? path.size() - 1U - j : j;
    if (!sample.empty()) {walked += length(path[i] - sample.back());}
    sample.push_back(path[i]);
    if (walked >= window) {break;}
  }
  cv::Vec4f line;
  cv::fitLine(sample, line, cv::DIST_L2, 0.0, 0.01, 0.01);
  Point direction(line[0], line[1]);
  const Point forward = at_end ? sample.front() - sample.back() : sample.back() - sample.front();
  if (direction.dot(forward) < 0.0F) {direction *= -1.0F;}
  return unit(direction);
}

struct BorderEndpoint
{
  Point point;
  Point outward;
  int component;
  int border;  // 0=left image edge, 1=right image edge.
};

bool intersects(const Point & a, const Point & b, const Point & c, const Point & d)
{
  if (std::max(a.x, b.x) < std::min(c.x, d.x) ||
    std::max(c.x, d.x) < std::min(a.x, b.x) ||
    std::max(a.y, b.y) < std::min(c.y, d.y) ||
    std::max(c.y, d.y) < std::min(a.y, b.y)) {return false;}
  const double ab_c = cross(b - a, c - a);
  const double ab_d = cross(b - a, d - a);
  const double cd_a = cross(d - c, a - c);
  const double cd_b = cross(d - c, b - c);
  if (ab_c * ab_d < -1.0e-8 && cd_a * cd_b < -1.0e-8) {return true;}
  const auto on_segment = [](const Point & p, const Point & q, const Point & r) {
    return std::abs(cross(q - p, r - p)) < 1.0e-5 &&
           r.x >= std::min(p.x, q.x) - 1.0e-5 && r.x <= std::max(p.x, q.x) + 1.0e-5 &&
           r.y >= std::min(p.y, q.y) - 1.0e-5 && r.y <= std::max(p.y, q.y) + 1.0e-5;
  };
  return on_segment(a, b, c) || on_segment(a, b, d) ||
         on_segment(c, d, a) || on_segment(c, d, b);
}

bool self_intersects(const std::vector<Point> & points)
{
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (std::size_t j = i + 2U; j < points.size(); ++j) {
      if (intersects(points[i - 1], points[i], points[j - 1], points[j])) {return true;}
    }
  }
  return false;
}

std::vector<Point> bridge(
  const BorderEndpoint & a, const BorderEndpoint & b, const LaneConnectionConfig & config,
  const int width, const int height)
{
  if (a.component == b.component || a.border != b.border || config.padding_px == 0) {return {};}
  const Point start = a.point;
  const Point end = b.point;
  const Point chord = end - start;
  const double gap = length(chord);
  if (gap < 1.0 || gap > config.max_gap_px) {return {};}
  const Point entry = -b.outward;
  // Both measured fragments must point OUT of the same image edge.
  const float sign = a.border == 0 ? -1.0F : 1.0F;
  if (a.outward.x * sign <= 0.05F || b.outward.x * sign <= 0.05F) {return {};}
  const double cosine = std::clamp(static_cast<double>(a.outward.dot(entry)), -1.0, 1.0);
  if (std::acos(cosine) > config.max_turn_deg * kPi / 180.0) {return {};}
  const Point average = length(a.outward + entry) < 0.2 ? unit(chord) : unit(a.outward + entry);
  const double corridor = config.corridor_half_width_px +
    gap * std::tan(config.direction_tolerance_deg * kPi / 180.0);
  if (average.dot(chord) <= 0.0F || std::abs(cross(average, chord)) > corridor ||
    a.outward.dot(chord) < -config.corridor_half_width_px ||
    entry.dot(chord) < -config.corridor_half_width_px) {return {};}

  std::vector<Point> best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const double handle_ratio : {0.25, 0.4, 0.6, 0.9}) {
    const Point c1 = start + a.outward * static_cast<float>(gap * handle_ratio);
    const Point c2 = end - entry * static_cast<float>(gap * handle_ratio);
    // The Bezier derivative bound gives <= 1px arc steps.
    const int steps = std::max(8, static_cast<int>(std::ceil(
      3.0 * std::max({length(c1 - start), length(c2 - c1), length(end - c2)}))));
    std::vector<Point> curve;
    double maximum_curvature = 0.0;
    bool valid = true;
    for (int i = 0; i <= steps; ++i) {
      const float t = static_cast<float>(i) / steps;
      const float u = 1.0F - t;
      const Point point = start * (u * u * u) + c1 * (3.0F * u * u * t) +
        c2 * (3.0F * u * t * t) + end * (t * t * t);
      const Point velocity = (c1 - start) * (3.0F * u * u) +
        (c2 - c1) * (6.0F * u * t) + (end - c2) * (3.0F * t * t);
      const Point acceleration = (c2 - c1 * 2.0F + start) * (6.0F * u) +
        (end - c2 * 2.0F + c1) * (6.0F * t);
      const double speed = length(velocity);
      const double curvature = speed > 1.0e-4 ?
        std::abs(cross(velocity, acceleration)) / (speed * speed * speed) :
        std::numeric_limits<double>::infinity();
      maximum_curvature = std::max(maximum_curvature, curvature);
      const bool inside_source = a.border == 0 ? point.x > 0.001F : point.x < width - 1 - 0.001F;
      if (inside_source || point.x < -config.padding_px || point.x > width - 1 + config.padding_px ||
        point.y < 0.0F || point.y > height - 1 || curvature > config.max_curvature_per_px)
      {valid = false; break;}
      curve.push_back(point);
    }
    if (!valid) {continue;}
    const double arc = arc_length(curve);
    if (arc > gap * config.max_arc_ratio || self_intersects(curve)) {continue;}
    // Prefer short, gently bending bridges, never reward unsupported length.
    const double cost = arc + maximum_curvature * gap;
    if (cost < best_cost) {best_cost = cost; best = std::move(curve);}
  }
  return best;
}

void add_endpoint(std::vector<BorderEndpoint> & endpoints, const std::vector<Point> & path,
  const bool at_end, const cv::Mat & components, const int id,
  const LaneConnectionConfig & config)
{
  const Point tip = at_end ? path.back() : path.front();
  for (const int border : {0, 1}) {
    const int x = border == 0 ? 0 : components.cols - 1;
    if (std::abs(tip.x - x) > config.border_endpoint_distance_px) {continue;}
    // Require an actual model pixel on the edge, not just a nearby interior endpoint.
    Point contact;
    double nearest = config.border_endpoint_distance_px;
    bool found = false;
    for (int y = 0; y < components.rows; ++y) {
      if (components.at<int>(y, x) != id) {continue;}
      const double distance = length(Point(static_cast<float>(x), static_cast<float>(y)) - tip);
      if (distance <= nearest) {
        nearest = distance;
        contact = Point(static_cast<float>(x), static_cast<float>(y));
        found = true;
      }
    }
    if (found) {
      const Point outward = tangent(path, at_end, config.tangent_window_px) * (at_end ? 1.0F : -1.0F);
      endpoints.push_back(BorderEndpoint{contact, outward, id, border});
      break;
    }
  }
}

std::vector<BorderEndpoint> retain_components(
  const cv::Mat & input, const int side, const LaneConnectionConfig & config,
  cv::Mat & output, std::vector<std::vector<Point>> & observed_paths,
  const bool connect_borders, ProcessingProfile * profile)
{
  ProfileTimer components_timer(profile, ProfileStage::components);
  // This scratch storage belongs to the processing thread. Result masks/paths
  // own their data, so the next side/frame can safely reuse these allocations.
  struct Scratch {cv::Mat binary, labels16, components, stats, centroids, mask;};
  thread_local Scratch scratch;
  auto & components = scratch.components;
  auto & stats = scratch.stats;
  cv::compare(input, side + 1, scratch.binary, cv::CMP_EQ);
  int count;
  if (config.ccl_serial_enabled &&
    input.total() < std::numeric_limits<std::uint16_t>::max())
  {
    // OpenCV's CV_16U CCL uses its serial implementation. The pixel-count
    // bound guarantees room for every possible component plus background.
    count = cv::connectedComponentsWithStats(
      scratch.binary, scratch.labels16, stats, scratch.centroids, 8, CV_16U);
    scratch.labels16.convertTo(components, CV_32S);
  } else {
    count = cv::connectedComponentsWithStats(
      scratch.binary, components, stats, scratch.centroids, 8, CV_32S);
  }
  std::vector<int> retained_components;
  std::vector<int> border_candidates;
  for (int id = 1; id < count; ++id) {
    if (stats.at<int>(id, cv::CC_STAT_AREA) < config.min_component_area_px) {continue;}
    const cv::Rect roi(stats.at<int>(id, cv::CC_STAT_LEFT), stats.at<int>(id, cv::CC_STAT_TOP),
      stats.at<int>(id, cv::CC_STAT_WIDTH), stats.at<int>(id, cv::CC_STAT_HEIGHT));
    cv::compare(components(roi), id, scratch.mask, cv::CMP_EQ);
    const cv::Mat & mask = scratch.mask;
    // No thinning/repainting, minimum-length deletion, winner selection or pair rejection.
    output(cv::Rect(roi.x + config.padding_px, roi.y, roi.width, roi.height)).setTo(side + 1, mask);
    retained_components.push_back(id);
    if (roi.x == 0 || roi.x + roi.width == input.cols) {border_candidates.push_back(id);}
  }
  if (profile) {
    profile->add(ProfileCounter::components_found, count - 1);
    profile->add(ProfileCounter::components_retained, retained_components.size());
  }
  components_timer.stop();
  ProfileTimer skeleton_timer(profile, ProfileStage::skeleton);
  std::vector<std::vector<Point>> component_paths(static_cast<std::size_t>(count));
  for (const int id : retained_components) {
    const cv::Rect roi(stats.at<int>(id, cv::CC_STAT_LEFT), stats.at<int>(id, cv::CC_STAT_TOP),
      stats.at<int>(id, cv::CC_STAT_WIDTH), stats.at<int>(id, cv::CC_STAT_HEIGHT));
    cv::compare(components(roi), id, scratch.mask, cv::CMP_EQ);
    auto path = component_path(
      scratch.mask, roi.tl(), config.skeleton_downsample_factor, profile);
    if (path.size() >= 2U) {
      auto padded_path = path;
      for (auto & point : padded_path) {point.x += config.padding_px;}
      observed_paths.push_back(std::move(padded_path));
    }
    component_paths[static_cast<std::size_t>(id)] = std::move(path);
  }
  skeleton_timer.stop();
  if (!connect_borders) {return {};}
  ProfileTimer endpoint_timer(profile, ProfileStage::border_endpoints);
  std::stable_sort(border_candidates.begin(), border_candidates.end(), [&](int a, int b) {
    return stats.at<int>(a, cv::CC_STAT_AREA) > stats.at<int>(b, cv::CC_STAT_AREA);
  });
  if (border_candidates.size() > static_cast<std::size_t>(config.max_fragments)) {
    border_candidates.resize(config.max_fragments);
  }
  std::vector<BorderEndpoint> endpoints;
  for (const int id : border_candidates) {
    const auto & path = component_paths[static_cast<std::size_t>(id)];
    // Length only gates extrapolation eligibility, never removes model pixels.
    if (arc_length(path) < config.min_fragment_length_px || path.size() < 2U) {continue;}
    add_endpoint(endpoints, path, false, components, id, config);
    add_endpoint(endpoints, path, true, components, id, config);
  }
  if (profile) {profile->add(ProfileCounter::border_endpoints, endpoints.size());}
  return endpoints;
}

cv::Mat bridge_mask(const std::vector<Point> & curve, const cv::Size & size,
  const LaneConnectionConfig & config)
{
  cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
  for (std::size_t i = 1; i < curve.size(); ++i) {
    const Point a = (curve[i - 1] + Point(static_cast<float>(config.padding_px), 0.0F)) * 256.0F;
    const Point b = (curve[i] + Point(static_cast<float>(config.padding_px), 0.0F)) * 256.0F;
    cv::line(mask, cv::Point(cvRound(a.x), cvRound(a.y)), cv::Point(cvRound(b.x), cvRound(b.y)),
      cv::Scalar(255), config.line_width_px, cv::LINE_8, 8);
  }
  // Even stroke thickness must never add/change pixels inside the original BEV.
  mask(cv::Rect(config.padding_px, 0, size.width - 2 * config.padding_px, size.height)).setTo(0);
  return mask;
}

}  // namespace

void validate_lane_connection(const LaneConnectionConfig & config)
{
  const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  if (config.skeleton_downsample_factor < 1 || config.skeleton_downsample_factor > 4) {
    throw std::invalid_argument("connection_skeleton_downsample_factor must be within 1..4");
  }
  if (config.padding_px < 0 || config.padding_px > 300 || config.min_component_area_px < 1 ||
    config.max_fragments < 1 || config.max_fragments > 64 || config.line_width_px < 1 ||
    config.line_width_px > 10 || !positive(config.min_fragment_length_px) ||
    !positive(config.tangent_window_px) || !positive(config.max_gap_px) || config.max_gap_px > 600 ||
    !positive(config.corridor_half_width_px) || !positive(config.direction_tolerance_deg) ||
    config.direction_tolerance_deg >= 80 || !positive(config.max_turn_deg) ||
    config.max_turn_deg > 180 || !positive(config.max_curvature_per_px) ||
    !positive(config.max_arc_ratio) || config.max_arc_ratio < 1.0 ||
    !positive(config.border_endpoint_distance_px))
  {throw std::invalid_argument("Invalid border interpolation geometry/size parameters");}
}

LaneConnectionResult connect_lane_fragments(
  const cv::Mat & labels, const LaneConnectionConfig & config,
  const bool render_image, ProcessingProfile * profile, const bool render_bgr)
{
  ProfileTimer timer(profile, ProfileStage::connector);
  if (labels.type() != CV_8UC1 || labels.empty()) {
    throw std::invalid_argument("Lane connector expects a nonempty mono8 label image");
  }
  LaneConnectionResult result;
  const cv::Size size(labels.cols + 2 * config.padding_px, labels.rows);
  result.labels = cv::Mat::zeros(size, CV_8UC1);
  struct Candidate {int side; int a; int b; std::vector<Point> curve;};
  std::array<std::vector<BorderEndpoint>, 2> endpoints;
  std::vector<Candidate> candidates;
  for (int side = 0; side < 2; ++side) {
    endpoints[side] = retain_components(
      labels, side, config, result.labels, result.observed_paths[side], render_image, profile);
    ProfileTimer candidate_timer(profile, ProfileStage::bridge_candidates);
    const auto & tips = endpoints[side];
    for (std::size_t a = 0; a < tips.size(); ++a) {
      for (std::size_t b = a + 1; b < tips.size(); ++b) {
        if (profile) {profile->add(ProfileCounter::bridge_pairs);}
        auto curve = bridge(tips[a], tips[b], config, labels.cols, labels.rows);
        if (!curve.empty()) {
          if (profile) {profile->add(ProfileCounter::valid_bridges);}
          candidates.push_back(Candidate{side, static_cast<int>(a), static_cast<int>(b), std::move(curve)});
        }
      }
    }
  }
  ProfileTimer bridge_timer(profile, ProfileStage::bridge_render);
  // Nearest compatible pair first; every endpoint can participate only once.
  std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return arc_length(a.curve) < arc_length(b.curve);
  });
  std::array<std::vector<bool>, 2> used{
    std::vector<bool>(endpoints[0].size(), false), std::vector<bool>(endpoints[1].size(), false)};
  for (const auto & candidate : candidates) {
    if (used[candidate.side][candidate.a] || used[candidate.side][candidate.b]) {continue;}
    const cv::Mat mask = bridge_mask(candidate.curve, size, config);
    if (cv::countNonZero(mask) == 0) {continue;}
    cv::Mat collision;
    cv::bitwise_and(mask, result.labels != 0, collision);
    if (cv::countNonZero(collision) != 0) {continue;}
    result.labels.setTo(candidate.side + 3, mask);
    used[candidate.side][candidate.a] = true;
    used[candidate.side][candidate.b] = true;
  }
  bridge_timer.stop();
  ProfileTimer render_timer(profile, ProfileStage::lane_render);
  if (render_image && render_bgr) {result.image = cv::Mat::zeros(size, CV_8UC3);}
  for (int side = 0; side < 2; ++side) {
    const cv::Mat model_mask = result.labels == side + 1;
    if (cv::countNonZero(model_mask) == 0) {continue;}
    result.state |= static_cast<std::uint8_t>(1U << side);
    if (render_image && render_bgr) {
      const cv::Mat mask = model_mask | (result.labels == side + 3);
      result.image.setTo(side == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), mask);
    }
  }
  return result;
}

}  // namespace line_detactor
