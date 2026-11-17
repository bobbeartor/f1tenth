#include "multiclass_yolo/object_localization.hpp"
#include <iostream>
#include <stdexcept>

using multiclass_yolo::Footprint;
using multiclass_yolo::ObjectPoint;
using multiclass_yolo::localize;

void check(bool v, const char * msg) {if (!v) throw std::runtime_error(msg);}
bool near(double a, double b, double tolerance = 1e-6) {return std::abs(a - b) <= tolerance;}

// `count` points spread across `width` at forward distance `x`.
void add(std::vector<ObjectPoint> & points, double x, double width, int count) {
  for (int i = 0; i < count; ++i)
    points.push_back({x + .002 * (i % 5), width * (static_cast<double>(i) / (count - 1) - .5)});
}

int main() {
  const Footprint sedan{.50, .30}, cone{.13, .13};

  std::vector<ObjectPoint> points;
  add(points, 1.00, .28, 30);   // sedan rear face
  add(points, 2.50, .60, 100);  // wall behind it, inside the same box
  points.push_back({.40, 0});   // stray near returns from a box edge
  points.push_back({.42, .01});
  points.push_back({.45, -.01});
  auto object = localize(points, 0, 0, 0.0, sedan);
  check(object.valid, "object found");
  check(near(object.range, 1.0, .02), "nearest populated peak, not the wall or stray points");
  check(object.cluster_points >= 30 && object.cluster_points < 40, "wall excluded from the object");
  check(object.cluster_max < 2.0 && object.cluster_min > .9, "accepted range interval");
  check(near(object.near_range, 1.0, .02), "near range ignores stray points");
  check(near(object.center_x, object.range + .25) && near(object.center_y, 0), "centre half a length behind the face");
  check(near(object.corners[0].x, object.range) && near(object.corners[0].y, .15), "near-left corner");
  check(near(object.corners[2].x, object.range + .50) && near(object.corners[2].y, -.15), "far-right corner");

  // The box-centre ray sets direction; points only set range.
  const double bearing = std::atan2(1.0, 1.0);
  object = localize(points, 0, 0, bearing, cone);
  check(near(object.front_x, object.front_y) && near(object.center_x, object.center_y), "bearing hint used");
  check(near(std::hypot(object.center_x, object.center_y), object.range + .065), "cone centre offset");

  // Without a ray the selected points decide the direction.
  std::vector<ObjectPoint> left;
  for (int i = 0; i < 12; ++i) left.push_back({1.0, 1.0 + .001 * i});
  object = localize(left, 0, 0, std::nullopt, cone);
  check(object.valid && near(object.bearing, std::atan2(1.0, 1.0), .01), "bearing from points");

  // Range is measured from the camera, not the vehicle origin.
  object = localize(left, -.09, 0, std::nullopt, cone);
  check(near(object.range, std::hypot(1.09, 1.005), .01), "camera origin");

  check(!localize({{1, 0}, {1, .01}, {1, .02}}, 0, 0, 0.0, cone).valid, "too few points");
  std::vector<ObjectPoint> scattered;
  for (int i = 0; i < 20; ++i) scattered.push_back({1.0 + .5 * i, 0});
  check(!localize(scattered, 0, 0, 0.0, cone).valid, "no populated peak");

  // A class without a footprint still gets a position.
  object = localize(left, 0, 0, std::nullopt, Footprint{});
  check(object.valid && near(object.center_x, object.front_x), "no footprint keeps the face position");
  std::cout << "Object localization tests passed\n";
}
