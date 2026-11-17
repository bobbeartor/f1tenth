#pragma once
#include "roi_lidar/clusters.hpp"
#include "roi_lidar/view.hpp"
#include "roi_lidar/terrain.hpp"
#include <opencv2/core.hpp>
#include <string>
namespace roi_lidar
{
struct MapLayout
{
  double scale, center_x, origin_y;
  cv::Size canvas;
  cv::Rect area;
};
MapLayout mapLayout(const ViewOptions & view, int size);
cv::Point mapPixel(double x, double y, const ViewOptions & view, int size);
cv::Mat mapPreview(const Scan & scan, const Options & o, const ViewOptions & view, int size, const std::string & status, const cv::Mat & background = {}, const Clusters * clusters = nullptr, bool show_raw_points = false);
class BevProjector
{
  cv::Mat map_x_, map_y_;
public:
  void configure(int width, int height, const point_cloud::Intrinsics & k,
    const point_cloud::RigidTransform & vehicle_from_camera, const ViewOptions & view, int size);
  void configureLane(int width, int height, const point_cloud::Intrinsics & k,
    const point_cloud::RigidTransform & vehicle_from_camera);
  cv::Mat render(const cv::Mat & bgr, const std::string & interpolation = "linear") const;
};
cv::Mat roiPreview(const cv::Mat & gray_or_depth, const Options & o, bool depth);
void terrainOverlay(cv::Mat & image, const Terrain & terrain, const Scan & scan,
  const Options & o, const ViewOptions & view, int size, bool show_cells,
  bool show_points, double alpha, int point_radius, double vehicle_width_m);
}
