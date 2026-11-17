#include "roi_lidar/config.hpp"
#include <functional>
#include <iostream>
int main()
{
  roi_lidar::Config c; roi_lidar::validate(c);
  if (c.camera.fps!=60 || c.terrain_alpha!=.8 || c.point_radius!=2 || c.view.width_m!=1.5 || c.view.forward_m!=4) {throw std::runtime_error("Requested defaults lost");}
  c.rgb_fps=80; roi_lidar::validate(c);
  c.camera.fps=110; c.camera.subpixel=false; roi_lidar::validate(c);
  c.camera.resolution="800p"; c.camera.fps=30; c.camera.subpixel=true;
  c.scan.max_depth=25; c.scan.max_range=20; roi_lidar::validate(c);
  const auto rejects=[](const std::function<void(roi_lidar::Config &)> & change) {
    roi_lidar::Config bad; change(bad);
    try {roi_lidar::validate(bad);} catch (const std::invalid_argument &) {return;}
    throw std::runtime_error("Invalid config accepted");
  };
  rejects([](auto & p){p.preview_fps=0;});
  rejects([](auto & p){p.rgb_fps=121;});
  rejects([](auto & p){p.sync_sec=-1;});
  rejects([](auto & p){p.max_age=std::numeric_limits<double>::quiet_NaN();});
  rejects([](auto & p){p.view.width_m=0;});
  rejects([](auto & p){p.view.forward_m=std::numeric_limits<double>::quiet_NaN();});
  rejects([](auto & p){p.preview_size=10000;});
  rejects([](auto & p){p.scan.roi_width=2;});
  rejects([](auto & p){p.camera.subpixel=true; p.camera.extended=true;});
  rejects([](auto & p){p.terrain.cell_m=0;});
  rejects([](auto & p){p.terrain.fill_max_cells=0;});
  rejects([](auto & p){p.terrain.fill_max_cells=17;});
  rejects([](auto & p){p.terrain.fill_clearance_cells=-1;});
  rejects([](auto & p){p.terrain.fill_clearance_cells=5;});
  rejects([](auto & p){p.terrain.below_ground_m=-.01;});
  rejects([](auto & p){p.terrain.min_ground_samples=0;});
  rejects([](auto & p){p.terrain.vehicle_width_m=0;});
  rejects([](auto & p){p.terrain_alpha=1.1;});
  rejects([](auto & p){p.point_radius=0;});
  std::cout<<"ROI camera/profile configuration tests passed\n";
}
