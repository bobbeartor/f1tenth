#include "roi_lidar/controls.hpp"
#include <map>
#include <stdexcept>
#include <iostream>
int main() {
  using namespace roi_lidar;
  std::map<std::string,double> values{{"height.max_m",1},{"ground.distance_m",.05},{"ground.distance_per_meter",.01},
    {"ground.max_distance_m",.12},{"ground.enabled",1},{"terrain.below_ground_m",.05},{"terrain.cell_m",.05},
    {"terrain.min_ground_samples",2},{"terrain.min_obstacle_samples",2},{"terrain.vehicle_width_m",.3},{"points.pixel_stride",2},
    {"preview.show_terrain",1},{"preview.show_projected_points",1},{"preview.terrain_alpha",.8},
    {"preview.point_radius",2},{"preview.width_m",1.2},{"preview.forward_m",3},{"camera.fps",30},
    {"depth.confidence_threshold",200},{"depth.left_right_check",1},{"depth.left_right_threshold",10},
    {"depth.subpixel",1},{"depth.ir_dot_projector_intensity",.7},{"depth.ir_flood_light_intensity",0}};
  values["terrain.fill_enabled"]=1; values["terrain.fill_max_cells"]=2; values["terrain.fill_clearance_cells"]=1;
  values["obstacle.min_height_m"]=.05;
  values["ground.ransac_enabled"]=1;values["ground.ransac_distance_m"]=.025;
  values["ground.ransac_max_range_m"]=3;values["ground.ransac_max_tilt_deg"]=10;
  values["ground.ransac_max_offset_m"]=.08;values["ground.ransac_min_inlier_ratio"]=.5;
  values["ground.ransac_max_iterations"]=60;values["ground.ransac_max_samples"]=800;
  values["outlier.enabled"]=1;values["outlier.radius_m"]=.06;values["outlier.min_neighbors"]=3;
  Controls c; auto read=[&](const char * n) {return values.at(n);};
  if (!c.poll(read).empty()) {throw std::runtime_error("Initial GUI modified parameters");}
  c.poll(read,9); // Focus the first numeric field; typing replaces its selected value.
  c.poll(read,'0'); c.poll(read,'.'); c.poll(read,'5');
  if (!c.poll(read).empty()) {throw std::runtime_error("Draft applied before confirmation");}
  auto changes=c.poll(read,13);
  if (changes.size()!=1 || changes[0].name!="height.max_m" || std::abs(changes[0].value-.5)>1e-9) {throw std::runtime_error("Numeric height input/apply failed");}
  values["height.max_m"]=.5; c.poll(read);
  c.setDraft("depth.confidence_threshold","120");
  if (!c.poll(read).empty()) {throw std::runtime_error("Camera changed before confirmation");}
  changes=c.poll(read,'a');
  if (changes.size()!=1 || changes[0].name!="depth.confidence_threshold" || changes[0].value!=120) {throw std::runtime_error("Numeric camera change failed");}
  values["depth.confidence_threshold"]=120; c.poll(read);
  values["height.max_m"]=.75;
  if (!c.poll(read).empty()) {throw std::runtime_error("External change unexpectedly reapplied");}
  c.setDraft("height.max_m","0.75");
  if (!c.poll(read,13).empty()) {throw std::runtime_error("External parameter sync failed");}
  c.setDraft("ground.enabled","2");
  c.setDraft("terrain.cell_m","0.08");
  if (!c.poll(read,13).empty()) {throw std::runtime_error("Invalid boolean allowed a partial batch");}
  c.setDraft("ground.enabled","1");
  changes=c.poll(read,13);
  if (changes.size()!=1 || changes[0].name!="terrain.cell_m") {throw std::runtime_error("Corrected numeric batch failed");}
  values["terrain.cell_m"]=.08; c.poll(read);
  c.setDraft("terrain.fill_enabled","0"); c.setDraft("terrain.fill_max_cells","4");
  changes=c.poll(read,13);
  if (changes.size()!=2) {throw std::runtime_error("Live hole controls missing");}
  values["terrain.fill_enabled"]=0;values["terrain.fill_max_cells"]=4;c.poll(read);
  c.setLocked(true);c.poll(read,9);c.poll(read,'5');
  if(!c.poll(read,13).empty() || c.editing())throw std::runtime_error("Locked controls accepted edit");
  c.setLocked(false);c.setDraft("terrain.fill_enabled","1");
  changes=c.poll(read,13);if(changes.size()!=1)throw std::runtime_error("Stop did not unlock controls");
  cv::waitKey(1); cv::destroyAllWindows();
  std::cout<<"GUI tests passed: numeric input, explicit apply, invalid batch rejection, external parameter synchronization\n";
}
