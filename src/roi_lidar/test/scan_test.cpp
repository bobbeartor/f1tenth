#include "roi_lidar/scan.hpp"
#include "roi_lidar/freshness.hpp"
#include <iostream>
#include <functional>
using namespace roi_lidar;
static void check(bool ok,const char * text) {if (!ok) {throw std::runtime_error(text);}}
static void rejects(const std::function<void()> & f) {
  bool bad=false; try {f();} catch(const std::exception &) {bad=true;} check(bad,"Invalid input accepted");
}
int main()
{
  check(overlayAllowed(1.0,1.04,1.05,.25,.08),"Fresh RGB/depth incorrectly suppressed");
  check(!overlayAllowed(1.0,1.1,1.11,.25,.08),"Unsynchronized overlay admitted");
  check(!overlayAllowed(1.0,1.02,1.4,.25,.08),"Stale overlay admitted");
  check(!overlayAllowed(2.0,2.02,1.0,.25,.08),"Future frames admitted");
  Options o; o.ransac.enabled=false; o.outlier.enabled=false; o.max_height=1.0; o.min_samples=1; // Preserve the synthetic elevated samples.
  o.ground_distance_per_meter=0; o.pixel_stride=1; o.bins=141;
  const int width=5,height=5,stride=14; // padded RAW16 rows
  std::vector<std::uint8_t> bytes(stride*height,0);
  auto set=[&](int u,int v,int mm) {bytes[v*stride+u*2]=mm&255; bytes[v*stride+u*2+1]=mm>>8;};
  point_cloud::Intrinsics k{10,10,2,2};
  const auto mount=point_cloud::vehicleFromRgb(0,0,.2,0,0,0);
  Projector p;
  p.configure(width,height,k,mount,o);
  // 2m bottom row hits floor at z=0; central row is obstacle at camera height.
  set(2,3,2000); set(2,2,2000); set(1,2,4000); set(3,2,8000);
  auto scan=p.project(bytes.data(),bytes.size(),stride);
  check(scan.surface_points.size()==4,"Measured floor endpoints lost before terrain classification");
  check(scan.ground_removed==1 && scan.valid_bins==3,"Floor/obstacles not separated");
  check(std::abs(scan.ranges[70]-2)<1e-6,"Central surface range wrong");
  check(std::isfinite(scan.ranges[76]) && std::isfinite(scan.ranges[64]),"Left/right axes incorrect");
  check(std::isnan(scan.ranges[0]),"Unobserved bin marked free");
  set(2,2,0);
  scan=p.project(bytes.data(),bytes.size(),stride);
  check(std::isnan(scan.ranges[70]),"Old obstacle retained");
  o.ground_enabled=false; p.configure(width,height,k,mount,o);
  check(std::isfinite(p.project(bytes.data(),bytes.size(),stride).ranges[70]),"Ground toggle failed");
  o.ground_enabled=true;
  o.roi_x=.4; o.roi_y=.4; o.roi_width=.2; o.roi_height=.2;
  p.configure(width,height,k,mount,o);
  check(p.rayCount()==1 && p.project(bytes.data(),bytes.size(),stride).valid_bins==0,"ROI leaked other pixels");
  o.roi_x=o.roi_y=0; o.roi_width=o.roi_height=1;
  o.pixel_stride=2; p.configure(width,height,k,mount,o);
  check(p.rayCount()==9,"Stride sampling wrong");
  o.pixel_stride=1; o.min_samples=2;
  // Far samples in a bin must not support an isolated near outlier.
  bytes.assign(bytes.size(),0); set(2,2,2000); set(2,1,5000);
  p.configure(width,height,k,mount,o);
  const auto two_surfaces=p.project(bytes.data(),bytes.size(),stride);
  check(two_surfaces.valid_bins==0,"Distant points supported near outlier");
  check(two_surfaces.surface_points.size()==2,"XY points incorrectly collapsed to angular nearest return");
  set(2,1,2050);
  check(p.project(bytes.data(),bytes.size(),stride).valid_bins==1,"Same-range support lost");
  o.min_samples=1; o.max_range=12;
  bytes.assign(bytes.size(),0); set(2,2,10000);
  p.configure(width,height,k,mount,o);
  check(std::abs(p.project(bytes.data(),bytes.size(),stride).ranges[70]-10)<1e-5,"Old 3m range cap remains");
  o.max_range=9; p.configure(width,height,k,mount,o);
  check(p.project(bytes.data(),bytes.size(),stride).valid_bins==0,"Range limit ignored");
  rejects([&](){p.project(bytes.data(),3,stride);});
  rejects([&](){p.project(bytes.data(),bytes.size(),2);});
  rejects([&](){auto bad=o; bad.roi_x=.5; validate(bad);});
  rejects([&](){auto bad=o; bad.roi_width=0; validate(bad);});
  rejects([&](){auto bad=o; bad.max_range=std::numeric_limits<double>::quiet_NaN(); validate(bad);});
  rejects([&](){auto bad=o; bad.ground_distance=1; validate(bad);});
  rejects([&](){p.configure(width,height,{0,10,2,2},mount,o);});
  // Tilted mount: floor rays still remove the plane after full calibration transform.
  const auto tilted=point_cloud::vehicleFromRgb(-.16,.02,.2,3,15,7);
  o.max_range=12; bytes.assign(bytes.size(),0);
  for (int v=0;v<height;++v) {for (int u=0;u<width;++u) {
    const auto d=point_cloud::rotate(tilted.rotation,{(u-k.cx)/k.fx,(v-k.cy)/k.fy,1});
    if (d[2]<0) {const double depth=-tilted.translation[2]/d[2]; if (depth<20) {set(u,v,std::lround(depth*1000));}}
  }}
  p.configure(width,height,k,tilted,o); scan=p.project(bytes.data(),bytes.size(),stride);
  check(scan.depth_points>0 && scan.ground_removed==scan.depth_points && scan.valid_bins==0,"Tilted floor survived");
  std::cout<<"ROI scan tests passed: ground, ROI, range, tilt, axes, support, no history, invalid inputs\n";
}
