#include "roi_lidar/preview.hpp"
#include <iostream>
using namespace roi_lidar;
static void check(bool ok,const char * text) {if (!ok) {throw std::runtime_error(text);}}
int main()
{
  Options o;
  ViewOptions view;
  check(view.width_m==1.5 && view.forward_m==4.0,"New display defaults lost");
  view.width_m=1.5; view.forward_m=4.0; // Exercise the original configurable footprint.
  const auto layout=mapLayout(view,800);
  const auto origin=mapPixel(0,0,view,800),front=mapPixel(1,0,view,800),left=mapPixel(1,.5,view,800);
  check(front.y<origin.y && left.x<front.x,"BEV axes mirrored");
  check(view.width_m==1.5 && view.forward_m==4.0,"Requested BEV footprint lost");
  const auto near_left=mapPixel(0,.75,view,800), near_right=mapPixel(0,-.75,view,800);
  const auto far=mapPixel(4,0,view,800);
  check(std::abs(double(near_right.x-near_left.x)/(origin.y-far.y)-1.5/4.0)<1e-6,
    "BEV metric aspect ratio distorted");
  check(insideView(4,.75,view) && !insideView(4.01,0,view) && !insideView(1,.751,view),
    "BEV display boundary incorrect");
  BevProjector bev;
  // Down-facing pinhole: world ground -> center pixel; RGB is already undistorted.
  point_cloud::RigidTransform t{{1,0,0,0,-1,0,0,0,-1},{1,0,1}};
  point_cloud::Intrinsics k{20,20,50,50};
  bev.configure(100,100,k,t,view,800);
  cv::Mat rgb(100,100,CV_8UC3,cv::Scalar(10,80,160));
  const auto background=bev.render(rgb);
  check(background.at<cv::Vec3b>(front)==cv::Vec3b(10,80,160),"RGB ground projection misaligned");
  check(background.at<cv::Vec3b>(0,0)==cv::Vec3b(35,35,35),"Outside RGB FOV not masked");
  auto scan=emptyScan(o); scan.ranges[o.bins/2]=1;
  const auto overlay=mapPreview(scan,o,view,800,"TEST",background);
  check(overlay.at<cv::Vec3b>(front)==cv::Vec3b(30,85,225),"Obstacle not overlaid at metric position");
  auto distant=emptyScan(o); distant.ranges[o.bins/2]=6;
  // A lateral return outside +/-75cm is also hidden, even though the scan accepts it.
  distant.ranges[240]=1.5;
  const auto blank=mapPreview(emptyScan(o),o,view,800,"TEST",background);
  const auto clipped=mapPreview(distant,o,view,800,"TEST",background);
  check(cv::norm(blank,clipped,cv::NORM_INF)==0,"Out-of-view scan returns leaked into BEV");
  auto far_options=o; far_options.max_range=20;
  const auto wider_scan=mapPreview(scan,far_options,view,800,"TEST",background);
  check(wider_scan.size()==overlay.size() && wider_scan.at<cv::Vec3b>(front)==cv::Vec3b(30,85,225),
    "Changing LiDAR range changed the display scale");
  // Non-symmetric rotation and translated camera: verify inverse ground mapping.
  const auto tilted=point_cloud::vehicleFromRgb(-.16,.03,.2,3,15,7);
  const point_cloud::Intrinsics intrinsics{180,180,160,100};
  bev.configure(320,200,intrinsics,tilted,view,800);
  cv::Mat gradient(200,320,CV_8UC3);
  for (int row=0;row<200;++row) {for (int col=0;col<320;++col) {
    gradient.at<cv::Vec3b>(row,col)={static_cast<unsigned char>(col/2),static_cast<unsigned char>(row),100};
  }}
  const auto pixel=mapPixel(1,.2,view,800);
  const double scale=layout.scale;
  const point_cloud::Vector3 delta{(layout.origin_y-pixel.y)/scale-tilted.translation[0],
    (layout.center_x-pixel.x)/scale-tilted.translation[1],-tilted.translation[2]};
  point_cloud::Vector3 optical{};
  for (int axis=0;axis<3;++axis) {for (int j=0;j<3;++j) {
    optical[axis]+=tilted.rotation[j*3+axis]*delta[j];
  }}
  const double expected_u=intrinsics.fx*optical[0]/optical[2]+intrinsics.cx;
  const double expected_v=intrinsics.fy*optical[1]/optical[2]+intrinsics.cy;
  check(expected_u>0 && expected_u<319 && expected_v>0 && expected_v<199,"Invalid test geometry");
  const auto color=bev.render(gradient).at<cv::Vec3b>(pixel);
  check(std::abs(color[0]-expected_u/2)<2 && std::abs(color[1]-expected_v)<2,
    "Tilted/translated RGB BEV mapping incorrect");
  cv::Mat depth(40,60,CV_16UC1,cv::Scalar(1000));
  auto roi=roiPreview(depth,o,true);
  check(roi.rows==40 && roi.cols==60 && roi.type()==CV_8UC3,"Depth ROI preview malformed");
  cv::Mat gray(40,60,CV_8UC1,cv::Scalar(100));
  check(roiPreview(gray,o,false).type()==CV_8UC3,"Camera ROI preview malformed");
  // All XY endpoints survive, including multiple depths along the same scan bin.
  auto dense=emptyScan(o);
  dense.surface_points={{1,.12,.2},{2,.12,.3},{3,.12,1.5},{1.5,.12,.0}};
  TerrainOptions terrain;terrain.min_obstacle_samples=2;terrain.cell_m=.05;terrain.below_ground_m=.05; // Fixed color fixture.
  const auto grid=terrainGrid(dense,o,view,terrain);
  auto topdown=blank.clone();
  terrainOverlay(topdown,grid,dense,o,view,800,false,true,.4,1,.3);
  check(topdown.at<cv::Vec3b>(mapPixel(1,.12,view,800))==cv::Vec3b(0,0,255),"First obstacle endpoint lost");
  check(topdown.at<cv::Vec3b>(mapPixel(2,.12,view,800))==cv::Vec3b(0,0,255),"Farther endpoint wrongly reduced to nearest scan");
  check(topdown.at<cv::Vec3b>(mapPixel(3,.12,view,800))!=cv::Vec3b(0,0,255),"Height cap ignored");
  check(topdown.at<cv::Vec3b>(mapPixel(1.5,.12,view,800))!=cv::Vec3b(0,0,255),"Floor leaked into obstacle layer");
  auto colors=blank.clone();
  auto measured=emptyScan(o); measured.surface_points={{1.01,.12,0},{1.02,.12,0},{2.01,.12,.2},{2.02,.12,.2}};
  const auto classified=terrainGrid(measured,o,view,terrain);
  terrainOverlay(colors,classified,measured,o,view,800,true,false,1,1,.3);
  check(colors.at<cv::Vec3b>(mapPixel(1.025,.125,view,800))==cv::Vec3b(65,190,65),"Ground color incorrect");
  check(colors.at<cv::Vec3b>(mapPixel(2.025,.125,view,800))==cv::Vec3b(45,55,220),"Hazard color incorrect");
  check(colors.at<cv::Vec3b>(mapPixel(3.025,.125,view,800))==cv::Vec3b(90,90,90),"Unknown color incorrect");
  BevProjector clean;clean.configureLane(100,100,k,t);
  const auto clean_image=clean.render(rgb);
  check(clean_image.rows==300 && clean_image.cols==120,"Lane input includes preview padding");
  check(clean_image.at<cv::Vec3b>(199,59)==cv::Vec3b(10,80,160),"Metric lane pixel mapping lost");
  clean.configureLane(320,200,intrinsics,tilted);
  const point_cloud::Vector3 lane_delta{1.005-tilted.translation[0],.205-tilted.translation[1],-tilted.translation[2]};
  point_cloud::Vector3 lane_optical{};
  for(int axis=0;axis<3;++axis)for(int j=0;j<3;++j)lane_optical[axis]+=tilted.rotation[j*3+axis]*lane_delta[j];
  const double lane_u=intrinsics.fx*lane_optical[0]/lane_optical[2]+intrinsics.cx;
  const double lane_v=intrinsics.fy*lane_optical[1]/lane_optical[2]+intrinsics.cy;
  const auto lane_color=clean.render(gradient).at<cv::Vec3b>(199,39);
  check(std::abs(lane_color[0]-lane_u/2)<2 && std::abs(lane_color[1]-lane_v)<2,"Clean lane BEV axes/centers mismatched");
  std::cout<<"RGB BEV preview tests passed: metric projection, axes, FOV mask, obstacle overlay, ROI\n";
}
