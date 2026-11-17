#include "roi_lidar/preview.hpp"
#include <opencv2/imgproc.hpp>
#include <iomanip>
#include <array>
#include <sstream>
namespace roi_lidar
{
MapLayout mapLayout(const ViewOptions & view, int size)
{
  validateView(view);
  if (size<240 || size>1600) {throw std::invalid_argument("Invalid BEV preview size");}
  const double scale=(size-160)/std::max(view.width_m,view.forward_m);
  const int plot_width=static_cast<int>(std::lround(view.width_m*scale));
  const int plot_height=static_cast<int>(std::lround(view.forward_m*scale));
  const int width=std::max(420,plot_width+80);
  const double center=width/2.0, bottom=80+view.forward_m*scale;
  const int left=static_cast<int>(std::lround(center-view.width_m*scale/2));
  return {scale,center,bottom,{width,plot_height+160},{left,80,plot_width+1,plot_height+1}};
}
cv::Point mapPixel(double x, double y, const ViewOptions & view, int size)
{
  const auto layout=mapLayout(view,size);
  return {static_cast<int>(std::lround(layout.center_x-y*layout.scale)),
    static_cast<int>(std::lround(layout.origin_y-x*layout.scale))};
}
cv::Mat mapPreview(const Scan & scan, const Options & o, const ViewOptions & view,
  int size, const std::string & status, const cv::Mat & background,
  const Clusters * clusters, bool show_raw_points)
{
  const auto layout=mapLayout(view,size);
  cv::Mat canvas(layout.canvas,CV_8UC3,cv::Scalar(245,245,245));
  if (!background.empty()) {
    if (background.size()!=layout.canvas || background.type()!=CV_8UC3) {
      throw std::invalid_argument("RGB BEV background does not match the display footprint");
    }
    background(layout.area).copyTo(canvas(layout.area));
  }
  const auto pixel=[&](double x,double y) {return mapPixel(x,y,view,size);};
  const double step=std::max(view.forward_m,view.width_m)<=6 ? 0.5 :
    (std::max(view.forward_m,view.width_m)<=15 ? 1.0 : 5.0);
  for (double x=0;x<=view.forward_m+1e-9;x+=step) {
    cv::line(canvas,pixel(x,-view.width_m/2),pixel(x,view.width_m/2),{170,170,170});
    std::ostringstream label; label<<std::fixed<<std::setprecision(step<1 ? 1 : 0)<<x<<"m";
    cv::putText(canvas,label.str(),pixel(x,-view.width_m/2)+cv::Point(5,4),
      cv::FONT_HERSHEY_SIMPLEX,.35,{90,90,90},1,cv::LINE_AA);
  }
  for (double y=std::ceil(-view.width_m/2/step)*step;y<=view.width_m/2;y+=step) {
    cv::line(canvas,pixel(0,y),pixel(view.forward_m,y),{170,170,170});
  }
  cv::rectangle(canvas,layout.area,{130,130,130});
  const auto origin=pixel(0,0);
  for (const double angle : {o.angle_min,o.angle_max}) {
    auto a=origin, b=pixel(o.max_range*std::cos(angle*radians),o.max_range*std::sin(angle*radians));
    if (cv::clipLine(layout.area,a,b)) {cv::line(canvas,a,b,{190,180,120},1,cv::LINE_AA);}
  }
  float closest=std::numeric_limits<float>::infinity();
  cv::Point nearest;
  auto plot=canvas(layout.area);
  if (!clusters || show_raw_points) {
    for (std::size_t i=0;i<scan.ranges.size();++i) {
      const float r=scan.ranges[i];
      if (!std::isfinite(r) || r<o.min_range || r>o.max_range) {continue;}
      const double a=(o.angle_min+i*(o.angle_max-o.angle_min)/(o.bins-1))*radians;
      const double x=r*std::cos(a), y=r*std::sin(a);
      if (!insideView(x,y,view)) {continue;}
      const auto p=pixel(x,y);
      cv::circle(plot,p-layout.area.tl(),clusters ? 2 : 3,
        clusters ? cv::Scalar(160,160,160) : cv::Scalar(30,85,225),-1,cv::LINE_AA);
      if (r<closest) {closest=r; nearest=p;}
    }
  }
  if (clusters) {
    const std::array<cv::Scalar,6> colors{{{60,180,255},{100,235,100},{255,180,80},
      {220,110,245},{90,235,235},{190,190,255}}};
    for (std::size_t id=0;id<clusters->objects.size();++id) {
      const auto & object=clusters->objects[id];
      const auto color=colors[id%colors.size()];
      std::vector<cv::Point> points;
      for (const auto bin:object.bins) {
        const double r=scan.ranges[bin];
        const double a=(o.angle_min+bin*(o.angle_max-o.angle_min)/(o.bins-1))*radians;
        points.push_back(pixel(r*std::cos(a),r*std::sin(a))-layout.area.tl());
      }
      // Outline of observed front surfaces only; never a claimed full footprint.
      if (points.size()>=3) {
        std::vector<cv::Point> hull; cv::convexHull(points,hull);
        cv::polylines(plot,std::vector<std::vector<cv::Point>>{hull},true,color,2,cv::LINE_AA);
      } else if (points.size()==2) {cv::line(plot,points[0],points[1],color,2,cv::LINE_AA);}
      for (const auto & point:points) {cv::circle(plot,point,3,color,-1,cv::LINE_AA);}
      if (insideView(object.x,object.y,view)) {
        const auto center=pixel(object.x,object.y)-layout.area.tl();
        cv::drawMarker(plot,center,color,cv::MARKER_CROSS,10,2,cv::LINE_AA);
        std::ostringstream label; label<<"#"<<id+1<<" "<<std::fixed<<std::setprecision(2)<<object.nearest_range<<"m";
        const cv::Point at(std::clamp(center.x+8,0,std::max(0,plot.cols-100)),std::clamp(center.y-8,15,std::max(15,plot.rows-4)));
        cv::putText(plot,label.str(),at,cv::FONT_HERSHEY_SIMPLEX,.4,{25,25,25},3,cv::LINE_AA);
        cv::putText(plot,label.str(),at,cv::FONT_HERSHEY_SIMPLEX,.4,color,1,cv::LINE_AA);
      }
    }
  }
  cv::arrowedLine(canvas,origin,origin+cv::Point(0,-22),{80,150,30},3,cv::LINE_AA);
  cv::putText(canvas,"FRONT AXLE | +X forward / +Y left",{20,layout.canvas.height-48},
    cv::FONT_HERSHEY_SIMPLEX,.4,{50,50,50},1,cv::LINE_AA);
  if (!clusters && std::isfinite(closest) && layout.area.width>=80 && layout.area.height>=20) {
    std::ostringstream label; label<<std::fixed<<std::setprecision(2)<<closest<<" m";
    const auto at=cv::Point(std::clamp(nearest.x+8,layout.area.x,std::max(layout.area.x,layout.area.br().x-80)),
      std::clamp(nearest.y-8,layout.area.y+15,layout.area.br().y-4));
    cv::putText(canvas,label.str(),at,cv::FONT_HERSHEY_SIMPLEX,.45,{245,245,245},3,cv::LINE_AA);
    cv::putText(canvas,label.str(),at,cv::FONT_HERSHEY_SIMPLEX,.45,{20,60,190},1,cv::LINE_AA);
  }
  std::ostringstream title; title<<"BEV "<<std::fixed<<std::setprecision(2)<<view.width_m<<"m wide x "<<view.forward_m<<"m forward";
  cv::putText(canvas,title.str(),{12,22},cv::FONT_HERSHEY_SIMPLEX,.45,{40,40,40},1,cv::LINE_AA);
  std::istringstream words(status); std::string word,line; int row=42;
  while (words>>word) {
    const auto next=line.empty() ? word : line+" "+word;
    if (!line.empty() && cv::getTextSize(next,cv::FONT_HERSHEY_SIMPLEX,.35,1,nullptr).width>layout.canvas.width-24) {
      cv::putText(canvas,line,{12,row},cv::FONT_HERSHEY_SIMPLEX,.35,{50,50,50},1,cv::LINE_AA);
      row+=15; line=word; if (row>72) {line.clear(); break;}
    } else {line=next;}
  }
  if (!line.empty()) {cv::putText(canvas,line,{12,row},cv::FONT_HERSHEY_SIMPLEX,.35,{50,50,50},1,cv::LINE_AA);}
  cv::putText(canvas,"Unobserved = UNKNOWN | Q: quit | R: full ROI",{12,layout.canvas.height-20},
    cv::FONT_HERSHEY_SIMPLEX,.35,{65,65,65},1,cv::LINE_AA);
  return canvas;
}
void BevProjector::configure(int width, int height, const point_cloud::Intrinsics & k,
  const point_cloud::RigidTransform & t, const ViewOptions & view, int size)
{
  if (width<1 || height<1 || size<240 || size>1600 || k.fx<=0 || k.fy<=0) {
    throw std::invalid_argument("Invalid RGB BEV geometry");
  }
  const auto layout=mapLayout(view,size);
  map_x_=cv::Mat(layout.canvas,CV_32F,cv::Scalar(-1));
  map_y_=cv::Mat(layout.canvas,CV_32F,cv::Scalar(-1));
  for (int row=0;row<map_x_.rows;++row) {
    for (int col=0;col<map_x_.cols;++col) {
      const double x=(layout.origin_y-row)/layout.scale, y=(layout.center_x-col)/layout.scale;
      if (!insideView(x,y,view)) {continue;}
      const point_cloud::Vector3 delta{x-t.translation[0],y-t.translation[1],-t.translation[2]};
      point_cloud::Vector3 camera{};
      for (int axis=0;axis<3;++axis) {
        for (int j=0;j<3;++j) {camera[axis]+=t.rotation[j*3+axis]*delta[j];}
      }
      if (camera[2]<=0) {continue;}
      const double u=k.fx*camera[0]/camera[2]+k.cx, v=k.fy*camera[1]/camera[2]+k.cy;
      if (u<0 || v<0 || u>=width-1 || v>=height-1) {continue;}
      map_x_.at<float>(row,col)=u; map_y_.at<float>(row,col)=v;
    }
  }
}
// Clean 120x300, 1cm/pixel, front-axle BEV for the existing lane model.
// Pixel centers match LaneResult's explicit metric conversion; no HUD or terrain.
void BevProjector::configureLane(int width,int height,const point_cloud::Intrinsics & k,
  const point_cloud::RigidTransform & t)
{
  if (width<1 || height<1 || k.fx<=0 || k.fy<=0) {throw std::invalid_argument("Invalid lane BEV camera");}
  map_x_=cv::Mat(300,120,CV_32F,cv::Scalar(-1));
  map_y_=cv::Mat(300,120,CV_32F,cv::Scalar(-1));
  for (int v=0;v<300;++v) {for (int u=0;u<120;++u) {
    const point_cloud::Vector3 delta{3-(v+.5)*.01-t.translation[0],
      .6-(u+.5)*.01-t.translation[1],-t.translation[2]};
    point_cloud::Vector3 camera{};
    for (int axis=0;axis<3;++axis) {for (int j=0;j<3;++j) {camera[axis]+=t.rotation[j*3+axis]*delta[j];}}
    if (camera[2]<=0) {continue;}
    const double x=k.fx*camera[0]/camera[2]+k.cx,y=k.fy*camera[1]/camera[2]+k.cy;
    if (x>=0 && y>=0 && x<width-1 && y<height-1) {map_x_.at<float>(v,u)=x; map_y_.at<float>(v,u)=y;}
  }}
}
cv::Mat BevProjector::render(const cv::Mat & bgr, const std::string & interpolation) const
{
  if (map_x_.empty() || bgr.type()!=CV_8UC3) {throw std::invalid_argument("Unconfigured RGB BEV preview");}
  int method=cv::INTER_LINEAR;
  if (interpolation=="nearest") {method=cv::INTER_NEAREST;}
  else if (interpolation=="linear") {method=cv::INTER_LINEAR;}
  else if (interpolation=="cubic") {method=cv::INTER_CUBIC;}
  else if (interpolation=="lanczos4") {method=cv::INTER_LANCZOS4;}
  else {throw std::invalid_argument("Unknown BEV interpolation: "+interpolation);}
  cv::Mat out;
  cv::remap(bgr,out,map_x_,map_y_,method,cv::BORDER_CONSTANT,cv::Scalar(35,35,35));
  return out;
}
cv::Mat roiPreview(const cv::Mat & input, const Options & o, bool depth)
{
  cv::Mat gray, image;
  if (depth) {
    input.convertTo(gray,CV_8U,255.0/o.max_depth/1000.0);
    cv::applyColorMap(gray,image,cv::COLORMAP_TURBO);
    image.setTo(cv::Scalar(0,0,0),input==0);
  } else {cv::cvtColor(input,image,cv::COLOR_GRAY2BGR);}
  const auto roi=imageRoi(image.cols,image.rows,o);
  cv::rectangle(image,{roi.x,roi.y,roi.width,roi.height},{40,240,40},2);
  cv::putText(image,"Drag ROI | R: reset",{10,22},cv::FONT_HERSHEY_SIMPLEX,.5,{40,255,40},1,cv::LINE_AA);
  return image;
}
void terrainOverlay(cv::Mat & image, const Terrain & terrain, const Scan & scan,
  const Options & o, const ViewOptions & view, int size, bool show_cells,
  bool show_points, double alpha, int point_radius, double vehicle_width_m)
{
  const auto layout=mapLayout(view,size);
  if (image.size()!=layout.canvas || image.type()!=CV_8UC3) {
    throw std::invalid_argument("Terrain overlay canvas mismatch");
  }
  if (show_cells && !terrain.cells.empty()) {
    auto tinted=image.clone();
    for (int row=layout.area.y;row<layout.area.br().y;++row) {
      for (int col=layout.area.x;col<layout.area.br().x;++col) {
        const double x=(layout.origin_y-row)/layout.scale;
        const double y=(layout.center_x-col)/layout.scale;
        const int gx=static_cast<int>(std::floor(x/terrain.cell_m));
        const int gy=static_cast<int>(std::floor((y-terrain.y_min)/terrain.cell_m));
        int state=-1; bool inferred=false;
        if (x>=0 && x<terrain.x_max && y>=terrain.y_min && y<terrain.y_max &&
          gx>=0 && gy>=0 && gx<terrain.width && gy<terrain.height) {
          const auto index=static_cast<std::size_t>(gy)*terrain.width+gx;
          state=terrain.cells[index];
          inferred=index<terrain.inferred.size() && terrain.inferred[index];
        }
        tinted.at<cv::Vec3b>(row,col)=state==0 ? (inferred ? cv::Vec3b(190,190,65) : cv::Vec3b(65,190,65)) :
          (state==100 ? cv::Vec3b(45,55,220) : cv::Vec3b(90,90,90));
      }
    }
    cv::addWeighted(tinted(layout.area),alpha,image(layout.area),1-alpha,0,image(layout.area));
  }
  auto plot=image(layout.area);
  if (show_points) {
    forEachObstacle(scan,o,[&](const auto& p){
      if (!insideView(p[0],p[1],view)) {return;}
      // Post-ground/outlier measured endpoints; no angular nearest-return collapse.
      const cv::Point pixel(static_cast<int>(std::lround(layout.center_x-p[1]*layout.scale)),
        static_cast<int>(std::lround(layout.origin_y-p[0]*layout.scale)));
      cv::circle(plot,pixel-layout.area.tl(),point_radius,
        {0,0,255},-1,cv::LINE_8);
    });
  }
  // Vehicle width reference at the origin only; no clearance computation.
  cv::line(plot,mapPixel(0,-vehicle_width_m/2,view,size)-layout.area.tl(),
    mapPixel(0,vehicle_width_m/2,view,size)-layout.area.tl(),{255,210,40},3);
  cv::rectangle(image,{0,image.rows-38,image.cols,38},{245,245,245},-1);
  cv::putText(image,"GREEN floor | CYAN inferred | RED hazard | GRAY unknown",{12,image.rows-23},
    cv::FONT_HERSHEY_SIMPLEX,.34,{45,45,45},1,cv::LINE_AA);
  cv::putText(image,"XY points | width reference | Q quit / R ROI / A depth",{12,image.rows-8},
    cv::FONT_HERSHEY_SIMPLEX,.32,{45,45,45},1,cv::LINE_AA);
}
}
