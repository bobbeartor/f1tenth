#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace multiclass_yolo {
inline std::string liveDistanceText(const nlohmann::json& value) {
  return value.is_null() ? "N/A":cv::format("%.3f",value.get<double>());
}
inline cv::Mat liveDistancePreview(const cv::Mat& image,const nlohmann::json& objects,
  const std::string& algorithm,bool stereo,double fps,double yolo_ms,double model_ms) {
  const int top=55,panel=500;
  cv::Mat shown(image.rows+top,image.cols+panel,CV_8UC3,cv::Scalar(20,20,20));
  image.copyTo(shown(cv::Rect(0,top,image.cols,image.rows)));
  const auto font=cv::FONT_HERSHEY_SIMPLEX;
  const auto title=cv::format("%s / %s | FPS %.1f | YOLO %.1f ms | Model %.3f ms",
    stereo ? "RGB + STEREO":"RGB ONLY",algorithm.c_str(),fps,yolo_ms,model_ms);
  cv::putText(shown,title,{8,22},font,.57,{0,230,230},1,cv::LINE_AA);
  cv::putText(shown,stereo ? "P = model / S = Stereo / D = P - S (m)":"P = model (m) / Stereo OFF",
    {8,44},font,.48,{210,210,210},1,cv::LINE_AA);
  const int visible_rows=std::max(1,(image.rows-20)/50);
  for(std::size_t i=0;i<objects.size();++i) {
    const auto& o=objects[i];const auto& b=o.at("bbox");
    const bool good_reference=!o.at("stereo_distance_m").is_null();
    const cv::Scalar color=o.at("distance_m").is_null() ? cv::Scalar(70,70,255):
      (stereo && !good_reference ? cv::Scalar(0,165,255):cv::Scalar(90,230,90));
    cv::Rect box(int(std::lround(b[0].get<double>())),int(std::lround(b[1].get<double>()))+top,
      std::max(1,int(std::lround(b[2].get<double>()))),std::max(1,int(std::lround(b[3].get<double>()))));
    cv::rectangle(shown,box,color,2);
    const auto identity=cv::format("#%d %s",int(i+1),o.at("class").get<std::string>().c_str());
    const auto values="P:"+liveDistanceText(o.at("distance_m"))+
      (stereo ? " S:"+liveDistanceText(o.at("stereo_distance_m")):"");
    cv::putText(shown,identity,{box.x,std::max(top+14,box.y-19)},font,.48,{0,0,0},3,cv::LINE_AA);
    cv::putText(shown,identity,{box.x,std::max(top+14,box.y-19)},font,.48,color,1,cv::LINE_AA);
    cv::putText(shown,values,{box.x,std::max(top+28,box.y-4)},font,.42,{0,0,0},3,cv::LINE_AA);
    cv::putText(shown,values,{box.x,std::max(top+28,box.y-4)},font,.42,color,1,cv::LINE_AA);
    if(int(i)<visible_rows) {
      const int y=top+22+int(i)*50;
      cv::putText(shown,identity+cv::format(" / conf %.2f",o.at("confidence").get<double>()),
        {image.cols+12,y},font,.51,color,1,cv::LINE_AA);
      std::string detail="P: "+liveDistanceText(o.at("distance_m"))+" m";
      if(stereo) {
        detail+=" | S: "+liveDistanceText(o.at("stereo_distance_m"))+" m";
        if(!o.at("prediction_minus_stereo_m").is_null())detail+=cv::format(" | D: %+.3f",o.at("prediction_minus_stereo_m").get<double>());
      }
      cv::putText(shown,detail,{image.cols+12,y+21},font,.43,{230,230,230},1,cv::LINE_AA);
    }
  }
  if(int(objects.size())>visible_rows)cv::putText(shown,
    cv::format("+%d more: all boxes/logs retained",int(objects.size())-visible_rows),
    {image.cols+12,shown.rows-8},font,.4,{190,190,190},1,cv::LINE_AA);
  return shown;
}
}
