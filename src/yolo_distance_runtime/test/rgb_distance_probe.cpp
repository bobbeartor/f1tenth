#include "multiclass_yolo/rgb_distance_model.hpp"
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    multiclass_yolo::RgbDistanceModel model(argv[1]);std::string line;
    while(std::getline(std::cin,line)) {
      const auto r=nlohmann::json::parse(line);
      if(r.contains("camera_contract"))model.checkCamera(r.at("camera_contract").is_string() ? r.at("camera_contract").get<std::string>() : r.at("camera_contract").dump());
      const auto& b=r.at("bbox");
      auto p=model.predict(b.at(0),b.at(1),b.at(2),b.at(3),r.at("width"),r.at("height"),r.at("class_id"));
      nlohmann::json out={{"distance_m",nullptr},{"reason",p.reason}};
      if(p.distance)out["distance_m"]=*p.distance;
      std::cout<<out.dump()<<'\n';
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
