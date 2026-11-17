#pragma once
#include <nlohmann/json.hpp>
#include "multiclass_yolo/distance_quality.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <vector>

namespace multiclass_yolo {
using Json = nlohmann::json;
class DistanceModel {
  Json model_;
  struct Layer {std::vector<std::vector<double>> weights; std::vector<double> bias;};
  std::vector<Layer> layers_;
 public:
  explicit DistanceModel(const std::string& path) {
    std::ifstream f(path); if(!f) throw std::runtime_error("Distance model missing"); f>>model_;
    if(model_.at("version")!=1 || model_.at("distance_definition")!="rgb_optical_z_m" ||
       model_.at("classes")!=Json({"con","truck","sedan","traffic_light"}) || model_.at("layers").size()!=3)
      throw std::runtime_error("Invalid distance model contract");
    const int sizes[]{9,64,32,1};
    for(int i=0;i<3;++i) {
      const auto& l=model_.at("layers")[i];
      if(l.at("weight").size()!=std::size_t(sizes[i+1]) || l.at("bias").size()!=std::size_t(sizes[i+1]))
        throw std::runtime_error("Invalid MLP shape");
      for(const auto& row:l.at("weight")) {
        if(row.size()!=std::size_t(sizes[i])) throw std::runtime_error("Invalid MLP shape");
        for(const auto& v:row) if(!std::isfinite(v.get<double>())) throw std::runtime_error("Invalid weight");
      }
      for(const auto& v:l.at("bias")) if(!std::isfinite(v.get<double>())) throw std::runtime_error("Invalid bias");
      layers_.push_back({l.at("weight").get<std::vector<std::vector<double>>>(),
        l.at("bias").get<std::vector<double>>()});
    }
  }
  void checkCamera(const std::string& contract) const {
    if(model_.at("camera_contract")!=contract) throw std::runtime_error("MLP camera geometry mismatch");
  }
  std::optional<double> predict(std::vector<double> f,int cls) const {
    if(f.size()!=9) return {};
    const auto& supported=model_.at("supported_classes");
    if(std::find(supported.begin(),supported.end(),cls)==supported.end()) return {};
    const auto& domain=model_.at("domains").at(std::to_string(cls));
    for(int i=0;i<5;++i) if(!std::isfinite(f[i]) || f[i]<domain.at("min")[i].get<double>() ||
      f[i]>domain.at("max")[i].get<double>()) return {};
    for(int l=0;l<3;++l) {
      const auto& layer=layers_[l]; std::vector<double> out;
      for(std::size_t j=0;j<layer.bias.size();++j) {
        double v=layer.bias[j];
        for(std::size_t k=0;k<f.size();++k) v+=layer.weights[j][k]*f[k];
        out.push_back(l<2 ? std::max(0.,v):v);
      }
      f=std::move(out);
    }
    if(!std::isfinite(f[0]) || f[0]<=0 || f[0]<domain.at("distance_min").get<double>() ||
      f[0]>domain.at("distance_max").get<double>()) return {};
    return f[0];
  }
};
}
