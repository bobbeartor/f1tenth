#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace multiclass_yolo {
// Portable export of benchmark_v2 models. Inference needs neither Torch nor LightGBM.
class RgbDistanceModel {
  using Json=nlohmann::json;
  struct Layer {std::vector<std::vector<float>> weight; std::vector<float> bias;};
  struct Node {
    int feature{-1}, left{-1}, right{-1}; double value{0}, threshold{0};
    bool categorical{false}, default_left{false}, zero_missing{false}; std::vector<int> categories;
  };
  std::string algorithm_; Json camera_; std::vector<int> classes_, roots_;
  std::vector<Layer> layers_; std::vector<Node> nodes_;
  std::array<double,7> mean_{},scale_{}; double minimum_{0},maximum_{0};
  static double finite(const Json& v) {
    const auto n=v.get<double>(); if(!std::isfinite(n)) throw std::runtime_error("Nonfinite RGB distance model parameter"); return n;
  }
  int loadTree(const Json& tree,int depth=0) {
    if(depth>128) throw std::runtime_error("Excessive portable tree depth");
    Node n;
    if(tree.contains("leaf")) n.value=finite(tree.at("leaf"));
    else {
      n.feature=tree.at("feature").get<int>();
      if(n.feature<0 || n.feature>=8) throw std::runtime_error("Invalid distance tree feature");
      n.default_left=tree.at("default_left").get<bool>();
      const auto missing=tree.at("missing_type").get<std::string>();
      if(missing!="None" && missing!="NaN" && missing!="Zero") throw std::runtime_error("Unsupported missing-value policy");
      n.zero_missing=missing=="Zero";
      n.categorical=tree.contains("categories");
      if(n.categorical) n.categories=tree.at("categories").get<std::vector<int>>();
      else n.threshold=finite(tree.at("threshold"));
    }
    const auto index=static_cast<int>(nodes_.size()); nodes_.push_back(std::move(n));
    if(nodes_[index].feature>=0) {
      const int left=loadTree(tree.at("left"),depth+1),right=loadTree(tree.at("right"),depth+1);
      nodes_[index].left=left; nodes_[index].right=right;
    }
    return index;
  }
 public:
  struct Prediction {std::optional<double> distance; std::string reason;};
  explicit RgbDistanceModel(const std::string& filename) {
    std::ifstream f(filename); if(!f) throw std::runtime_error("RGB distance model missing: "+filename);
    Json m;f>>m;
    const Json names={"center_x","center_y","width","height","bottom_y","aspect_ratio","inverse_height","class_id"};
    if(m.at("version")!=3 || m.at("feature_names")!=names || m.at("distance_definition")!="rgb_optical_z_m")
      throw std::runtime_error("Expected portable RGB distance model version 3");
    algorithm_=m.at("algorithm").get<std::string>();camera_=m.at("camera_contract");
    classes_=m.at("classes").get<std::vector<int>>();
    if(classes_.empty()) throw std::runtime_error("Empty distance model class vocabulary");
    auto unique=classes_;std::sort(unique.begin(),unique.end());
    if(std::adjacent_find(unique.begin(),unique.end())!=unique.end()) throw std::runtime_error("Duplicate classes");
    minimum_=finite(m.at("prediction_range_m").at(0)); maximum_=finite(m.at("prediction_range_m").at(1));
    if(minimum_<=0 || maximum_<=minimum_) throw std::runtime_error("Invalid prediction range");
    if(algorithm_=="mlp") {
      if(m.at("mean").size()!=7 || m.at("scale").size()!=7 || m.at("layers").size()!=3)
        throw std::runtime_error("Invalid benchmark MLP metadata shape");
      for(int i=0;i<7;++i) {mean_[i]=finite(m.at("mean")[i]);scale_[i]=finite(m.at("scale")[i]);if(scale_[i]<=0)throw std::runtime_error("Invalid MLP scale");}
      const std::vector<std::size_t> sizes{7+classes_.size(),64,32,1};
      for(std::size_t i=0;i<3;++i) {
        const auto& layer=m.at("layers")[i];
        if(layer.at("bias").size()!=sizes[i+1] || layer.at("weight").size()!=sizes[i+1]) throw std::runtime_error("Invalid MLP output shape");
        Layer l;
        for(std::size_t j=0;j<sizes[i+1];++j) {
          l.bias.push_back(static_cast<float>(finite(layer.at("bias")[j])));
          if(layer.at("weight")[j].size()!=sizes[i]) throw std::runtime_error("Invalid MLP input shape");
          std::vector<float> row;for(const auto& v:layer.at("weight")[j])row.push_back(static_cast<float>(finite(v)));
          l.weight.push_back(std::move(row));
        }
        layers_.push_back(std::move(l));
      }
    } else if(algorithm_=="lightgbm") {
      for(const auto& tree:m.at("trees")) roots_.push_back(loadTree(tree));
      if(roots_.empty()) throw std::runtime_error("Empty tree ensemble");
    } else throw std::runtime_error("Unsupported portable distance algorithm");
  }
  const std::string& algorithm() const {return algorithm_;}
  void checkCamera(const std::string& contract) const {
    if(Json::parse(contract)!=camera_) throw std::runtime_error("RGB distance model camera geometry mismatch");
  }
  Prediction predict(double x,double y,double w,double h,double width,double height,int cls) const {
    if(!std::isfinite(x+y+w+h+width+height) || width<=0 || height<=0 || x<0 || y<0 || w<=0 || h<=0 || x+w>width+.001 || y+h>height+.001)
      return {{},"invalid_bbox"};
    if(std::find(classes_.begin(),classes_.end(),cls)==classes_.end())return {{},"unseen_class"};
    // Match the float32 feature contract used by both Python benchmark models.
    std::vector<float> f{float((x+w/2)/width),float((y+h/2)/height),float(w/width),float(h/height),
      float((y+h)/height),float(w/h),float(height/h),float(cls)};
    for(auto v:f)if(!std::isfinite(v))return {{},"invalid_features"};
    double distance=0;
    if(algorithm_=="mlp") {
      std::vector<float> a;for(int i=0;i<7;++i)a.push_back(float((double(f[i])-mean_[i])/scale_[i]));
      for(auto c:classes_)a.push_back(c==cls ? 1.F:0.F);
      for(std::size_t i=0;i<layers_.size();++i) {
        const auto& layer=layers_[i];std::vector<float> b;
        for(std::size_t j=0;j<layer.bias.size();++j) {
          float v=0;for(std::size_t k=0;k<a.size();++k)v+=a[k]*layer.weight[j][k];v+=layer.bias[j];
          b.push_back(i+1<layers_.size() ? std::max(0.F,v):v);
        }
        a=std::move(b);
      }
      distance=a[0];
    } else {
      for(auto root:roots_) {
        int index=root;
        while(nodes_[index].feature>=0) {
          const auto& n=nodes_[index];const double v=f[n.feature];bool left;
          if(!n.categorical && n.zero_missing && std::abs(v)<=1e-35)left=n.default_left;
          else if(n.categorical)left=std::find(n.categories.begin(),n.categories.end(),static_cast<int>(v))!=n.categories.end();
          else left=v<=n.threshold;
          index=left ? n.left:n.right;
        }
        distance+=nodes_[index].value;
      }
    }
    if(!std::isfinite(distance) || distance<=0 || distance<minimum_ || distance>maximum_)return {{},"prediction_outside_measurement_range"};
    return {distance,""};
  }
};
}
