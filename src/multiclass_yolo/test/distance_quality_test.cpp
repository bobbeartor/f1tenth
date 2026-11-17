#include "multiclass_yolo/distance_quality.hpp"
#include <stdexcept>
static void require(bool ok) {if(!ok) throw std::runtime_error("distance quality regression");}
int main() {
  using namespace multiclass_yolo;
  auto f=distanceFeatures(20,10,40,20,100,50,2);
  require(f.size()==9 && std::abs(f[0]-.4)<1e-9 && f[7]==1);
  bool rejected=false;
  try {distanceFeatures(-1,0,10,10,100,100,0);} catch(const std::invalid_argument&) {rejected=true;}
  require(rejected);
  DepthQuality q;
  require(!q.extract({},100));
  require(!q.extract(std::vector<double>(19,2),100));
  require(!q.extract(std::vector<double>(20,2),100));
  require(std::abs(*q.extract(std::vector<double>(40,2),100)-2)<1e-9);
  std::vector<double> mixed(30,2); mixed.insert(mixed.end(),30,5);
  require(!q.extract(mixed,100));
  std::vector<double> small_outliers(50,2);small_outliers.push_back(8);
  require(std::abs(*q.extract(small_outliers,100)-2)<1e-9);
  require(median({1,3,8,10})==5.5);
}
