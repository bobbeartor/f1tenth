#pragma once
#include <point_cloud/bev_crop.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>
namespace roi_lidar
{
struct RansacOptions
{
  bool enabled{true};
  double distance_m{0.06}, max_range_m{3.0}, max_tilt_deg{10.0}, max_offset_m{0.08};
  double min_inlier_ratio{0.50};
  int max_iterations{60}, max_samples{700};
};
struct OutlierOptions
{
  bool enabled{true};
  double radius_m{0.08};
  int min_neighbors{8}; // Other non-ground points, excluding the query point.
};
inline void validateFilters(const RansacOptions & r,const OutlierOptions & o)
{
  for(double v:{r.distance_m,r.max_range_m,r.max_tilt_deg,r.max_offset_m,r.min_inlier_ratio,o.radius_m})
    if(!std::isfinite(v))throw std::invalid_argument("Ground/outlier parameters must be finite");
  if(r.distance_m<.002 || r.distance_m>.10 || r.max_range_m<.2 || r.max_range_m>12 ||
    r.max_tilt_deg<.1 || r.max_tilt_deg>30 || r.max_offset_m<0 || r.max_offset_m>.30 ||
    r.min_inlier_ratio<.1 || r.min_inlier_ratio>1 || r.max_iterations<1 || r.max_iterations>300 ||
    r.max_samples<30 || r.max_samples>4000 || o.radius_m<.01 || o.radius_m>.5 || o.min_neighbors<1 || o.min_neighbors>64)
    throw std::invalid_argument("RANSAC: distance .002..0.1m, range .2..12m, tilt .1..30deg, offset 0..0.3m, ratio .1..1, iterations 1..300, samples 30..4000; outlier: radius .01..0.5m, neighbors 1..64");
}
struct GroundPlane
{
  std::array<double,4> coefficients{0,0,1,0}; // Normalized, positive Z; in startup vehicle coordinates.
  bool detected{false};
  std::size_t candidates{0}, samples{0}, inliers{0};
};
inline double planeDistance(const std::array<double,4>& n,const point_cloud::Vector3& p)
{return n[0]*p[0]+n[1]*p[1]+n[2]*p[2]+n[3];}
inline GroundPlane fitGround(const std::vector<point_cloud::Vector3>& points,const RansacOptions& o)
{
  GroundPlane result;
  if(!o.enabled)return result;
  const double max_r2=o.max_range_m*o.max_range_m;
  const double min_nz=std::cos(o.max_tilt_deg*3.14159265358979323846/180.0);
  const double band=o.max_offset_m+std::tan(o.max_tilt_deg*3.14159265358979323846/180.0)*o.max_range_m+o.distance_m;
  std::vector<point_cloud::Vector3> pool;pool.reserve(o.max_samples);
  std::uint32_t rng=0x9635a7u;
  const auto random=[&](){rng=1664525u*rng+1013904223u;return rng;};
  for(const auto& p:points){
    if(p[0]*p[0]+p[1]*p[1]>max_r2 || std::abs(p[2])>band)continue;
    ++result.candidates;
    if(pool.size()<static_cast<std::size_t>(o.max_samples))pool.push_back(p);
    else {const auto slot=random()%result.candidates;if(slot<pool.size())pool[slot]=p;}
  }
  result.samples=pool.size();
  if(pool.size()<30)return result;
  struct Support {std::size_t count{0};double error{0},xmin{1e9},xmax{-1e9},ymin{1e9},ymax{-1e9};};
  const auto evaluate=[&](const std::array<double,4>& plane){
    Support s;
    for(const auto& p:pool){const double d=std::abs(planeDistance(plane,p));if(d>o.distance_m)continue;
      ++s.count;s.error+=d;s.xmin=std::min(s.xmin,p[0]);s.xmax=std::max(s.xmax,p[0]);
      s.ymin=std::min(s.ymin,p[1]);s.ymax=std::max(s.ymax,p[1]);}
    return s;
  };
  const auto supported=[&](const Support& s){return s.count>=30 && s.count>=o.min_inlier_ratio*pool.size() && s.xmax-s.xmin>=.25 && s.ymax-s.ymin>=.20;};
  const auto plausible=[&](const std::array<double,4>& n){return n[2]>=min_nz && std::abs(n[3])<=o.max_offset_m;};
  Support best;std::array<double,4> plane{0,0,1,0};int budget=o.max_iterations;
  for(int iteration=0;iteration<budget;++iteration){
    const auto& a=pool[random()%pool.size()];const auto& b=pool[random()%pool.size()];const auto& c=pool[random()%pool.size()];
    const point_cloud::Vector3 u{b[0]-a[0],b[1]-a[1],b[2]-a[2]},v{c[0]-a[0],c[1]-a[1],c[2]-a[2]};
    std::array<double,4> n{u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0],0};
    double length=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);if(length<1e-9)continue;
    if(n[2]<0)length=-length;
    for(int k=0;k<3;++k)n[k]/=length;
    n[3]=-(n[0]*a[0]+n[1]*a[1]+n[2]*a[2]);if(!plausible(n))continue;
    const auto support=evaluate(n);
    if(!supported(support) || (support.count<best.count) || (support.count==best.count && support.error>=best.error))continue;
    best=support;plane=n;
    const double w=double(best.count)/pool.size(),miss=1-w*w*w;
    const int required=miss<1e-9 ? 1 : int(std::ceil(std::log(.01)/std::log(miss)));
    budget=std::min(budget,std::max(8,required));
  }
  if(!supported(best))return result;
  // Refine z=ax+by+c on the winning near-horizontal inliers (small 2x2 solve).
  point_cloud::Vector3 mean{};std::size_t count=0;
  for(const auto& p:pool)if(std::abs(planeDistance(plane,p))<=o.distance_m){++count;for(int k=0;k<3;++k)mean[k]+=p[k];}
  for(double& v:mean)v/=count;
  double xx=0,xy=0,yy=0,xz=0,yz=0;
  for(const auto& p:pool)if(std::abs(planeDistance(plane,p))<=o.distance_m){
    const double x=p[0]-mean[0],y=p[1]-mean[1],z=p[2]-mean[2];xx+=x*x;xy+=x*y;yy+=y*y;xz+=x*z;yz+=y*z;}
  const double determinant=xx*yy-xy*xy;
  if(determinant>1e-10){
    const double a=(xz*yy-yz*xy)/determinant,b=(yz*xx-xz*xy)/determinant,c=mean[2]-a*mean[0]-b*mean[1];
    const double length=std::sqrt(a*a+b*b+1);
    const std::array<double,4> refined{-a/length,-b/length,1/length,-c/length};
    if(plausible(refined)){const auto support=evaluate(refined);if(supported(support)){plane=refined;best=support;}}
  }
  result.coefficients=plane;result.detected=true;result.inliers=best.count;return result;
}
struct IndexedPoint {point_cloud::Vector3 physical;std::size_t index;};
inline std::vector<std::uint8_t> supportedPoints(const std::vector<IndexedPoint>& points,const OutlierOptions& o)
{
  std::vector<std::uint8_t> keep(points.size(),1);if(!o.enabled || points.empty())return keep;
  struct Key {int x,y,z;bool operator==(const Key& b)const{return x==b.x && y==b.y && z==b.z;}};
  struct Hash {std::size_t operator()(const Key& k)const{
    // Mix signed voxel coordinates; plain XOR clusters diagonal/planar keys.
    std::uint64_t h=std::uint32_t(k.x)*73856093ull ^ std::uint32_t(k.y)*19349663ull ^ std::uint32_t(k.z)*83492791ull;
    h^=h>>33;h*=0xff51afd7ed558ccdull;h^=h>>33;
    return static_cast<std::size_t>(h);
  }};
  const auto key=[&](const point_cloud::Vector3& p){return Key{int(std::floor(p[0]/o.radius_m)),int(std::floor(p[1]/o.radius_m)),int(std::floor(p[2]/o.radius_m))};};
  std::unordered_map<Key,int,Hash> heads;heads.reserve(std::min<std::size_t>(points.size(),65536));
  std::vector<int> next(points.size(),-1);
  for(std::size_t i=0;i<points.size();++i){const auto k=key(points[i].physical);const auto found=heads.find(k);if(found!=heads.end())next[i]=found->second;heads[k]=int(i);}
  const double radius2=o.radius_m*o.radius_m;
  for(std::size_t i=0;i<points.size();++i){
    const auto center=key(points[i].physical);int support=0;
    for(int z=-1;z<=1 && support<o.min_neighbors;++z)for(int y=-1;y<=1 && support<o.min_neighbors;++y)for(int x=-1;x<=1 && support<o.min_neighbors;++x){
      const auto found=heads.find({center.x+x,center.y+y,center.z+z});if(found==heads.end())continue;
      for(int j=found->second;j>=0 && support<o.min_neighbors;j=next[j]){
        if(std::size_t(j)==i)continue;const auto& a=points[i].physical;const auto& b=points[j].physical;
        const double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];if(dx*dx+dy*dy+dz*dz<=radius2)++support;
      }
    }
    keep[i]=support>=o.min_neighbors;
  }
  return keep;
}
}
