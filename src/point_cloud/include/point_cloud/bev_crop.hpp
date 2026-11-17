#pragma once

#include "point_cloud/depth_projection.hpp"

#include <array>

namespace point_cloud
{
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;
struct RigidTransform
{
  Matrix3 rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vector3 translation{0, 0, 0};
};

inline Vector3 rotate(const Matrix3 & r, const Vector3 & p)
{
  return {r[0]*p[0] + r[1]*p[1] + r[2]*p[2],
    r[3]*p[0] + r[4]*p[1] + r[5]*p[2], r[6]*p[0] + r[7]*p[1] + r[8]*p[2]};
}

// compose(a, b) maps a point through b first, then a.
inline RigidTransform compose(const RigidTransform & a, const RigidTransform & b)
{
  RigidTransform result;
  result.rotation.fill(0);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      for (int k = 0; k < 3; ++k) {
        result.rotation[row*3+col] += a.rotation[row*3+k] * b.rotation[k*3+col];
      }
    }
  }
  result.translation = rotate(a.rotation, b.translation);
  for (int k = 0; k < 3; ++k) {result.translation[k] += a.translation[k];}
  return result;
}

// Same convention as bev_processor::mountRotationVehicleFromCamera:
// Rz(yaw) * Ry(downward pitch) * Rx(roll) * optical_to_vehicle.
inline RigidTransform vehicleFromRgb(double x, double y, double height,
  double roll_deg, double pitch_down_deg, double yaw_deg)
{
  for (const auto value : {x, y, height, roll_deg, pitch_down_deg, yaw_deg}) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite BEV mount");}
  }
  if (height <= 0) {throw std::invalid_argument("BEV camera height must be positive");}
  constexpr double radians = 3.14159265358979323846 / 180.0;
  const double cr = std::cos(roll_deg*radians), sr = std::sin(roll_deg*radians);
  const double cp = std::cos(pitch_down_deg*radians), sp = std::sin(pitch_down_deg*radians);
  const double cy = std::cos(yaw_deg*radians), sy = std::sin(yaw_deg*radians);
  const RigidTransform optical{{0,0,1, -1,0,0, 0,-1,0}, {0,0,0}};
  const RigidTransform rx{{1,0,0, 0,cr,-sr, 0,sr,cr}, {0,0,0}};
  const RigidTransform ry{{cp,0,sp, 0,1,0, -sp,0,cp}, {0,0,0}};
  const RigidTransform rz{{cy,-sy,0, sy,cy,0, 0,0,1}, {x,y,height}};
  return compose(rz, compose(ry, compose(rx, optical)));
}

// Accept DepthAI's matrix containers; frame metadata uses meters while EEPROM
// CameraExtrinsics defaults to centimeters. Never infer the unit from magnitude.
template<typename Matrix>
RigidTransform calibratedTransform(const Matrix & matrix, double translation_scale)
{
  if (matrix.size() < 3) {throw std::runtime_error("Missing calibration transform");}
  RigidTransform t;
  for (int row = 0; row < 3; ++row) {
    if (matrix[row].size() < 4) {throw std::runtime_error("Missing calibration translation");}
    for (int col = 0; col < 3; ++col) {t.rotation[row*3+col] = matrix[row][col];}
    t.translation[row] = matrix[row][3] * translation_scale;
    if (!std::isfinite(t.translation[row])) {throw std::runtime_error("Invalid calibration translation");}
  }
  const auto & r = t.rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double product = 0;
      for (int k = 0; k < 3; ++k) {product += r[row*3+k] * r[col*3+k];}
      if (!std::isfinite(product) || std::abs(product - (row == col ? 1.0 : 0.0)) > 0.02) {
        throw std::runtime_error("Calibration is not an orthonormal rotation");
      }
    }
  }
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
    r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  if (std::abs(determinant - 1.0) > 0.02) {throw std::runtime_error("Reflected calibration rotation");}
  return t;
}

struct BevOptions
{
  bool enabled{true};
  double x_min_m{0.0}, x_max_m{3.0}, y_min_m{-0.6}, y_max_m{0.6};
};
inline void validateBev(const BevOptions & o)
{
  if (!std::isfinite(o.x_min_m) || !std::isfinite(o.x_max_m) ||
    !std::isfinite(o.y_min_m) || !std::isfinite(o.y_max_m) ||
    o.x_min_m >= o.x_max_m || o.y_min_m >= o.y_max_m)
  {
    throw std::invalid_argument("BEV bounds must be finite with min < max");
  }
}

// Preserve Z and organized slots. This selects the BEV ground footprint, not
// a flattened cloud or an image-pixel crop. Upper bounds are exclusive.
inline void transformAndCrop(Cloud & cloud, const RigidTransform & t, const BevOptions & o)
{
  validateBev(o);
  cloud.valid_points = 0;
  for (std::size_t j = 0; j < cloud.xyz.size(); j += 3) {
    Vector3 p{cloud.xyz[j], cloud.xyz[j+1], cloud.xyz[j+2]};
    p = rotate(t.rotation, p);
    for (int k = 0; k < 3; ++k) {p[k] += t.translation[k];}
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
      (o.enabled && (p[0] < o.x_min_m || p[0] >= o.x_max_m ||
      p[1] < o.y_min_m || p[1] >= o.y_max_m)))
    {
      for (int k = 0; k < 3; ++k) {cloud.xyz[j+k] = std::numeric_limits<float>::quiet_NaN();}
    } else {
      for (int k = 0; k < 3; ++k) {cloud.xyz[j+k] = static_cast<float>(p[k]);}
      ++cloud.valid_points;
    }
  }
}
}  // namespace point_cloud
