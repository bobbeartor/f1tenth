#include "point_cloud/bev_crop.hpp"
#include "bev_processor/bev_geometry.hpp"

#include <iostream>
#include <stdexcept>

int main()
{
  // Compare directly to the actual BEV implementation for combined roll/pitch/yaw.
  for (const double roll : {-12.0, 0.0, 8.0}) {
    for (const double pitch : {-5.0, 0.0, 22.0}) {
      for (const double yaw : {-20.0, 0.0, 15.0}) {
        const auto actual = point_cloud::vehicleFromRgb(-0.16,0,0.2,roll,pitch,yaw);
        const auto expected = bev_processor::mountRotationVehicleFromCamera(
          bev_processor::degToRad(roll), bev_processor::degToRad(pitch), bev_processor::degToRad(yaw));
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            if (std::abs(actual.rotation[row*3+col] - expected(row,col)) > 1e-12) {
              throw std::runtime_error("Point cloud mount differs from BEV rotation");
            }
          }
        }
      }
    }
  }
  std::cout << "BEV reference tests passed: 27 combined mount poses match bev_processor\n";
}
