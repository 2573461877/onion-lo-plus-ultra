#pragma once

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace traj {

// A PCD-compatible map point.  Mainstream consumers that request PointXYZ or
// PointXYZI can ignore the extra "label" field, while Onion-LO++ can restore
// its feature-class constraint when the map is loaded again.
struct EIGEN_ALIGN16 MapPointXYZIL {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t label;

  MapPointXYZIL()
      : intensity(0.0F), label(0U) {
    x = 0.0F;
    y = 0.0F;
    z = 0.0F;
    data[3] = 1.0F;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj

POINT_CLOUD_REGISTER_POINT_STRUCT(
    traj::MapPointXYZIL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, label, label))
