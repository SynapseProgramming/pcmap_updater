#ifndef PCMAP_H
#define PCMAP_H

#include <ros/ros.h>

#include <pcmap_updater/pcl_utils.hpp>
#include <pcmap_updater/probabilistic_map.hpp>

using namespace Bonxai;
using Vector3D = Eigen::Vector3d;

class PCMAP {
 private:
  std::vector<Eigen::Vector3d> map_points;
  ProbabilisticMap probMap;

 public:
  PCMAP() : probMap(0.1) {
    ReadPointsFromPCD("/home/ro/Documents/pcd_files/office_val_183.pcd",
                      map_points);

    // insert points into prob map
    for (Vector3D iter : map_points) {
      // Vector3D convpt = ConvertPoint(iter);
      probMap.addInitPoint(iter);
    }
    // load points back into a point cloud
  }
};

#endif