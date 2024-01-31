#ifndef PCMAP_H
#define PCMAP_H

#include <ros/ros.h>

#include <pcmap_updater/pcl_utils.hpp>

using namespace Bonxai;

class PCMAP {
 private:
  std::vector<Point3D> map_points;

 public:
  PCMAP() {
    ReadPointsFromPCD("/home/ro/Documents/pcd_files/office_val_183.pcd",
                      map_points);
    if (map_points.size() >= 1) {
      Point3D first = map_points[0];
      std::cout << first.x << " " << first.y << " " << first.z << "\n";
    }
  }
};

#endif