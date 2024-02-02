#ifndef PCMAP_H
#define PCMAP_H

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcmap_updater/tf2_sensor_msgs.h>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcmap_updater/pcl_utils.hpp>
#include <pcmap_updater/probabilistic_map.hpp>

using namespace Bonxai;
using Vector3D = Eigen::Vector3d;

class PCMAP {
 private:
  std::vector<Eigen::Vector3d> map_points;
  ProbabilisticMap probMap;
  ros::NodeHandle nh_;
  ros::Publisher pc_pub_;
  ros::Subscriber pc_sub_;

  pcl::PointCloud<pcl::PointXYZ> pc;

  // transforms
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  bool ready;

  void scanCB(const sensor_msgs::PointCloud2ConstPtr& inp);

 public:
  PCMAP() : nh_("~"), probMap(0.1), ready(false), tfListener(tfBuffer) {
    // ReadPointsFromPCD("/home/ro/Documents/pcd_files/decathlon.pcd",
    // map_points);

    std::cout << "map points size: " << map_points.size() << "\n";
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("voxeled", 10, true);
    pc_sub_ = nh_.subscribe("/rslidar_points", 1, &PCMAP::scanCB, this);
    // insert points into prob map
    // for (Vector3D iter : map_points) {
    //   probMap.addInitPoint(iter);
    // }
    // load points back into a point cloud
    // std::vector<CoordT> points;
    // probMap.getOccupiedVoxels(points);
    // std::cout << "size: " << points.size() << "\n";

    // for (CoordT pts : points) {
    //   pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    //   pc.push_back(pclpt);
    // }
    // pcl::toROSMsg(pc, output_msg);
    // output_msg.header.frame_id = "map";
    ready = true;
  }
  void publish_new() {
    if (ready == false) return;

    // load points back into a point cloud
    sensor_msgs::PointCloud2 output_msg;

    std::vector<CoordT> points;
    probMap.getOccupiedVoxels(points);
    std::cout << "size: " << points.size() << "\n";

    for (CoordT pts : points) {
      pcl::PointXYZ pclpt(pts.x / 10.0, pts.y / 10.0, pts.z / 10.0);
      pc.push_back(pclpt);
    }
    pcl::toROSMsg(pc, output_msg);
    output_msg.header.frame_id = "map";
    pc_pub_.publish(output_msg);
  }
};

#endif