#ifndef PCMAP_H
#define PCMAP_H

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcmap_updater/Save.h>
#include <pcmap_updater/tf2_sensor_msgs.h>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcmap_updater/pcl_utils.hpp>
#include <pcmap_updater/probabilistic_map.hpp>

class PCMAP {
 private:
  std::vector<Eigen::Vector3d> map_points;
  Bonxai::ProbabilisticMap probMap;

  ros::NodeHandle nh_;
  ros::Publisher pc_pub_;
  ros::Subscriber pc_sub_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer save_server_;

  // transforms
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::string mapFrameName;
  std::string scanFrameName;
  std::string scanTopicName;
  std::string odomTopicName;
  std::string pcTopicName;
  std::string loadPcdPath;
  std::string savePcdPath;

  double maxDistance;
  double maxLinearV;
  double maxAngularV;

  bool ready;
  bool isMoving;

  void scanCB(const sensor_msgs::PointCloud2ConstPtr &inp);

  void odomCB(const nav_msgs::OdometryConstPtr &inp);

 public:
  PCMAP()
      : nh_("~"),
        probMap(0.1),
        ready(false),
        tfListener(tfBuffer),
        mapFrameName("map"),
        scanFrameName("lidar_3d_straight"),
        scanTopicName("/rslidar_points"),
        odomTopicName("/odom"),
        pcTopicName("pcmap"),
        maxDistance(100.0),
        maxLinearV(0.02),
        maxAngularV(0.02),
        loadPcdPath("/home/ro/Documents/pcd_files/decathlon.pcd"),
        savePcdPath("/home/ro/Documents/test_save/test.pcd")

  {
    nh_.param("map_frame_name", mapFrameName, mapFrameName);
    nh_.param("scan_frame_name", scanFrameName, scanFrameName);
    nh_.param("scan_topic_name", scanTopicName, scanTopicName);
    nh_.param("odom_topic_name", odomTopicName, odomTopicName);
    nh_.param("pc_topic_name", pcTopicName, pcTopicName);
    nh_.param("max_distance", maxDistance, maxDistance);
    nh_.param("max_angular_velocity", maxAngularV, maxAngularV);
    nh_.param("max_linear_velocity", maxLinearV, maxLinearV);
    nh_.param("load_pcd_path", loadPcdPath, loadPcdPath);
    nh_.param("save_pcd_path", savePcdPath, savePcdPath);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcTopicName, 10, true);
    pc_sub_ = nh_.subscribe(scanTopicName, 1, &PCMAP::scanCB, this);
    odom_sub_ = nh_.subscribe(odomTopicName, 1, &PCMAP::odomCB, this);
    save_server_ = nh_.advertiseService("save_pc_map", &PCMAP::save_map, this);
    loadPcd(loadPcdPath);
    ready = true;
    isMoving = false;
  }

  bool save_map(pcmap_updater::Save::Request &request,
                pcmap_updater::Save::Response &response);

  bool loadPcd(std::string filepath);

  void publish_map_pc();
};

#endif