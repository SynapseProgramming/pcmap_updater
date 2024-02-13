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
  PCMAP();

  bool save_map(pcmap_updater::Save::Request &request,
                pcmap_updater::Save::Response &response);

  bool loadPcd(std::string filepath);

  void publish_map_pc();
};

#endif