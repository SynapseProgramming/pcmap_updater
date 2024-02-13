#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

#include <iostream>

PCMAP::PCMAP()
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

void PCMAP::scanCB(const sensor_msgs::PointCloud2ConstPtr& inp) {
  if (!isMoving) return;

  sensor_msgs::PointCloud2 ros_pcl;
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  pcl::PointCloud<pcl::PointXYZ> filtered_pcl_pc;
  try {
    geometry_msgs::TransformStamped laserTransform =
        tfBuffer.lookupTransform(mapFrameName, scanFrameName, ros::Time(0));
    tf2::doTransform(*inp, ros_pcl, laserTransform);
    pcl::fromROSMsg(ros_pcl, pcl_pc);
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(pcl_pc, filtered_pcl_pc, indices);
    std::vector<pcl::PointXYZ> data;
    for (const auto& point : filtered_pcl_pc.points) {
      data.push_back(point);
    }
    double x = laserTransform.transform.translation.x;
    double y = laserTransform.transform.translation.y;
    double z = laserTransform.transform.translation.z;
    pcl::PointXYZ laser_origin(x, y, z);
    std::cout << "x: " << x << " y: " << y << " z: " << z << "\n";
    probMap.insertPointCloud(data, laser_origin, maxDistance);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
}

void PCMAP::odomCB(const nav_msgs::OdometryConstPtr& inp) {
  double linearv = inp->twist.twist.linear.x;
  double angularv = inp->twist.twist.angular.z;
  if (std::abs(angularv) <= maxAngularV && std::abs(linearv) <= maxLinearV) {
    isMoving = false;
  } else {
    isMoving = true;
  }
}

bool PCMAP::save_map(pcmap_updater::Save::Request& request,
                     pcmap_updater::Save::Response& response) {
  std::vector<Bonxai::Point3D> converted;
  probMap.getOccupiedVoxels(converted);
  Bonxai::WritePointsFromPCD(savePcdPath, converted);

  response.Status = true;

  return true;
}

bool PCMAP::loadPcd(std::string filepath) {
  bool status = Bonxai::ReadPointsFromPCD(filepath, map_points);

  if (status) {
    for (Eigen::Vector3d iter : map_points) {
      probMap.addInitPoint(iter);
    }
    return true;
  } else
    return false;
}

void PCMAP::publish_map_pc() {
  if (ready == false) return;

  // load points back into a point cloud
  sensor_msgs::PointCloud2 output_msg;
  pcl::PointCloud<pcl::PointXYZ> pc;

  std::vector<Bonxai::Point3D> points;
  probMap.getOccupiedVoxels(points);
  std::cout << "points size: " << points.size() << "\n";
  std::cout << "is moving: " << isMoving << "\n";

  for (Bonxai::Point3D pts : points) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    pc.push_back(pclpt);
  }
  pcl::toROSMsg(pc, output_msg);
  output_msg.header.frame_id = mapFrameName;
  pc_pub_.publish(output_msg);
}
