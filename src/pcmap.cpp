#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

#include <iostream>

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

bool PCMAP::save_map(pcmap_updater::Save::Request& request,
                     pcmap_updater::Save::Response& response) {
  std::vector<Bonxai::Point3D> converted;
  probMap.getOccupiedVoxels(converted);
  Bonxai::WritePointsFromPCD(savePcdPath, converted);

  response.Status = true;

  return true;
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
