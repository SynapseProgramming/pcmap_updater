#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

void PCMAP::scanCB(const sensor_msgs::PointCloud2ConstPtr& inp) {
  sensor_msgs::PointCloud2 ros_pcl;
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  try {
    geometry_msgs::TransformStamped laserTransform =
        tfBuffer.lookupTransform("map", "rslidar", ros::Time(0));
    tf2::doTransform(*inp, ros_pcl, laserTransform);
    pcl::fromROSMsg(ros_pcl, pcl_pc);
    std::vector<pcl::PointXYZ> data;
    for (const auto& point : pcl_pc.points) {
      data.push_back(point);
    }
    double x = laserTransform.transform.translation.x;
    double y = laserTransform.transform.translation.y;
    double z = laserTransform.transform.translation.z;
    pcl::PointXYZ laser_origin(x, y, z);
    probMap.insertPointCloud(data, laser_origin, 100.0);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
}

bool PCMAP::loadPcd(std::string filepath) {
  bool status = ReadPointsFromPCD(filepath, map_points);

  if (status) {
    for (Vector3D iter : map_points) {
      probMap.addInitPoint(iter);
    }
    return true;
  } else
    return false;
}

bool PCMAP::save_map(pcmap_updater::Save::Request& request,
                     pcmap_updater::Save::Response& response) {
  std::vector<Point3D> converted;
  probMap.getOccupiedVoxels(converted);

  WritePointsFromPCD("/home/ro/Documents/test_save/test.pcd", converted);

  response.Status = true;

  return true;
}

void PCMAP::publish_map_pc() {
  if (ready == false) return;

  // load points back into a point cloud
  sensor_msgs::PointCloud2 output_msg;
  pcl::PointCloud<pcl::PointXYZ> pc;

  std::vector<Point3D> points;
  probMap.getOccupiedVoxels(points);

  for (Point3D pts : points) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    pc.push_back(pclpt);
  }
  pcl::toROSMsg(pc, output_msg);
  output_msg.header.frame_id = "map";
  pc_pub_.publish(output_msg);
}
