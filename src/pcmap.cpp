#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

void PCMAP::scanCB(const sensor_msgs::PointCloud2ConstPtr& inp) {
  sensor_msgs::PointCloud2 ros_pcl;
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  try {
    geometry_msgs::TransformStamped laserTransform =
        tfBuffer.lookupTransform("map", "rslidar", ros::Time(0));
    geometry_msgs::TransformStamped laserPose =
        tfBuffer.lookupTransform("rslidar", "map", ros::Time(0));
    tf2::doTransform(*inp, ros_pcl, laserTransform);
    pcl::fromROSMsg(ros_pcl, pcl_pc);
    std::vector<pcl::PointXYZ> data;
    for (const auto& point : pcl_pc.points) {
      data.push_back(point);
    }
    // std::cout << pcl_pc.points[0].x << "\n";
    double x = laserTransform.transform.translation.x;
    double y = laserTransform.transform.translation.y;
    double z = laserTransform.transform.translation.z;
    pcl::PointXYZ laser_origin(x, y, z);
    // std::cout << x << " " << y << " " << z << "\n";
    probMap.insertPointCloud(data, laser_origin, 100.0);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
}

bool PCMAP::loadPcd(std::string filepath) {
  bool status = ReadPointsFromPCD(filepath, map_points);

  if (status) {
    // insert points into prob map
    for (Vector3D iter : map_points) {
      probMap.addInitPoint(iter);
    }
    return true;
  } else
    return false;
}
