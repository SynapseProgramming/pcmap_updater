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
    std::cout << pcl_pc.points[0].x << "\n";


  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
}
