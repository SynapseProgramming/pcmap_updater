#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcmap");
  PCMAP map;
  ros::Rate loopRate(2);
  while (ros::ok()) {
    map.publish_new();
    loopRate.sleep();
    ros::spinOnce();
  }
  // ros::spin();

  return 0;
}