#include <pcmap_updater/pcmap.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcmap");
  PCMAP map;
    map.publish_new();
    ros::spin();

  return 0;
}
