#include <ros/ros.h>

#include <pcmap_updater/pcmap.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "pcmap");
  PCMAP map;


  ros::spin();
  return 0;
}
