cmake_minimum_required(VERSION 3.0.2)
project(pcmap_updater)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(PCL REQUIRED COMPONENTS common io)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(pcmap src/pcmap.cpp src/pcl_utils.cpp src/probabilistic_map.cpp)
target_link_libraries(pcmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
  TARGETS
  pcmap
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
