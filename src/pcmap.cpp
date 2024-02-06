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
    // publish out the laser origin
    // std::cout << "x: " << x << " y: " << y << " z: " << z << "\n";
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
    std::cout << "Size of map_points: " << map_points.size() << "\n";
    for (Vector3D iter : map_points) {
      probMap.addInitPoint(iter);
    }
    return true;
  } else
    return false;
}

bool PCMAP::save_map(pcmap_updater::Save::Request& request,
                     pcmap_updater::Save::Response& response) {
  std::cout << "I heard " << request.Save << "\n";
  std::vector<Point3D> converted;
  probMap.getOccupiedVoxels(converted);

  WritePointsFromPCD("/home/ro/Documents/test_save/test.pcd", converted);

  response.Status = true;

  return true;
}

void PCMAP::publish_new() {
  if (ready == false) return;

  // load points back into a point cloud
  sensor_msgs::PointCloud2 output_msg;

  std::vector<Point3D> points;
  probMap.getOccupiedVoxels(points);
  std::cout << "size: " << points.size() << "\n";

  for (Point3D pts : points) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    pc.push_back(pclpt);
  }
  pcl::toROSMsg(pc, output_msg);
  output_msg.header.frame_id = "map";
  pc_pub_.publish(output_msg);
}

void PCMAP::publish_raytraced() {
  if (ready == false) return;
  sensor_msgs::PointCloud2 output_msg;
  pcl::PointCloud<pcl::PointXYZ> og_pcl;
  std::vector<Point3D> points;
  probMap.getRaytracedVoxels(points);
  for (Point3D pts : points) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    og_pcl.push_back(pclpt);
  }
  pcl::toROSMsg(og_pcl, output_msg);
  output_msg.header.frame_id = "map";
  raytrace_pub_.publish(output_msg);
}

void PCMAP::intersect() {
  std::unordered_set<CoordT> cnt;
  std::vector<Point3D> occupied_points;
  std::vector<Point3D> raytraced_points;
  std::vector<Point3D> intersected;
  probMap.getOccupiedVoxels(occupied_points);
  probMap.getRaytracedVoxels(raytraced_points);

  for (auto it : raytraced_points) {
    cnt.insert(probMap._grid.posToCoord(it));
  }
  for (auto it : occupied_points) {
    if (cnt.count(probMap._grid.posToCoord(it))) {
      intersected.push_back(it);
    }
  }

  if (ready == false) return;
  sensor_msgs::PointCloud2 output_msg;
  pcl::PointCloud<pcl::PointXYZ> og_pcl;

  sensor_msgs::PointCloud2 global_output_msg;
  pcl::PointCloud<pcl::PointXYZ> global_og_pcl;

  for (Point3D pts : intersected) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    og_pcl.push_back(pclpt);
  }
  pcl::toROSMsg(og_pcl, output_msg);
  output_msg.header.frame_id = "map";
  inter_pub_.publish(output_msg);

  // std::cout << "cnt size: " << cnt.size() << "\n";
  // std::cout << "intersected size: " << intersected.size() << "\n";
  for (int i = 0; i < intersected.size(); i++) {
    Point3D curr = intersected[i];
    std::cout << i << " prob value: " << probMap.getVoxelProbability(curr)
              << "\n";
  }

  for (Point3D pts : occupied_points) {
    pcl::PointXYZ pclpt(pts.x, pts.y, pts.z);
    global_og_pcl.push_back(pclpt);
  }
  pcl::toROSMsg(global_og_pcl, global_output_msg);
  global_output_msg.header.frame_id = "map";
  pc_pub_.publish(global_output_msg);

  probMap.raytracedVoxels.clear();
}