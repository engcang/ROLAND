#ifndef UTILITY_H
#define UTILITY_H

#include <chrono> 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using namespace std::chrono; 
using namespace std;

/////////// utils
//////////// common
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

high_resolution_clock::time_point start; //global, to use tic()
void tic(){
   start = high_resolution_clock::now();
}
void toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%.3f ms spent", duration.count()/1000.0);
}
void toc(string text){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%s %.3f ms spent", text.c_str(), duration.count()/1000.0);
}


//////////// transforms
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}

#endif