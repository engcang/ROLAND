#ifndef ekf_land_H
#define ekf_land_H

#include "utility.h"

///// common headers for depth_to_pcl/octo and local_planner
#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow
#include <vector>
#include <chrono> 
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// headers for depto_to_pcl + YOLO boxes
#include <tf2_msgs/TFMessage.h> //for tf between frames
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <ekf_landing/bbox.h>
#include <ekf_landing/bboxes.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////////////////////////////


class ekf_land{
  public:
    sensor_msgs::Image depth;
    cv_bridge::CvImagePtr depth_ptr;
    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl, depth_cvt_pcl_empty;
    pcl::PointXYZ p3d, p3d_empty;

    bool pcl_check=false, tf_check=false, box_check=false, body_t_cam_check=false;
    std::string depth_topic, bboxes_topic, pcl_topic, pcl_base, body_base, agg_pcl_base; 

    double f_x=0.0, f_y=0.0, c_x=0.0, c_y=0.0, depth_max_range=0.0, hfov=0.0;
    double curr_roll=0.0, curr_pitch=0.0, curr_yaw=0.0;

    ///// octomap
    Matrix4f map_t_cam = Matrix4f::Identity();
    Matrix4f map_t_body = Matrix4f::Identity();
    Matrix4f body_t_cam = Matrix4f::Identity();

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Subscriber tf_sub;
    ros::Subscriber yolo_sub;
    ros::Publisher pcl_pub;

    // void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void bbox_callback(const ekf_landing::bboxes::ConstPtr& msg);

    ekf_land(ros::NodeHandle& n) : nh(n){
      ///// params
      nh.param("/depth_max_range", depth_max_range, 5.0);
      nh.param("/depth_fx", f_x, 554.254691191187);
      nh.param("/depth_fy", f_y, 554.254691191187);
      nh.param("/depth_cx", c_x, 320.5);
      nh.param("/depth_cy", c_y, 240.5);
      nh.param("/hfov", hfov, 1.5); // radian
      nh.param<std::string>("/depth_topic", depth_topic, "/camera/depth/image_raw");
      nh.param<std::string>("/bboxes_topic", bboxes_topic, "bboxes");

      nh.param<std::string>("/pcl_topic", pcl_topic, "converted_pcl");
      nh.param<std::string>("/pcl_base", pcl_base, "camera_link");
      nh.param<std::string>("/body_base", body_base, "base_link");
      nh.param<std::string>("/agg_pcl_base", agg_pcl_base, "map");

      ///// sub 
      depth_sub = nh.subscribe<sensor_msgs::Image>(depth_topic, 10, &ekf_land::depth_callback, this);
      tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &ekf_land::tf_callback, this);
      yolo_sub = nh.subscribe<ekf_landing::bboxes>(bboxes_topic, 10, &ekf_land::bbox_callback, this);

      ///// pub
      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic, 10);

      ROS_WARN("Class generated, started node...");
    }
};

void ekf_land::bbox_callback(const ekf_landing::bboxes::ConstPtr& msg){
}

void ekf_land::depth_callback(const sensor_msgs::Image::ConstPtr& msg){
  depth=*msg;
  try {
    // tic(); 
    depth_cvt_pcl.clear();
    // depth_cvt_pcl_empty.clear();
    if (depth.encoding=="32FC1"){
      depth_ptr = cv_bridge::toCvCopy(depth, "32FC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
      for (int i=0; i<depth_ptr->image.rows; i++)
      {
        for (int j=0; j<depth_ptr->image.cols; j++)
        {
          float temp_depth = depth_ptr->image.at<float>(i,j);
          if (std::isnan(temp_depth)){
            // p3d_empty.z = depth_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*hfov/2.0) * cos(abs(depth_ptr->image.rows/2.0 - i)/(depth_ptr->image.rows/2.0*0.488692191)); //float!!! double makes error here!!! because encoding is "32FC", float
            // p3d_empty.z = depth_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*hfov/2.0); //float!!! double makes error here!!! because encoding is "32FC", float
            // p3d_empty.x = ( j - c_x ) * p3d_empty.z / f_x;
            // p3d_empty.y = ( i - c_y ) * p3d_empty.z / f_y;
            // depth_cvt_pcl_empty.push_back(p3d_empty);
            continue;
          }
          else if (temp_depth >= 0.2 and temp_depth <=depth_max_range){
            p3d.z = temp_depth; //float!!! double makes error here!!! because encoding is "32FC", float
            p3d.x = ( j - c_x ) * p3d.z / f_x;
            p3d.y = ( i - c_y ) * p3d.z / f_y;
            depth_cvt_pcl.push_back(p3d);
          }
        }
      }
    }
    else if (depth.encoding=="16UC1") // uint16_t (stdint.h) or ushort or unsigned_short
    {
      depth_ptr = cv_bridge::toCvCopy(depth, "16UC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
      for (int i=0; i<depth_ptr->image.rows; i++)
      {
        for (int j=0; j<depth_ptr->image.cols; j++)
        {
          // if (115 <= i && i <= 185){ //prop guard
          //     if( j <= 125 || 515<= j){
          //         continue;
          //     }
          // }
          float temp_depth = depth_ptr->image.at<ushort>(i,j);
          if (std::isnan(temp_depth)){
            // p3d_empty.z = depth_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*0.75) * cos(abs(depth_ptr->image.rows/2.0 - i)/(depth_ptr->image.rows/2.0*0.488692191)); //float!!! double makes error here!!! because encoding is "32FC", float
            // p3d_empty.z = depth_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*hfov/2.0); //float!!! double makes error here!!! because encoding is "32FC", float
            // p3d_empty.x = ( j - c_x ) * p3d_empty.z / f_x;
            // p3d_empty.y = ( i - c_y ) * p3d_empty.z / f_y;
            // depth_cvt_pcl_empty.push_back(p3d_empty);
            continue;
          }
          else if (temp_depth/1000.0 >= 0.1 and temp_depth/1000.0 <=depth_max_range){
            p3d.z = (temp_depth/1000.0); //float!!! double makes error here!!! because encoding is "32FC", float
            p3d.x = ( j - c_x ) * p3d.z / f_x;
            p3d.y = ( i - c_y ) * p3d.z / f_y;
            depth_cvt_pcl.push_back(p3d);
          }
        }
      }
    }
    pcl_pub.publish(cloud2msg(depth_cvt_pcl, pcl_base));
    pcl_check=true;
    // toc();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error to cvt depth img");
    return;
  }
}

void ekf_land::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id==agg_pcl_base && msg->transforms[l].child_frame_id==body_base){
      ///// for tf between map and body
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      map_t_body(0,0) = m[0][0];
      map_t_body(0,1) = m[0][1];
      map_t_body(0,2) = m[0][2];
      map_t_body(1,0) = m[1][0];
      map_t_body(1,1) = m[1][1];
      map_t_body(1,2) = m[1][2];
      map_t_body(2,0) = m[2][0];
      map_t_body(2,1) = m[2][1];
      map_t_body(2,2) = m[2][2];

      map_t_body(0,3) = msg->transforms[l].transform.translation.x;
      map_t_body(1,3) = msg->transforms[l].transform.translation.y;
      map_t_body(2,3) = msg->transforms[l].transform.translation.z;
      map_t_body(3,3) = 1.0;

      ///// for tf between local-planner(map) and body -> roll=0, pitch=0 tf for simplification!!!
      m.getRPY(curr_roll, curr_pitch, curr_yaw);
    }
    if (msg->transforms[l].header.frame_id==body_base && !body_t_cam_check){
      tf::Quaternion q2(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m2(q2);
      body_t_cam(0,0) = m2[0][0];
      body_t_cam(0,1) = m2[0][1];
      body_t_cam(0,2) = m2[0][2];
      body_t_cam(1,0) = m2[1][0];
      body_t_cam(1,1) = m2[1][1];
      body_t_cam(1,2) = m2[1][2];
      body_t_cam(2,0) = m2[2][0];
      body_t_cam(2,1) = m2[2][1];
      body_t_cam(2,2) = m2[2][2];

      body_t_cam(0,3) = msg->transforms[l].transform.translation.x;
      body_t_cam(1,3) = msg->transforms[l].transform.translation.y;
      body_t_cam(2,3) = msg->transforms[l].transform.translation.z;
      body_t_cam(3,3) = 1.0;

      body_t_cam_check = true; // body_t_cam is fixed!!!
    }
  }
  map_t_cam = map_t_body * body_t_cam ;
  tf_check=true;
}

    // depth_cvt_pcl_map.clear();
    // depth_cvt_pcl_map_empty.clear();
    // pcl::transformPointCloud(depth_cvt_pcl, depth_cvt_pcl_map, map_t_cam); //essential for agg_pcl and octomap(based on world naturally!)
    // pcl::transformPointCloud(depth_cvt_pcl_empty, depth_cvt_pcl_map_empty, map_t_cam); //essential for agg_pcl and octomap(based on world naturally!)

    // octo_pcl_pub_em->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
    // octomap_pub.publish(cloud2msg(*octo_pcl_pub, agg_pcl_base));
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr fron_cells_sensor_end_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
    //   for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = depth_cvt_pcl_map_empty.begin(); it!=depth_cvt_pcl_map_empty.end(); ++it){
    //       fron_cells_sensor_end_pcl_pub->push_back(pcl::PointXYZ(fron_pt.x(), fron_pt.y(), fron_pt.z()));
    //   }
    //   frontier_sensor_end_pub.publish(cloud2msg(*fron_cells_sensor_end_pcl_pub, agg_pcl_base));

#endif