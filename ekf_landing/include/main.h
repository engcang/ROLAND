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
    ekf_landing::bboxes boxes;
    cv_bridge::CvImagePtr depth_ptr;
    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl;
    pcl::PointXYZ p3d;

    bool depth_check=false, depth2_check=false, tf_check=false, box_check=false, body_t_cam_check=false;
    std::string depth_topic, bboxes_topic, pcl_topic, pcl_base, body_base, agg_pcl_base; 

    double f_x=0.0, f_y=0.0, c_x=0.0, c_y=0.0, depth_max_range=0.0, hfov=0.0;
    double curr_roll=0.0, curr_pitch=0.0, curr_yaw=0.0;
    double scale_factor=1.0;

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
    ros::Publisher center_pub;

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
      nh.param<std::string>("/bboxes_topic", bboxes_topic, "/bboxes");

      nh.param<std::string>("/pcl_topic", pcl_topic, "/converted_pcl");
      nh.param<std::string>("/pcl_base", pcl_base, "/camera_link");
      nh.param<std::string>("/body_base", body_base, "/base_link");
      nh.param<std::string>("/agg_pcl_base", agg_pcl_base, "map");

      ///// sub 
      depth_sub = nh.subscribe<sensor_msgs::Image>(depth_topic, 10, &ekf_land::depth_callback, this);
      tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &ekf_land::tf_callback, this);
      yolo_sub = nh.subscribe<ekf_landing::bboxes>(bboxes_topic, 10, &ekf_land::bbox_callback, this);

      ///// pub
      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic, 10);
      center_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic+"/center", 10);

      ROS_WARN("Class generated, started node...");
    }
};

void ekf_land::bbox_callback(const ekf_landing::bboxes::ConstPtr& msg){
  // MatrixXd A = MatrixXd::Zero(row,3);
  // A << x1, y1, z1,
  //   x1, y1, z1,
  //   x1, y1, z1;

  //   VectorXd b(row);
  //   VectorXd x_(3);
  //   b << 1, 1, 1, ... 1;
  //   x_ = A.lu().solve(b);
  // x(1) = a, x(2) = b, x(3) = c;

  boxes=*msg;
  depth_cvt_pcl.clear();

  if(depth_check){
    int count=0;
    pcl::PointXYZ p3d_center;
    p3d_center.x = 0; p3d_center.y = 0; p3d_center.z = 0;
    cv::Mat depth_img = depth_ptr->image;
    for (int l=0; l < boxes.bboxes.size(); l++){
      for (int j=boxes.bboxes[l].x; j < boxes.bboxes[l].x + boxes.bboxes[l].width; ++j){
        for (int i=boxes.bboxes[l].y; i < boxes.bboxes[l].y + boxes.bboxes[l].height; ++i){
          float temp_depth = depth_img.at<float>(i,j); //float!!! double makes error here!!! because encoding is "32FC", float
          if (std::isnan(temp_depth)){
            continue;}
          else if (temp_depth/scale_factor >= 0.2 and temp_depth/scale_factor <=depth_max_range){ // scale factor = 1 for 32FC, 1000 for 16UC
            p3d.z = (temp_depth/scale_factor); 
            p3d.x = ( j - c_x ) * p3d.z / f_x;
            p3d.y = ( i - c_y ) * p3d.z / f_y;
            depth_cvt_pcl.push_back(p3d);
            p3d_center.x = p3d_center.x + p3d.x;
            p3d_center.y = p3d_center.y + p3d.y;
            p3d_center.z = p3d_center.z + p3d.z;
            count++;
          }
        }
      }
    }



    box_check=true;
    pcl_pub.publish(cloud2msg(depth_cvt_pcl, pcl_base));

    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl_center;
    p3d_center.x /= count; p3d_center.y /= count; p3d_center.z /= count;
    depth_cvt_pcl_center.push_back(p3d_center);
    center_pub.publish(cloud2msg(depth_cvt_pcl_center, pcl_base));
  }
}

void ekf_land::depth_callback(const sensor_msgs::Image::ConstPtr& msg){
  depth=*msg;
  try {
    depth_cvt_pcl.clear();
    if (depth.encoding=="32FC1"){
      depth_ptr = cv_bridge::toCvCopy(depth, "32FC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
      scale_factor=1.0;
    }
    else if (depth.encoding=="16UC1"){ // uint16_t (stdint.h) or ushort or unsigned_short
      depth_ptr = cv_bridge::toCvCopy(depth, "16UC1"); // == sensor_msgs::image_encodings::TYPE_16UC1
      scale_factor=1000.0;
    }
    depth_check=true;
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

#endif