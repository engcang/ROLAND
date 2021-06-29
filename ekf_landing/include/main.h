#ifndef ekf_land_H
#define ekf_land_H

#include "utility.h"

///// common headers for depth_to_pcl/octo and local_planner
#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow
#include <time.h>  // time
#include <vector>
#include <chrono>
#include <algorithm> // min max
#include <random> //random
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// headers for depto_to_pcl + YOLO boxes
#include <tf2_msgs/TFMessage.h> //for tf between frames
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <ekf_landing/bbox.h>
#include <ekf_landing/bboxes.h>
#include <gtec_msgs/Ranging.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

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
    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl;
    
    ekf_landing::bboxes boxes;
    pcl::PointXYZ p3d;
    geometry_msgs::PoseWithCovarianceStamped mobile_pose_ekf;
    nav_msgs::Odometry mobile_pose;
    gtec_msgs::Ranging uwb_measured;

    bool depth_check=false, tf_check=false, body_t_cam_check=false;
    bool uwb_kalman=false, yolo_kalman=false;
    std::string depth_topic, bboxes_topic, pcl_topic, pcl_base, body_base, agg_pcl_base; 

    double f_x=0.0, f_y=0.0, c_x=0.0, c_y=0.0, depth_max_range=0.0, hfov=0.0;
    double curr_roll=0.0, curr_pitch=0.0, curr_yaw=0.0;
    double scale_factor=1.0;
    

    ///// tf
    Matrix4f map_t_cam = Matrix4f::Identity();
    Matrix4f map_t_body = Matrix4f::Identity();
    Matrix4f map_t_body_rot = Matrix4f::Identity();
    Matrix4f body_t_cam = Matrix4f::Identity();

    ///// Kalman
    bool init=false, corrected=false;
    MatrixXf P_ = MatrixXf::Zero(6,6);
    MatrixXf P = MatrixXf::Zero(6,6);
    MatrixXf Q = MatrixXf::Zero(6,6);
    MatrixXf X_ = MatrixXf::Zero(6, 1);
    MatrixXf Xhat = MatrixXf::Zero(6, 1);
    MatrixXf delta_x = MatrixXf::Zero(6, 1);

    MatrixXf Hu = MatrixXf::Zero(1, 6); //uwb
    MatrixXf Ru = MatrixXf::Zero(1, 1);
    MatrixXf Zu = MatrixXf::Zero(1, 1);
    MatrixXf Ku = MatrixXf::Zero(6, 1);

    MatrixXf Hc = MatrixXf::Zero(3, 6); //camera
    MatrixXf Rc = MatrixXf::Zero(3, 3);
    MatrixXf Zc = MatrixXf::Zero(3, 1);
    MatrixXf Kc = MatrixXf::Zero(6, 3);

    ///// ros
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Subscriber tf_sub;
    ros::Subscriber yolo_sub;
    ros::Subscriber uwb_sub;
    ros::Subscriber mobile_ekf_sub;
    ros::Subscriber mobile_sub;
    ros::Publisher pcl_pub;
    ros::Publisher center_pub;
    ros::Publisher estimated_pub;
    ros::Publisher estimated_pose_diff_pub;
    ros::Timer estimated_timer;

    // void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void bbox_callback(const ekf_landing::bboxes::ConstPtr& msg);
    void uwb_callback(const gtec_msgs::Ranging::ConstPtr& msg);
    void mobile_robot_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    //void mobile_robot_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pub_Timer(const ros::TimerEvent& event);

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

      nh.param<bool>("/yolo_kalman", yolo_kalman, true);
      nh.param<bool>("/uwb_kalman", uwb_kalman, true);

      ///// sub 
      depth_sub = nh.subscribe<sensor_msgs::Image>(depth_topic, 10, &ekf_land::depth_callback, this);
      tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &ekf_land::tf_callback, this);
      yolo_sub = nh.subscribe<ekf_landing::bboxes>(bboxes_topic, 10, &ekf_land::bbox_callback, this);
      uwb_sub = nh.subscribe<gtec_msgs::Ranging>("/gtec/toa/ranging", 10, &ekf_land::uwb_callback, this);
      mobile_ekf_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 10, &ekf_land::mobile_robot_ekf_callback, this);
      // mobile_sub = nh.subscribe<nav_msgs::Odometry>("/jackal1/jackal_velocity_controller/odom", 10, &ekf_land::mobile_robot_callback, this);

      ///// pub
      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic, 10);
      center_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic+"/center", 10);
      estimated_pub = nh.advertise<sensor_msgs::PointCloud2>("/estimated_pose", 10);
      estimated_pose_diff_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose_diff", 10);
      // estimated_mobile_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimated_mobile_vel", 10);
      

      ///// timer
      estimated_timer = nh.createTimer(ros::Duration(1/20.0), &ekf_land::pub_Timer, this); // every 1/30 second.

      // last_time = ros::Time::now();
      ROS_WARN("Class generated, started node...");
    }
};

void ekf_land::bbox_callback(const ekf_landing::bboxes::ConstPtr& msg){
  boxes=*msg;
  depth_cvt_pcl.clear();

  if(depth_check && tf_check && init){ // TODO: check if ground points are within box
    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl_center;
    cv::Mat depth_img = depth_ptr->image;

    int max_score_idx=0;
    for (int l=0; l < boxes.bboxes.size(); l++){
      if (l!=0){ max_score_idx = boxes.bboxes[max_score_idx].score < boxes.bboxes[l].score ? l : max_score_idx; }
      pcl::PointCloud<pcl::PointXYZ> ground_pcl;
      int ground_count = 0;
      int counter = 0;
      int ground_num = 10;
      int xlower = std::max(0, (int)boxes.bboxes[l].x - (int)(boxes.bboxes[l].width*0.5));
      int xupper = std::min(depth_img.size().width-1 , (int)boxes.bboxes[l].x + (int)(boxes.bboxes[l].width*1.5));
      int ylower = std::max(0, (int)boxes.bboxes[l].y - (int)(boxes.bboxes[l].width*0.5));
      int yupper = std::min(depth_img.size().height-1 , (int)boxes.bboxes[l].y + (int)(boxes.bboxes[l].height*1.5));
      
      std::random_device rd; 
      std::mt19937 mersenne(rd());
      std::uniform_int_distribution<> xdist(xlower, xupper);
      std::uniform_int_distribution<> ydist(ylower, yupper);
      while (ground_count <= ground_num){
        if (counter>1000){
          return; }
        int j = xdist(mersenne);
        int i = ydist(mersenne);
        if (boxes.bboxes[l].x<j && j<boxes.bboxes[l].x + boxes.bboxes[l].width && boxes.bboxes[l].y<i && i<boxes.bboxes[l].y + boxes.bboxes[l].height){
          counter++;
          continue;
        }
        float temp_depth = depth_img.at<float>(i,j);
        if (std::isnan(temp_depth)){
          counter++;
          continue;
        }
        else if (temp_depth/scale_factor >= 0.2 and temp_depth/scale_factor <=depth_max_range){ // scale factor = 1 for 32FC, 1000 for 16UC
          p3d.z = (temp_depth/scale_factor); 
          p3d.x = ( j - c_x ) * p3d.z / f_x;
          p3d.y = ( i - c_y ) * p3d.z / f_y;

          ground_pcl.push_back(p3d);
          ground_count++;
          counter++;
        }
      }
      if (ground_count<3)
        return;

      MatrixXd A = MatrixXd::Zero(ground_num,3);
      VectorXd b(ground_num);
      VectorXd x_(3);
      for (int i = 0; i < ground_num; i++)
      {
        A(i,0) = ground_pcl.at(i).x;
        A(i,1) = ground_pcl.at(i).y;
        A(i,2) = ground_pcl.at(i).z;
        b(i) = 1;
      }
      x_ = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
      
      
      int count=0; float temp_depth=0.0;
      pcl::PointXYZ p3d_center;
      p3d_center.x = 0; p3d_center.y = 0; p3d_center.z = 0;
      for (int j=boxes.bboxes[l].x; j < boxes.bboxes[l].x + boxes.bboxes[l].width; ++j){
        for (int i=boxes.bboxes[l].y; i < boxes.bboxes[l].y + boxes.bboxes[l].height; ++i){
          if(scale_factor==1.0)
            temp_depth = depth_img.at<float>(i,j); //float!!! double makes error here!!! because encoding is "32FC", float
          else if(scale_factor==1000.0)
            temp_depth = depth_img.at<ushort>(i,j);
          if (std::isnan(temp_depth))
            continue;
          else if (temp_depth/scale_factor >= 0.2 and temp_depth/scale_factor <=depth_max_range){ // scale factor = 1 for 32FC, 1000 for 16UC
            p3d.z = (temp_depth/scale_factor); 
            p3d.x = ( j - c_x ) * p3d.z / f_x;
            p3d.y = ( i - c_y ) * p3d.z / f_y;

            float distance = abs(x_(0)*p3d.x + x_(1)*p3d.y + x_(2)*p3d.z - 1)/sqrtf(x_(0)*x_(0) + x_(1)*x_(1) + x_(2)*x_(2));
            if(distance < 0.1f){
              continue;
            }

            depth_cvt_pcl.push_back(p3d);
            p3d_center.x = p3d_center.x + p3d.x;
            p3d_center.y = p3d_center.y + p3d.y;
            p3d_center.z = p3d_center.z + p3d.z;
            count++;
          }
        }
      }
      if (count > 0){
        p3d_center.x /= count; p3d_center.y /= count; p3d_center.z /= count;
        depth_cvt_pcl_center.push_back(p3d_center);
      }
    }
    pcl_pub.publish(cloud2msg(depth_cvt_pcl, pcl_base));

    if (!depth_cvt_pcl_center.empty()){
      center_pub.publish(cloud2msg(depth_cvt_pcl_center, pcl_base));
      if (yolo_kalman){
        MatrixXf temp(4,1); MatrixXf temp2(4,1);
        temp << depth_cvt_pcl_center[max_score_idx].x, depth_cvt_pcl_center[max_score_idx].y, depth_cvt_pcl_center[max_score_idx].z, 1.0;
        double cov = 1.0;
        temp2 << map_t_body_rot * body_t_cam * temp;

        Zc << temp2(0), temp2(1), temp2(2);
        Rc = MatrixXf::Identity(3,3) * cov / boxes.bboxes[max_score_idx].score;
        Kc = P_ * Hc.transpose() * (Hc * P_ * Hc.transpose() + Rc).inverse();
        Xhat = X_ + Kc * (Zc - Hc * X_);
        P = P_ - (Kc * Hc * P_);

        corrected = true;
      }
    }
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

      map_t_body_rot = map_t_body;
      map_t_body_rot(0,3) = 0;
      map_t_body_rot(1,3) = 0;
      map_t_body_rot(2,3) = 0;
      map_t_body_rot(3,3) = 1.0;

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

void ekf_land::uwb_callback(const gtec_msgs::Ranging::ConstPtr& msg){
  if(init){
    if (uwb_kalman && msg->anchorId==1 && msg->tagId==0){
      uwb_measured=*msg;
      if (uwb_measured.range/1000.0 > 0.8 && uwb_measured.range/1000.0 < 10.0){
        Zu << uwb_measured.range/1000.0;
        Ru << (-uwb_measured.rss)*uwb_measured.errorEstimation*100.0;
          double rt = sqrt(pow(X_(3) - X_(0), 2) + pow(X_(4) - X_(1), 2) + pow(X_(5) - X_(2), 2));
        Hu << (X_(0)-X_(3))/rt, (X_(1)-X_(4))/rt, (X_(2)-X_(5))/rt, (X_(3)-X_(0))/rt, (X_(4)-X_(1))/rt, (X_(5)-X_(2))/rt;
        Ku = P_ * Hu.transpose() * (Hu * P_ * Hu.transpose() + Ru).inverse();
        Xhat = X_ + Ku * (Zu - Hu * X_);
        P = P_ - (Ku * Hu * P_);

        corrected = true;
      }
    }
  }
}

// void ekf_land::mobile_robot_callback(const nav_msgs::Odometry::ConstPtr& msg){
//   mobile_pose=*msg;

//   if(tf_check){
//     if (!init){
//       P_ << 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//             0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
//             0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
//             0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
//             0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
//             0.0, 0.0, 0.0, 0.0, 0.0, 1000.0;
//       P = P_;
//       Q << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
//            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
//            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
//            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
//            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
//            0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
//       Hc << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
//             0.0, -1.0, 0.0, 0.0, 1.0, 0.0,
//             0.0, 0.0, -1.0, 0.0, 0.0, 1.0;
//       delta_x << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose.pose.pose.position.x, mobile_pose.pose.pose.position.y, mobile_pose.pose.pose.position.z;
//       X_ << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose.pose.pose.position.x, mobile_pose.pose.pose.position.y, mobile_pose.pose.pose.position.z;
//       init=true;
//     }
//     else{
//       VectorXf current(6);
//       current << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose.pose.pose.position.x, mobile_pose.pose.pose.position.y, mobile_pose.pose.pose.position.z;
//       if (corrected){
//         X_ = Xhat + ( current - delta_x ); 
//         P_ = P + Q;
//       }
//       else{
//         X_ = X_ + ( current - delta_x );
//         P_ = P_ + Q;
//       }
//       delta_x = current;
//       corrected=false;
//     }
//   }
// }

void ekf_land::mobile_robot_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  mobile_pose_ekf=*msg;
  if(tf_check){
    if (!init){
      P_ << 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1000.0;
      P = P_;
      Q << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.05;
      Hc << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, -1.0, 0.0, 0.0, 1.0;
      delta_x << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose_ekf.pose.pose.position.x, mobile_pose_ekf.pose.pose.position.y, mobile_pose_ekf.pose.pose.position.z;
      X_ << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose_ekf.pose.pose.position.x, mobile_pose_ekf.pose.pose.position.y, mobile_pose_ekf.pose.pose.position.z;
      init=true;
    }
    else{
      VectorXf current(6);
      current << map_t_body(0,3), map_t_body(1,3), map_t_body(2,3), mobile_pose_ekf.pose.pose.position.x, mobile_pose_ekf.pose.pose.position.y, mobile_pose_ekf.pose.pose.position.z;
      if (corrected){
        X_ = Xhat + ( current - delta_x ); 
        P_ = P + Q;
      }
      else{
        X_ = X_ + ( current - delta_x );
        P_ = P_ + Q;
      }
      delta_x = current;
      corrected=false;
    }
  }
}

void ekf_land::pub_Timer(const ros::TimerEvent& event){
  if (init){
    pcl::PointCloud<pcl::PointXYZ> estimated_pcl;
    pcl::PointXYZ p3d_estimated, p3d_estimated2;
    geometry_msgs::PoseStamped estimated_pose_diff;
    // geometry_msgs::TwistStamped estimated_mobile_vel;
    if (corrected){
      p3d_estimated.x = Xhat(0);  p3d_estimated.y = Xhat(1);  p3d_estimated.z = Xhat(2);
      estimated_pcl.push_back(p3d_estimated);
      p3d_estimated2.x = Xhat(3);  p3d_estimated2.y = Xhat(4);  p3d_estimated2.z = Xhat(5);
      estimated_pcl.push_back(p3d_estimated2);
      estimated_pub.publish(cloud2msg(estimated_pcl, agg_pcl_base));
      estimated_pose_diff.pose.position.x = Xhat(3) - Xhat(0);
      estimated_pose_diff.pose.position.y = Xhat(4) - Xhat(1);
      estimated_pose_diff.pose.position.z = Xhat(5) - Xhat(2);
      estimated_pose_diff.header.frame_id = agg_pcl_base;
      estimated_pose_diff.header.stamp = ros::Time::now();
      estimated_pose_diff_pub.publish(estimated_pose_diff);
    }
    else{
      p3d_estimated.x = X_(0);  p3d_estimated.y = X_(1);  p3d_estimated.z = X_(2);
      estimated_pcl.push_back(p3d_estimated);
      p3d_estimated2.x = X_(3);  p3d_estimated2.y = X_(4);  p3d_estimated2.z = X_(5);
      estimated_pcl.push_back(p3d_estimated2);
      estimated_pub.publish(cloud2msg(estimated_pcl, agg_pcl_base));
      estimated_pose_diff.pose.position.x = X_(3) - X_(0);
      estimated_pose_diff.pose.position.y = X_(4) - X_(1);
      estimated_pose_diff.pose.position.z = X_(5) - X_(2);
      estimated_pose_diff.header.frame_id = agg_pcl_base;
      estimated_pose_diff.header.stamp = ros::Time::now();
      estimated_pose_diff_pub.publish(estimated_pose_diff);
    }
    ROS_WARN("%.1f %.1f %.1f %.1f %.1f %.1f", X_(0), X_(1), X_(2), X_(3), X_(4), X_(5));
  }
}
#endif