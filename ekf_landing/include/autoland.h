/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace Eigen;

class drone{
  public:
    nav_msgs::Odometry drone_odom;
    Vector3f drone_pose = MatrixXf::Zero(3,1);
    Vector3f drone_rpy = MatrixXf::Zero(3,1);
    geometry_msgs::TwistStamped vel_setpoint;

    ///// ros
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher local_vel_pub;
    ros::Timer estimated_timer;

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void run();


    drone(ros::NodeHandle& n) : nh(n){

      ///// sub 
      odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &drone::odom_callback, this);
      
      ///// pub
      local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

      ///// timer
      //estimated_timer = nh.createTimer(ros::Duration(1/20.0), &ekf_land::pub_Timer, this); // every 1/30 second.

      ROS_WARN("Class generated, started node...");
    }
};

void drone::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  drone_odom=*msg;
  drone_pose << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
  Quaternionf q;
  q.x() = drone_odom.pose.pose.orientation.x;
  q.y() = drone_odom.pose.pose.orientation.y;
  q.z() = drone_odom.pose.pose.orientation.z;
  q.w() = drone_odom.pose.pose.orientation.w;
  drone_rpy = q.toRotationMatrix().eulerAngles(0,1,2);
}

void drone::run(){

    local_vel_pub.publish(vel_setpoint);
}