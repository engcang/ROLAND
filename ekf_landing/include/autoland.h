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

float saturate(float value, float limit){
  if (value > limit){
    return limit;
  }
  if (value < -limit){
    return -limit;
  }
  return value;
  
}

using namespace Eigen;




class drone{
  public:
    mavros_msgs::State current_state;
    nav_msgs::Odometry drone_odom;
    geometry_msgs::PoseStamped pose_diff;
    Vector3f drone_pose = MatrixXf::Zero(3,1);
    Vector3f drone_heading = MatrixXf::Zero(3,1);
    Vector3f land_pose = MatrixXf::Zero(3,1);
    Vector3f land_vel = MatrixXf::Zero(3,1);
    geometry_msgs::TwistStamped vel_setpoint;

    ///// ros
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber pose_diff_sub;
    ros::Publisher local_vel_pub;
    ros::Publisher land_pose_pub;
    ros::Timer estimated_timer;

    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pose_diff_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void run();


    drone(ros::NodeHandle& n) : nh(n){

      ///// sub 
      state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &drone::state_callback, this);
      odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &drone::odom_callback, this);
      pose_diff_sub = nh.subscribe<geometry_msgs::PoseStamped>("/estimated_pose_diff", 10, &drone::pose_diff_callback, this);      
      ///// pub
      local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
      land_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/land_pose", 10);
      ///// timer
      //estimated_timer = nh.createTimer(ros::Duration(1/20.0), &ekf_land::pub_Timer, this); // every 1/30 second.

      ROS_WARN("Class generated, started node...");
    }
};

void drone::state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void drone::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  drone_odom=*msg;
  drone_pose << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
  Quaternionf q;
  Vector3f temp;
  temp << 1, 0, 0;
  q.x() = drone_odom.pose.pose.orientation.x;
  q.y() = drone_odom.pose.pose.orientation.y;
  q.z() = drone_odom.pose.pose.orientation.z;
  q.w() = drone_odom.pose.pose.orientation.w;
  drone_heading = q.toRotationMatrix()*temp;
}

void drone::pose_diff_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_diff=*msg;
  land_pose << pose_diff.pose.position.x + drone_pose(0), 
  pose_diff.pose.position.y + drone_pose(1),
  pose_diff.pose.position.z + drone_pose(2);

  geometry_msgs::PoseStamped land_pose_msg;
  land_pose_msg.pose.position.x = land_pose(0);
  land_pose_msg.pose.position.y = land_pose(1);
  land_pose_msg.pose.position.z = land_pose(2)+0.8;
  land_pose_msg.header.frame_id = "map";
  land_pose_msg.header.stamp = ros::Time::now();
  land_pose_pub.publish(land_pose_msg);
}
 
void drone::run(){
  Vector3f horizontal_err;
  Vector3f heading_cur;
  float vertical_err = land_pose(2)-drone_pose(2);
  heading_cur(0) = drone_heading(0);
  heading_cur(1) = drone_heading(1);
  heading_cur(2) = 0;
  horizontal_err(0) = land_pose(0)-drone_pose(0)+0.3*heading_cur(0)*vertical_err;
  horizontal_err(1) = land_pose(1)-drone_pose(1)+0.3*heading_cur(1)*vertical_err;
  horizontal_err(2) = 0;
  
  
  if (horizontal_err.norm() < 0.2)
  {
    vel_setpoint.twist.linear.x = saturate(0.5*horizontal_err(0),1);
    vel_setpoint.twist.linear.y = saturate(0.5*horizontal_err(1),1);
    vel_setpoint.twist.linear.z = -0.15f;
    vel_setpoint.twist.angular.z = 0;
    if (drone_odom.twist.twist.linear.z<0.05 && drone_odom.twist.twist.linear.z>-0.05 && vertical_err>-0.8)
    {
      exit(0);
    }
    
  }
  else
  {
    vel_setpoint.twist.linear.x = saturate(0.4*horizontal_err(0),1);
    vel_setpoint.twist.linear.y = saturate(0.4*horizontal_err(1),1);
    vel_setpoint.twist.linear.z = saturate(0.3*(vertical_err+2),0.3);
    if (horizontal_err.norm() > 0.4){
      vel_setpoint.twist.angular.z = 0.8*saturate(heading_cur.cross(horizontal_err.normalized())(2),0.5);  
    }
  }

  ROS_WARN("(%0.3f, %0.3f) %0.3f, %0.3f, %0.3f", drone_heading(0),drone_heading(1), horizontal_err.norm(), vertical_err,  drone_odom.twist.twist.linear.z);
  vel_setpoint.header.stamp = ros::Time::now();
  local_vel_pub.publish(vel_setpoint);
}
