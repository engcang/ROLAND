
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h> //kill service
#include <signal.h>
#include "pid_controller.h"
#include "utility.h"

/* namespace */
using namespace Eigen;

class drone{
  public:

    /*--------------------
        < Variables >
    --------------------*/

    mavros_msgs::State current_state;
    nav_msgs::Odometry drone_odom;
    nav_msgs::Odometry mobile_odom;
    geometry_msgs::PoseWithCovarianceStamped mobile_odom_ekf;
    geometry_msgs::PoseStamped pose_diff;
    geometry_msgs::TwistStamped mobile_vel;
    Vector3f drone_pose = MatrixXf::Zero(3,1);
    Vector3f drone_heading = MatrixXf::Zero(3,1);
    Vector3f land_pose = MatrixXf::Zero(3,1);
    geometry_msgs::TwistStamped vel_setpoint;
    
    /* controller */
    PID_controller PID_x;
    PID_controller PID_y;
    float Kp_xy = 1.5f;
    float Ki_xy = 0.03f;
    float Kd_xy = 0.0f;
    float vel_limit_xy = 1.5f;
    float integ_limit_xy = 10.0f;
    float err_bound = 0.25f;
    float Kp_z = 1.5f;
    float descend_vel_limit = 0.7f;
    

    /* land vel estimation */
    Vector3f land_pose_last = MatrixXf::Zero(3,1);
    Vector3f land_vel = MatrixXf::Zero(3,1);
    Vector3f land_vel_raw = MatrixXf::Zero(3,1);
    ros::Time last_time;
    double dt= 0.0;
    double a = 0.3;

    /* ros */
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber mobile_odom_sub;
    ros::Subscriber mobile_odom_ekf_sub;
    ros::Subscriber pose_diff_sub;
    ros::Publisher local_vel_pub;
    ros::Publisher land_pose_pub;
    ros::ServiceClient kill;

    /*--------------------
       < Class Methods > 
    --------------------*/

    /* callback functions */
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void mobile_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void mobile_odom_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void pose_diff_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mobile_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    /* etc */
    void run();

    /* init funtion */
    drone(ros::NodeHandle& n) : nh(n){

      /* subscriber */ 
      state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &drone::state_callback, this);
      odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &drone::odom_callback, this);
      mobile_odom_sub = nh.subscribe<nav_msgs::Odometry>("/jackal1/jackal_velocity_controller/odom", 10, &drone::mobile_odom_callback, this);
      mobile_odom_ekf_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 10, &drone::mobile_odom_ekf_callback, this);
      pose_diff_sub = nh.subscribe<geometry_msgs::PoseStamped>("/estimated_pose_diff", 10, &drone::pose_diff_callback, this);      

      /* pubulisher */
      local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
      land_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/land_pose", 10);

      /* service kill switch */
      kill = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

      /* set PID parameters */
      PID_x.set_parameter(Kp_xy, Ki_xy, Kd_xy, vel_limit_xy, integ_limit_xy);
      PID_y.set_parameter(Kp_xy, Ki_xy, Kd_xy, vel_limit_xy, integ_limit_xy);

      tic();
      last_time = ros::Time::now();
      ROS_WARN("Class generated, started node...");
    }
};


void drone::state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void drone::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  drone_odom=*msg;
  drone_pose << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
  
  /* get heading of drone */
  Quaternionf q;
  Vector3f temp;
  temp << 1, 0, 0;
  q.x() = drone_odom.pose.pose.orientation.x;
  q.y() = drone_odom.pose.pose.orientation.y;
  q.z() = drone_odom.pose.pose.orientation.z;
  q.w() = drone_odom.pose.pose.orientation.w;
  drone_heading = q.toRotationMatrix()*temp;
}

void drone::mobile_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  mobile_odom=*msg;
}

void drone::mobile_odom_ekf_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  mobile_odom_ekf=*msg;

  /* get velocity of mobile robot in world frame */
  Quaternionf q;
  Vector3f temp;
  temp << mobile_odom.twist.twist.linear.x, mobile_odom.twist.twist.linear.y, mobile_odom.twist.twist.linear.z;
  q.x() = mobile_odom_ekf.pose.pose.orientation.x;
  q.y() = mobile_odom_ekf.pose.pose.orientation.y;
  q.z() = mobile_odom_ekf.pose.pose.orientation.z;
  q.w() = mobile_odom_ekf.pose.pose.orientation.w;
  land_vel = q.toRotationMatrix()*temp;
}


void drone::pose_diff_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_diff=*msg;

  /* get position of target(mobile robot) */
  land_pose << pose_diff.pose.position.x + drone_pose(0), 
  pose_diff.pose.position.y + drone_pose(1),
  pose_diff.pose.position.z + drone_pose(2);

  /* get velocity of target(mobile robot) */
  dt = double(ros::Time::now().toSec()-last_time.toSec());
  land_vel_raw = (land_pose-land_pose_last)/dt;
  last_time = ros::Time::now();
  land_pose_last = land_pose;

  //land_vel = (1-a)*land_vel + a*land_vel_raw;
  
  /* publish land pose for rviz */
  geometry_msgs::PoseStamped land_pose_msg;
  land_pose_msg.pose.position.x = land_pose(0);
  land_pose_msg.pose.position.y = land_pose(1);
  land_pose_msg.pose.position.z = land_pose(2);
  land_pose_msg.header.frame_id = "map";
  land_pose_msg.header.stamp = ros::Time::now();
  land_pose_pub.publish(land_pose_msg);
}


void drone::run(){

  Vector3f heading_cur;
  Vector3f target_pose;
  Vector3f horizontal_err; // seperate pos err into horizontal component & vertical component
  float vertical_err;
  
  heading_cur = drone_heading;
  heading_cur(2) = 0; // use only yaw component
  heading_cur.normalize();

  target_pose = land_pose;
  /* for better camera view, descend obliquely */
  target_pose(0) += 0.3*heading_cur(0)*(land_pose(2)-drone_pose(2) -0.15); //bigger value decreases slope
  target_pose(1) += 0.3*heading_cur(1)*(land_pose(2)-drone_pose(2)-0.15);
  
  vertical_err = target_pose(2)-drone_pose(2);
  horizontal_err = target_pose - drone_pose;
  horizontal_err(2) = 0; // remove vertical component
  
  /* if horizontal err is small enough -> decend & land */
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  if (horizontal_err.norm() < err_bound and duration.count()/1000000.0 > 50.0)
  {
    /* follow target */
    vel_setpoint.twist.linear.x = PID_x.control(target_pose(0), drone_pose(0), land_vel(0));
    vel_setpoint.twist.linear.y = PID_y.control(target_pose(1), drone_pose(1), land_vel(1));
    
    /* smaller err -> faster descend */
    vel_setpoint.twist.linear.z = -descend_vel_limit*(err_bound-horizontal_err.norm())/err_bound;
    vel_setpoint.twist.angular.z = 0;
    
    /* if landing detected -> exit program */
    if (drone_odom.twist.twist.linear.z<0.03 && drone_odom.twist.twist.linear.z>-0.03 && vertical_err>-0.4)
    {
      // kill to stop drone
      mavros_msgs::CommandLong service;
      service.request.command = 400;
      service.request.param2 = 21196.0;
      if(kill.call(service) && service.response.success)
        exit(0);
    }
    
  }

  /* if horizontal err is not small enough */
  else
  { 
    /* follow target */
    vel_setpoint.twist.linear.x = PID_x.control(target_pose(0), drone_pose(0), land_vel(0));
    vel_setpoint.twist.linear.y = PID_y.control(target_pose(1), drone_pose(1), land_vel(1));
    
    /* maintain altitude */
    vel_setpoint.twist.linear.z = saturate(Kp_z*(vertical_err+2),-0.5, 0.5);
    
    /* move heading to mobile robot */
    if (horizontal_err.norm() > err_bound+0.1){
      vel_setpoint.twist.angular.z = saturate(1.5*heading_cur.cross((land_pose-drone_pose).normalized())(2),-1,1);    
    }
  }

  /* debug */
  ROS_WARN("err_xy:%0.3f, err_z:%0.3f, vel_z:%0.3f",  horizontal_err.norm(), vertical_err,  drone_odom.twist.twist.linear.z);
  
  vel_setpoint.header.stamp = ros::Time::now();
  local_vel_pub.publish(vel_setpoint);
}
