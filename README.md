# TODO
+ Jackal encoder gazebo
~~~xml
  <gazebo>
    <plugin filename="libgazebo_ros_skid_steer_drive.so" name="skid_steer_drive_controller">
<!--      <robotNamespace></robotNamespace>--> 
      <updateRate>10.0</updateRate>
      <robotBaseFrame>base_link</robotBaseFrame>
      <wheelSeparation>0.1</wheelSeparation>
      <wheelDiameter>0.08</wheelDiameter>
      <torque>0.5</torque>
      <leftFrontJoint>front_left_wheel_hinge</leftFrontJoint>
      <rightFrontJoint>front_right_wheel_hinge</rightFrontJoint>
      <leftRearJoint>rear_left_wheel_hinge</leftRearJoint>
      <rightRearJoint>rear_right_wheel_hinge</rightRearJoint>
      <topicName>cmd_vel</topicName>
      <commandTopic>cmd_vel</commandTopic>
      <broadcastTF>true</broadcastTF>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <covariance_x>0.000100</covariance_x>
      <covariance_y>0.000100</covariance_y>
      <covariance_yaw>0.010000</covariance_yaw>
    </plugin>
  </gazebo>
~~~
+ box to 3d PCL RGBD code
+ PCL average center point
+ kalman filter
+ UWB on Jackal and Drone
+ world, model(Drone with UWB and camera)

<br>

## RGBD: 6m, UWB: 15m, bbox further than 6m-> Z=UWB

<br>

# iccas2021
## Robust Landing of UAV on Moving Platform using Object Detection and UWB based Extended Kalman Filter

<br>

### Requirements
+ ROS and Gazebo
    + refer [here](http://wiki.ros.org/ROS/Installation)
    + `$ sudo apt install ros-<distro>-desktop-full`
+ PX4-SITL: [here](https://github.com/PX4/PX4-SITL_gazebo)
    + Installation: [here](https://github.com/engcang/mavros-gazebo-application#installation)
+ `tf_to_trajectory` pacakge from [here](https://github.com/engcang/tf_to_trajectory)

<br>

### Components
+ ROS-YOLO using OpenCV/OpenVINO code: from [here](https://github.com/engcang/ros-yolo-sort/blob/master/YOLO_and_ROS_ver/ros_opencv_dnn.py)
+ Jackal Gazebo model: from [here](https://github.com/jackal)
+ Drone and Jackal controll joystick code: from [here](https://github.com/engcang/mavros-gazebo-application/blob/master/mavros_joy_controller.py), also refer [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#mission--joystick-controller---supports-kobuki-and-jackal)
