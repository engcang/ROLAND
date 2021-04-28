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
