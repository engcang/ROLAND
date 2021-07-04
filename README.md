## ROLAND: Robust Landing of UAV on Moving Platform\\using Object Detection and UWB based Extended Kalman Filter
#### ChanYoung Kim, EungChang Mason Lee, JunHo Choi, JinWoo Jeon, SeokTae Kim, and Hyun Myung, "ROLAND: Robust Landing of UAV on Moving Platform using Object Detection and UWB based Extended Kalman Filter," in *Proc. of International Conference on Control, Automation and System (ICCAS)*, 2021.

<br>

### Requirements

<details><summary>[click to see]</summary>

+ (Optional) `Joystick`, I used `sony dualshock4`
+ `OpenCV` upper than 4.4.0 for OpenCV-DNN-YOLO, and also `cv_bridge`
    + `OpenCV`, `cv_bridge` manual build refer [here](https://github.com/engcang/ros-yolo-sort/tree/master/YOLO_and_ROS_ver#2-prerequisites)
+ ROS and Gazebo
    + refer [here](http://wiki.ros.org/ROS/Installation)
    + `$ sudo apt install ros-<distro>-desktop-full`
+ `robot-pose-ekf` package, refer [here](http://wiki.ros.org/robot_pose_ekf)
~~~shell
$ sudo apt install ros-<distro>-robot-pose-ekf
~~~
+ PX4-SITL: [here](https://github.com/PX4/PX4-SITL_gazebo)
    + Installation: [here](https://github.com/engcang/mavros-gazebo-application#installation)
+ This repo, which is including submodules as belows:
    + [uwb gazebo plugin](https://github.com/valentinbarral/gazebosensorplugins)
    + [uwb ROS msg](https://github.com/valentinbarral/rosmsgs)
    + `tf_to_trajectory` pacakge from [here (myself)](https://github.com/engcang/tf_to_trajectory)
~~~shell
$ git clone --recursive https://github.com/engcang/iccas2021

or

$ git clone https://github.com/engcang/iccas2021
$ cd iccas2021
$ git submodule update --init --recursive
~~~

+ [Important] Set gazebo model path
~~~shell
$ echo "export GAZEBO_MODEL_PATH=:/home/<your_pcname>/<your_workspace>/src/iccas2021/gazebo_maps/bounding_wall_world:/home/<your_pcname>/<your_workspace>/src/iccas2021/drone_package_for_gazebo:$GAZEBO_MODEL_PATH" >> ~/.bashrc
$ echo "export LD_LIBRARY_PATH=:/home/<your_pcname>/<your_workspace>/devel/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
$ . ~/.bashrc
~~~

+ For VINS-Fusion,
~~~shell
$ echo "export MALLOC_CHECK_=0" >> ~/.bashrc
$ source ~/.bashrc
~~~

---

</details>

<br>

### Components

<details><summary>[click to see]</summary>

+ ROS-YOLO using OpenCV/OpenVINO code: from [here (myself)](https://github.com/engcang/ros-yolo-sort/blob/master/YOLO_and_ROS_ver/ros_opencv_dnn.py)
+ Jackal Gazebo model: from [here](https://github.com/jackal)
+ Drone and Jackal controll joystick code: from [here (myself)](https://github.com/engcang/mavros-gazebo-application/blob/master/mavros_joy_controller.py), also refer [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#mission--joystick-controller---supports-kobuki-and-jackal)
+ UWB Gazebo sensor plugin and message from [uwb gazebo plugin](https://github.com/valentinbarral/gazebosensorplugins) and [uwb ROS msg](https://github.com/valentinbarral/rosmsgs)
+ [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) `frame_id` and `OpenCV` edited version from [here](https://github.com/engcang/vins-application#-vins-fusion-1)
    + `camera_models` package is edited to be compatible with `OpenCV4`
+ Gazebo [map (myself)](https://github.com/engcang/gazebo_maps)

---

</details>

<br>

### How to run

<details><summary>[click to see]</summary>

+ launch gazebo world
~~~shell
$ roslaunch ekf_landing world.launch
~~~
+ start `kalman filter node`
~~~shell
$ roslaunch ekf_landing kalman.launch
~~~
+ move around the drone / jackal: control `joystick`
+ start `autolanding node`
~~~shell
$ rosrun ekf_landing autolanding_node
~~~
---

</details>
