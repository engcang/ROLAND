# TODO
+ floor removing (for PCL average center point)
+ kalman filter
    + VIO, UWB, center point

<br>

## RGBD: 6m, UWB: 10m, bbox further than 6m-> Z=UWB

<br>

# iccas2021
## Robust Landing of UAV on Moving Platform using Object Detection and UWB based Extended Kalman Filter

<br>

### Requirements
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
$ echo "export GAZEBO_MODEL_PATH=:/home/<your_pcname>/<your_workspace>/src/iccas2021/gazebo_model_and_world:$GAZEBO_MODEL_PATH" >> ~/.bashrc
$ echo "export LD_LIBRARY_PATH=:/home/<your_pcname>/<your_workspace>/devel/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
$ . ~/.bashrc
~~~

<br>

### Components
+ ROS-YOLO using OpenCV/OpenVINO code: from [here (myself)](https://github.com/engcang/ros-yolo-sort/blob/master/YOLO_and_ROS_ver/ros_opencv_dnn.py)
+ Jackal Gazebo model: from [here](https://github.com/jackal)
+ Drone and Jackal controll joystick code: from [here (myself)](https://github.com/engcang/mavros-gazebo-application/blob/master/mavros_joy_controller.py), also refer [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#mission--joystick-controller---supports-kobuki-and-jackal)
+ UWB Gazebo sensor plugin and message from [uwb gazebo plugin](https://github.com/valentinbarral/gazebosensorplugins) and [uwb ROS msg](https://github.com/valentinbarral/rosmsgs)
