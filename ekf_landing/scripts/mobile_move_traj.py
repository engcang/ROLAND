#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, sqrt, cos, sin, atan2
import numpy as np

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def distance(x, y, x2, y2):
    return sqrt(pow(x-x2, 2)+pow(y-y2, 2))

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

def vel_saturation(vel, max_val):
    if vel>max_val:
        vel=max_val
    if vel<-max_val:
        vel=-max_val
    return vel


''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        # self.traj = rospy.get_param("/traj", "square")
        self.traj = rospy.get_param("/traj", "circle")
        self.robot_vel_topic = rospy.get_param("/mobile_robot_vel_topic", "/jackal1/jackal_velocity_controller/cmd_vel")
        self.mobile_robot_vel_pub = rospy.Publisher(self.robot_vel_topic, Twist, queue_size=10)
        self.gt_poses = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gtcallback)

        self.rate = rospy.Rate(30)
        self.check= 0

    def gtcallback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]=="jackal1":
                self.pos = msg.pose[i].position
                (roll, pitch, self.yaw) = euler_from_quaternion([msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w])
                self.check= 1

##############################################################################################

mobile_move = robot()
time.sleep(1) #wait 1 second to assure that all data comes in
waypoints=[[10, 0], [10, 10], [0, 10], [0, 0]]
idx=0
tolerance=1.2
k1=2; k2=2;
v_max = 1.0
w_max = 1.0
radius = 3.0

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if mobile_move.check==1:
                input_v = Twist()
                if mobile_move.traj=="square":
                    rho = distance(mobile_move.pos.x, mobile_move.pos.y, waypoints[idx%4][0], waypoints[idx%4][1])
                    if rho < tolerance:
                        idx=idx+1
                    psi=atan2(waypoints[idx%4][1]-mobile_move.pos.y, waypoints[idx%4][0]-mobile_move.pos.x)
                    theta=mobile_move.yaw
                    phi = rpy_saturation(theta-psi)
                    input_v.linear.x = vel_saturation(k1*rho*cos(phi), v_max)
                    input_v.angular.z = vel_saturation(-k1*sin(phi)*cos(phi)-k2*phi, w_max)
                elif mobile_move.traj=="circle":
                    input_v.linear.x = v_max
                    input_v.angular.z = v_max/radius
                else:
                    print("not correct trajectory type")
                mobile_move.mobile_robot_vel_pub.publish(input_v)
            mobile_move.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass