#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: chanyoung
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, Thrust


#for signal_handler
import sys 
import signal


global mav_check
mav_check=0

global d2r
global r2d
global max_vel_x
global max_vel_y
global max_vel_z
global yaw_rate
yaw_rate = 2

r2d = 180/np.pi
d2r = np.pi/180
max_vel_x = 12
max_vel_y = 12
max_vel_z = 12

global commands
commands = [["takoff",[0,0,4]],["go",[2,2,4]],["go",[-2,2,4]],["go",[-2,-2,4]],["go",[2,-2,4]],["go",[0,0,4]],["land",[0,0,4]]]


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def norm(vec):
    return (vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2])**0.5

class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.rate = rospy.Rate(30)

    def pose_callback(self, msg):
        global mav_check
        self.pose=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        mav_check=1
    
    def odom_callback(self, msg):
        global mav_check
        self.vel = msg.twist.twist.linear
        mav_check=1


class Offboard():
    def __init__(self,rbt):
        self.rbt = rbt
        self.commandList = []
        self.maxErrRad = 0.3
    
    def addCommand(self,command):
        self.commandList.append(command)
    
    def addCommandList(self,commandList):
        self.commandList.extend(commandList)

    def run(self):
        vel_input=TwistStamped()
        if len(self.commandList) != 0:
            command = self.commandList[0]
            print("%s,%d\n"%(command[0],len(self.commandList)))
            if command[0] == "takeoff":
                self.rbt.arming(True)

                pose_des = command[1]
                pose_cur = self.rbt.pose
                pose_err = [pose_des[0]-pose_cur.x,pose_des[1]-pose_cur.y,pose_des[2]-pose_cur.z]
                normE = min(norm(pose_err),5)/norm(pose_err)
                vel_des = [pose_err[0]*normE,pose_err[1]*normE,pose_err[2]*normE]

                self.commandList.pop(0)
                if norm(pose_err) > self.maxErrRad:
                    self.commandList.insert(0,["go",pose_des])
                    self.commandList.insert(0,["go",[pose_cur.x,pose_cur.y,pose_des[2]]])
                    

            elif command[0] == "go":
                pose_des = command[1]
                pose_cur = self.rbt.pose
                pose_err = [pose_des[0]-pose_cur.x,pose_des[1]-pose_cur.y,pose_des[2]-pose_cur.z]
                normE = min(norm(pose_err),10)/norm(pose_err)
                vel_cur = self.rbt.vel
                vel_des = [pose_err[0]*normE,pose_err[1]*normE,pose_err[2]*normE]

                limit = 1
                
                vel_input.twist.linear.x= max(min(vel_des[0], vel_cur.x+limit),vel_cur.x-limit)
                vel_input.twist.linear.y= max(min(vel_des[1], vel_cur.y+limit),vel_cur.y-limit)
                vel_input.twist.linear.z= vel_des[2]
                #vel_input.twist.angular.z = yaw_rate*(rbt.joy.axes[0])


                if norm(pose_err) < self.maxErrRad:
                    self.commandList.pop(0)
                    vel_input.twist.linear.x= 0
                    vel_input.twist.linear.y= 0
                    vel_input.twist.linear.z= 0

            elif command[0] == "landing":
                
                
                pose_des = command[1]
                pose_cur = self.rbt.pose
                pose_err = [pose_des[0]-pose_cur.x,pose_des[1]-pose_cur.y,0]
                normE = min(norm(pose_err),5)/norm(pose_err)
                vel_des = [pose_err[0]*normE,pose_err[1]*normE,pose_err[2]*normE]

                if norm(pose_err) > self.maxErrRad:
                    self.commandList.insert(0,["go",[pose_des[0],pose_des[1],pose_cur.z]])
                else :
                    vel_input.twist.linear.x= vel_des[0]
                    vel_input.twist.linear.y= vel_des[1]
                    
                    
                    if abs(pose_cur.z-pose_des[2])>1:
                        vel_input.twist.linear.z= -0.2 - min(2,abs(pose_cur.z-pose_des[2])-1)
                    else:
                        vel_input.twist.linear.z= -0.2

                    if abs(self.rbt.vel.z) < 0.05 and abs(pose_cur.z-pose_des[2])<0.3:
                        self.commandList.pop(0)
                        vel_input.twist.linear.x= 0
                        vel_input.twist.linear.y= 0
                        vel_input.twist.linear.z= 0


                
                

        vel_input.header.stamp = rospy.Time.now()
        self.rbt.local_vel_pub.publish(vel_input)

            


        
##############################################################################################

mav_ctr = robot()
ofb_ctr = Offboard(mav_ctr)
ofb_ctr.addCommandList([
    ["takeoff",[0,0,2]],
    ["go",[3.6,0,2]],
    ["go",[3.6,-3.6,2]],
    ["go",[-3.5,-3.5,2]],
    ["go",[-3.5,-8.5,2]],
    ["go",[0,-11,2]],
    ["go",[5.5,-11,2]],
    ["go",[5.5,-9,2]],
    ["go",[3.8,-11,2]],
    ["go",[0,-11,2]],
    ["go",[-3.5,-8.5,2]],
    ["go",[-3.5,-3.5,2]],
    ["go",[3.6,-3.6,2]],
    ["go",[3.6,0,2]],
    ["landing",[0,0,0]]
    ])
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if mav_check==1:
                if len(ofb_ctr.commandList)==0:
                    sys.exit(0)
                ofb_ctr.run()
                mav_ctr.rate.sleep()
            else: 
                mav_ctr.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass