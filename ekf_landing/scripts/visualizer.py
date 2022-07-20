#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon June 28 02:28:30 2021

@author: mason
"""

''' import libraries '''
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class viz():
    def __init__(self):
        rospy.init_node('viz', anonymous=True)
        rospy.Subscriber("/robot5/odometry/filtered", Odometry, self.encoder_cb)
        rospy.Subscriber("/uav/mavros/local_position/pose", PoseStamped, self.vio_cb)
        rospy.Subscriber("/estimated_pose_diff", PoseStamped, self.estimate_cb)

        self.vio = rospy.Publisher("vio", PointStamped, queue_size=2)
        self.encoder = rospy.Publisher("encoder", PointStamped, queue_size=2)
        self.vio_drone_path_pub = rospy.Publisher("vio_drone_path", Path, queue_size=2)
        self.encoder_mobile_path_pub = rospy.Publisher("encoder_mobile_path", Path, queue_size=2)
        self.estimated_drone = rospy.Publisher("estimated_drone", PointStamped, queue_size=2)
        self.estimated_mobile = rospy.Publisher("estimated_mobile", PointStamped, queue_size=2)
        self.estimated_drone_path_pub = rospy.Publisher("estimated_drone_path", Path, queue_size=2)
        self.estimated_mobile_path_pub = rospy.Publisher("estimated_mobile_path", Path, queue_size=2)

        self.drone_check = False
        self.mobile_check = False
        self.estimated_check=False
        self.rate=rospy.Rate(5)

    def encoder_cb(self, msg):
        d=msg.pose.pose.position
        self.encoder_mobile_pose = PoseStamped()
        self.encoder_mobile_pose.header.stamp = rospy.Time.now()
        self.encoder_mobile_pose.header.frame_id = "map"
        self.encoder_mobile_pose.pose.position = d
        self.encoder_mobile_pose.pose.orientation.w = 1
        m = PointStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.point.x = d.x
        m.point.y = d.y
        m.point.z = d.z
        self.encoder.publish(m)
        self.mobile_check = True

    def vio_cb(self, msg):
        d=msg.pose.position
        self.vio_drone_pose = msg
        self.vio_drone_pose.header.stamp = rospy.Time.now()
        self.vio_drone_pose.header.frame_id = "map"
        self.vio_drone_pose.pose.orientation.w = 1
        m = PointStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.point.x = d.x
        m.point.y = d.y
        m.point.z = d.z
        self.vio.publish(m)
        self.drone_check = True

    def estimate_cb(self, msg):
        if (self.mobile_check and self.drone_check):
            d=msg.pose.position
            #d: mobile-drone
            m = PointStamped()
            m2 = PointStamped()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.point.x = d.x + self.vio_drone_pose.pose.position.x
            m.point.y = d.y + self.vio_drone_pose.pose.position.y
            m.point.z = d.z + self.vio_drone_pose.pose.position.z
            m2.header.stamp = rospy.Time.now()
            m2.header.frame_id = "map"
            m2.point.x = self.encoder_mobile_pose.pose.position.x - d.x
            m2.point.y = self.encoder_mobile_pose.pose.position.y - d.y
            m2.point.z = self.encoder_mobile_pose.pose.position.z - d.z

            self.estimated_mobile_pose = PoseStamped()
            self.estimated_drone_pose = PoseStamped()
            self.estimated_mobile_pose.header.stamp = rospy.Time.now()
            self.estimated_mobile_pose.header.frame_id = "map"
            self.estimated_mobile_pose.pose.position = m.point
            self.estimated_mobile_pose.pose.orientation.w = 1
            self.estimated_drone_pose.header.stamp = rospy.Time.now()
            self.estimated_drone_pose.header.frame_id = "map"
            self.estimated_drone_pose.pose.position = m2.point
            self.estimated_drone_pose.pose.orientation.w = 1

            self.estimated_check=True
            self.estimated_mobile.publish(m)
            self.estimated_drone.publish(m2)

''' main '''
pub_class = viz()
vio_drone_path = Path()
encoder_mobile_path = Path()
estimated_drone_path = Path()
estimated_mobile_path = Path()
vio_drone_path.header.frame_id = "map"
encoder_mobile_path.header.frame_id = "map"
estimated_drone_path.header.frame_id = "map"
estimated_mobile_path.header.frame_id = "map"

if __name__ == '__main__':
    while 1:
        try:
            if (pub_class.mobile_check and pub_class.drone_check and pub_class.estimated_check):
                vio_drone_path.header.stamp = rospy.Time.now()
                encoder_mobile_path.header.stamp = rospy.Time.now()
                estimated_drone_path.header.stamp = rospy.Time.now()
                estimated_mobile_path.header.stamp = rospy.Time.now()
                vio_drone_path.poses.append(pub_class.vio_drone_pose)
                encoder_mobile_path.poses.append(pub_class.encoder_mobile_pose)
                estimated_drone_path.poses.append(pub_class.estimated_drone_pose)
                estimated_mobile_path.poses.append(pub_class.estimated_mobile_pose)
                pub_class.vio_drone_path_pub.publish(vio_drone_path)
                pub_class.encoder_mobile_path_pub.publish(encoder_mobile_path)
                pub_class.estimated_drone_path_pub.publish(estimated_drone_path)
                pub_class.estimated_mobile_path_pub.publish(estimated_mobile_path)
            pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
