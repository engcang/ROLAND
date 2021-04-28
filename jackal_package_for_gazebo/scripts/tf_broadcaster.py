#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.parent_frame_id = rospy.get_param("/parent_frame_id", 'world')
        self.robot_name = rospy.get_namespace()

        self.gt_poses = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gtcallback)

        self.rate = rospy.Rate(30)
        self.check=0
        self.br = tf.TransformBroadcaster()


    def gtcallback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]==self.robot_name[1:-1]:
                self.pose = msg.pose[i]
                self.check= 1

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            if cas.check==1:
                cas.br.sendTransform((cas.pose.position.x, cas.pose.position.y, cas.pose.position.z),\
        (cas.pose.orientation.x,cas.pose.orientation.y,cas.pose.orientation.z,cas.pose.orientation.w),\
        rospy.Time.now(),cas.robot_name+"base_link", cas.parent_frame_id)
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
