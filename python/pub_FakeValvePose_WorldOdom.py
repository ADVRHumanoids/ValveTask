import numpy as np
import random
import scipy.io as sio
import os, sys, cv2
import argparse


import time
import rospy
from geometry_msgs.msg import PoseStamped




if __name__ == '__main__':

    rospy.init_node('fake_valve_pose_pub_node')
    pub = rospy.Publisher('valve_pose', PoseStamped, queue_size=1)

    time.sleep(0.5)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world_odom'
    msg.pose.position.x = 0.8
    msg.pose.position.y = -0.4
    msg.pose.position.z = 1.5
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = -0.7071070192004544
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.7071070192004544

    pub.publish(msg)

