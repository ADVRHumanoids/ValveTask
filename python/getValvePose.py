#!/usr/bin/env python

"""
This script return the pose of valve model wrt gazebo world frame
"""

import sys
import rospy
from gazebo_ros.gazebo_interface import *


def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    res = gms_client("valve","world")
    print("res", res)