#!/usr/bin/env python

"""
This script will command the robot to given desired location [x(m), y(m), yaw(deg)]

usage:
./walkTo.py 0.2, 0.2, 45
"""

import tf
import sys
import rospy
import numpy as np
import tf.transformations as trans
from ADVR_ROS.srv import *
from gazebo_ros.gazebo_interface import *

cmd_type = {'WalkFront': 1,
            'WalkBack': 2,
            'WalkLeft': 3,
            'WalkRight': 4,
            'TurnLeft': 5,
            'TurnRight': 6}


def poseToPositionQuaternion(pose):
    position = np.array([pose.position.x,
                         pose.position.y,
                         pose.position.z])

    orientation = np.array([pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w])
    return position, orientation


def poseToMatrix(pose):
    position = np.array([pose.position.x,
                         pose.position.y,
                         pose.position.z])
    translation_matrix = trans.translation_matrix(position)
    orientation = np.array([pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w])
    rotation_matrix = trans.quaternion_matrix(orientation)
    transformation_matrix = translation_matrix.dot(rotation_matrix)
    return transformation_matrix


def matrixToPose(Trans):
    position = Trans[0:3, 3]
    orientation = trans.quaternion_from_matrix(Trans)

    pose = geometry_msgs.msg.Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def get_model_state(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name, 'world')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def get_walkman_state():
    state = get_model_state('walkman')
    position = np.array([state.pose.position.x,
                         state.pose.position.y,
                         state.pose.position.z])

    quaternion = np.array([state.pose.orientation.x,
                           state.pose.orientation.y,
                           state.pose.orientation.z,
                           state.pose.orientation.w])

    twist_linear = np.array([state.twist.linear.x,
                             state.twist.linear.y,
                             state.twist.linear.z])

    twist_angular = np.array([state.twist.angular.x,
                              state.twist.angular.y,
                              state.twist.angular.z])

    rpy = trans.euler_from_quaternion(quaternion)

    translation_matrix = trans.translation_matrix(position)

    rotation_matrix = trans.quaternion_matrix(quaternion)

    transformation_matrix = translation_matrix.dot(rotation_matrix)

    state = {"position": position,
             "quaternion": quaternion,
             "twist_linear": twist_linear,
             "twist_angular": twist_angular,
             "rpy": rpy,
             "translation_matrix": translation_matrix,
             "rotation_matrix": rotation_matrix,
             "transformation_matrix": transformation_matrix}

    return state




def walking_srv_client(cmd_name, quantity=0.2, step_length=0.15, execute=True):
    rospy.wait_for_service('/walking_command')
    try:
        ws = rospy.ServiceProxy('/walking_command', advr_locomotion)
        request = advr_locomotionRequest()
        request.command_type = cmd_type[cmd_name]
        request.command_value = quantity
        request.step_length = 0.15
        request.execute = True
        responce = ws(request)
        return responce
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "Usage: %s [desired x(m)] [desired y(m)] [desired yaw(deg)] \nFor example: ./walkTo.py 0.2, 0.2, 45" % \
           sys.argv[0]


if __name__ == "__main__":

    if len(sys.argv) == 4:
        world_x_goal = float(sys.argv[1])
        world_y_goal = float(sys.argv[2])
        world_yaw_goal = np.deg2rad(float(sys.argv[3]))

        # generate desired frame wrt world
        world_translation_goal = trans.translation_matrix(np.array([world_x_goal, world_y_goal, 0.0]))
        world_rotation_goal = trans.euler_matrix(0.0, 0.0, world_yaw_goal, 'sxyz')
        world_T_goal = world_translation_goal.dot(world_rotation_goal)

        # get walkman location wrt world
        # walkman_state = get_model_state('walkman')
        # walkman_state.pose.position.z = 0.0
        # world_T_walkman = poseToMatrix(walkman_state.pose)
        #
        # world_P_walkman, world_Quaternion_walkman = poseToPositionQuaternion(walkman_state.pose)
        # world_RPY_walkman = trans.euler_from_quaternion(world_Quaternion_walkman)

        world_State_walkman = get_walkman_state()
        print("walkman [x,y,z,roll,pitch,yaw]:", world_State_walkman['position'], world_State_walkman['rpy'])


        # Step 1: turn to face the goal position
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        turn_angle = np.math.atan2(walkman_P_goal[1], walkman_P_goal[0])
        if turn_angle > 0:
            print("turn left:", np.rad2deg(turn_angle), "deg")
            responce_1 = walking_srv_client('TurnLeft', quantity=np.rad2deg(turn_angle))
        else:
            print("turn right:", np.rad2deg(turn_angle), "deg")
            responce_1 = walking_srv_client('TurnRight', quantity=np.rad2deg(turn_angle))

        # check whether goal yaw is reached
        world_Yaw_goal = world_State_walkman['rpy'][2] + turn_angle
        world_State_walkman = get_walkman_state()
        while np.abs(world_State_walkman['rpy'][2] - world_Yaw_goal) > np.deg2rad(0.1):
            world_State_walkman = get_walkman_state()
            # print("yaw error: ", np.deg2rad(np.abs(world_State_walkman['rpy'][2] - world_Yaw_goal)))
        rospy.sleep(4)


        # Step 2: walk straight to the goal position
        world_State_walkman = get_walkman_state()
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        distance = np.linalg.norm(walkman_P_goal[:2])
        print("walk front:", distance, "m")
        responce_2 = walking_srv_client('WalkFront', quantity=distance)

        # check whether goal position is reached
        world_State_walkman = get_walkman_state()
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        while np.abs(np.linalg.norm(walkman_P_goal[:2])) > 0.05:
            world_State_walkman = get_walkman_state()
            walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
            walkman_P_goal = walkman_T_goal[:3, -1]
            walkman_R_goal = walkman_T_goal[:3, :3]
        rospy.sleep(5)


        # Step 3: turn to the goal orientation
        world_State_walkman = get_walkman_state()
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        walkman_RPY_goal = trans.euler_from_matrix(walkman_R_goal)
        turn_angle = walkman_RPY_goal[2]
        if turn_angle > 0:
            print("turn left:", np.rad2deg(turn_angle), "deg")
            responce_3 = walking_srv_client('TurnLeft', quantity=np.rad2deg(turn_angle))
        else:
            print("turn right:", np.rad2deg(turn_angle), "deg")
            responce_3 = walking_srv_client('TurnRight', quantity=np.rad2deg(turn_angle))


        # check whether goal yaw is reached

        world_State_walkman = get_walkman_state()
        while np.abs(world_State_walkman['rpy'][2] - world_yaw_goal) > np.deg2rad(0.1):
            world_State_walkman = get_walkman_state()
            # print("yaw error: ", np.deg2rad(np.abs(world_State_walkman['rpy'][2] - world_Yaw_goal)))
        rospy.sleep(4)


    else:
        print(usage())
        sys.exit(1)
