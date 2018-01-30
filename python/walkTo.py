#!/usr/bin/env python

"""
This script will command the robot to given desired location [x(m), y(m), yaw(deg)]

usage:
./walkTo.py 0.2, 0.2, 45
"""

import tf
import sys
import rospy
import angles
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

turn_threshold = 1.0 #deg
close_threshold = 0.1
walk_threshold = 0.1 #m


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
        world_yaw_goal = angles.normalize_angle(world_yaw_goal)

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

        if np.abs(np.linalg.norm(walkman_P_goal[:2])) > 0.2:
            turn_angle = np.math.atan2(walkman_P_goal[1], walkman_P_goal[0])
            if turn_angle > 0:
                print("turn left:", np.rad2deg(turn_angle), "deg")
                responce_1 = walking_srv_client('TurnLeft', quantity=np.rad2deg(turn_angle))
            else:
                print("turn right:", np.rad2deg(turn_angle), "deg")
                responce_1 = walking_srv_client('TurnRight', quantity=np.rad2deg(turn_angle))

            # check whether goal yaw is reached
            world_facing_yaw_goal = world_State_walkman['rpy'][2] + turn_angle
            world_facing_yaw_goal = angles.normalize_angle(world_facing_yaw_goal)
            world_State_walkman = get_walkman_state()
            while np.abs(world_State_walkman['rpy'][2] - world_facing_yaw_goal) > np.deg2rad(turn_threshold):
                world_State_walkman = get_walkman_state()
                # print("robot yaw: ", world_State_walkman['rpy'][2], "world_facing_yaw_goal: ", world_facing_yaw_goal)
                # print("yaw error: ", np.abs(world_State_walkman['rpy'][2] - world_facing_yaw_goal))
            rospy.sleep(4)


        # Step 2: walk straight to the goal position
        world_State_walkman = get_walkman_state()
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        distance = np.linalg.norm(walkman_P_goal[:2])
        print("walk front:", distance, "m")
        responce_2 = walking_srv_client('WalkFront', quantity=distance)

        # check whether the robot is close enough to the goal position
        world_State_walkman = get_walkman_state()
        walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        while np.abs(np.linalg.norm(walkman_P_goal[:2])) > close_threshold:
            world_State_walkman = get_walkman_state()
            walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
            walkman_P_goal = walkman_T_goal[:3, -1]
            walkman_R_goal = walkman_T_goal[:3, :3]
        rospy.sleep(5)

        # # Step 3: fin adjustment
        # world_State_walkman = get_walkman_state()
        # walkman_T_goal = np.linalg.inv(world_State_walkman['transformation_matrix']).dot(world_T_goal)
        # walkman_P_goal = walkman_T_goal[:3, -1]
        # walkman_R_goal = walkman_T_goal[:3, :3]
        # # x direction
        # print("fin adjustment...")
        # if walkman_P_goal[0]>0.0:
        #     responce_3_1 = walking_srv_client('WalkFront', quantity=np.abs(walkman_P_goal[0]))
        # else:
        #     responce_3_2 = walking_srv_client('WalkBack', quantity=np.abs(walkman_P_goal[0]))
        #
        # rospy.sleep(5)
        #
        # # y direction
        # if walkman_P_goal[1]>0.0:
        #     responce_3_3 = walking_srv_client('WalkLeft', quantity=np.abs(walkman_P_goal[1]))
        # else:
        #     responce_3_4 = walking_srv_client('WalkRight', quantity=np.abs(walkman_P_goal[1]))
        #
        # rospy.sleep(5)


        # Step 4: turn to the goal orientation
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
        while np.abs(world_State_walkman['rpy'][2] - world_yaw_goal) > np.deg2rad(turn_threshold):
            world_State_walkman = get_walkman_state()
            # print("yaw error: ", np.deg2rad(np.abs(world_State_walkman['rpy'][2] - world_yaw_goal)))
        rospy.sleep(4)


        # print the error [error_x, error_y, error_yaw]
        world_State_walkman = get_walkman_state()
        walkman_P_goal = walkman_T_goal[:3, -1]
        walkman_R_goal = walkman_T_goal[:3, :3]
        walkman_RPY_goal = trans.euler_from_matrix(walkman_R_goal)

        print("Global Error!")
        print("error_x: ", world_State_walkman['position'][0]-world_x_goal, "m")
        print("error_y: ", world_State_walkman['position'][1] - world_y_goal, "m")
        print("error_yaw: ", np.rad2deg(angles.normalize_angle(world_State_walkman['rpy'][2]-world_yaw_goal)), "deg")

    else:
        print(usage())
        sys.exit(1)