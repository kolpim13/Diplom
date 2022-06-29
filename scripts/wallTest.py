#!/usr/bin/env python
import rospy
import sys
import pathlib
import json
import math
import numpy as np

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = [round(item, 5) for item in laser.ranges]
    return distances

def append_to_file(file_path, pos, distances):
    with open(file_path, 'a') as jsonfile:
        json.dump({"pos": pos, "raw_data": distances}, jsonfile)
        jsonfile.write(str('\n'))

def get_real_robot_pos():
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_pose = gms(gazebo_model_name, "")
    real_x = robot_pose.pose.position.x
    real_y = robot_pose.pose.position.y
    return (real_x, real_y)

if __name__ == '__main__':
    # ROS 
    rospy.init_node("wall_test")
    rate = rospy.Rate(10)

    # Parameters read
    map_name = rospy.get_param("~map_name")
    gazebo_model_name = rospy.get_param("~gazebo_model_name")
    reference_frame = rospy.get_param("~reference_frame")

# Relative path to the maps
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_TEST = pathlib.Path(PATH_TO_MAP_FOLDER, "_Test.txt")

    # Publishers and subscribers
        # Robot model
    model_state = ModelState()
    pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

    # Adjusting
        # Robot model
    model_state.model_name = gazebo_model_name
    model_state.reference_frame = reference_frame
    model_state.pose.position.z = 0.1
    model_state.pose.orientation.w = 1
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = 0
    model_state.pose.orientation.z = 0

    (real_x, real_y) = get_real_robot_pos()
    distances = laser_topic_read()
    append_to_file(PATH_TO_MAP_TEST, [round(real_x, 1), round(real_y, 1), 0.0], distances)
   
    fl: np.float32 = distances[0]
    fl = 1 if fl == float('inf') else fl
    rospy.loginfo("Inf = %.3f", fl)
    
    rospy.loginfo("The end!")
