#!/usr/bin/env python
import rospy
import sys
import pathlib
import json
import math

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = [round(item, 5) for item in laser.ranges]
    return distances

def model_move_on(x: float, y: float):
    global pub_model_state, model_state

    # Position
    model_state.pose.position.x = x
    model_state.pose.position.y = y

    # Publish
    pub_model_state.publish(model_state)

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

def generate_descriptor(path_to_desc):
    obj = {
        "x_begin": float(x_begin),
        "x_end": float(x_end),
        "y_begin": float(y_begin),
        "y_end": float(y_end),
        "lowRes": float(lowRes),
        "lowRes_amount_x": int(lowRes_amount_x),
        "lowRes_amount_y": int(lowRes_amount_y),
        "lowRes_amount_total": int(lowRes_amount_total),
        "highRes": float(highRes),
        "highRes_amount_x": int(highRes_amount_x),
        "highRes_amount_y": int(highRes_amount_y),
        "highRes_amount_total": int(highRes_amount_total),
    }

    with open(path_to_desc, 'w+') as jsonfile:
        json.dump(obj, jsonfile)

def generate_map(path_to_map, resolution) -> tuple:
    # Open file
    with open(path_to_map, 'w+') as jsonfile:
        pass

    # Set start position
    x = x_begin
    y = y_begin

    # Go through all cells
    count = 0
    count_x = 0
    count_y = 0
    while y <= y_end:
        count_x = 0
        count_y += 1
        while x <= x_end:
            model_move_on(x, y)
            rate.sleep()

            (real_x, real_y) = get_real_robot_pos()
            distances = laser_topic_read()
            append_to_file(path_to_map, (round(real_x, 1), round(real_y, 1), 0.0), distances)
            x += resolution
            count_x += 1
            count += 1

        rospy.loginfo("y = %f done", y)
        x = x_begin
        y += resolution

    # Calculate amount cells in x, y axis and in total on the field
    amount_x = count_x
    amount_y = count_y
    amount_total = count
    return (amount_x, amount_y, amount_total)

def generate_map_void():
    global PATH_TO_MAP_LOW_RES

    with open(PATH_TO_MAP_LOW_RES, 'w+') as jsonfile:
        for i in range(10):
            model_move_on(0, 0)
            rate.sleep()
            (real_x, real_y) = get_real_robot_pos()
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_creater")
    rate = rospy.Rate(10)

    # Parameters read
    map_name = rospy.get_param("~map_name")
    gazebo_model_name = rospy.get_param("~gazebo_model_name")
    reference_frame = rospy.get_param("~reference_frame")

    x_begin = rospy.get_param("~x_begin")
    y_begin = rospy.get_param("~y_begin")
    x_end = rospy.get_param("~x_end")
    y_end = rospy.get_param("~y_end")

    laser_range_min = rospy.get_param("~laser_min")
    laser_range_max = rospy.get_param("~laser_max")

    lowRes = rospy.get_param("~lowRes")
    highRes = rospy.get_param("~highRes")

    # Relative path to the maps
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

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

    # Generate two maps
    generate_map_void()
    rospy.loginfo("Low resolution map:")
    (lowRes_amount_x, lowRes_amount_y, lowRes_amount_total) = generate_map(PATH_TO_MAP_LOW_RES, lowRes)
    rospy.loginfo("High resolution map:")
    (highRes_amount_x, highRes_amount_y, highRes_amount_total) = generate_map(PATH_TO_MAP_HIGH_RES, highRes)

    # Create desriptor for maps
    generate_descriptor(PATH_TO_MAP_DESCRIPTOR)
#=======================================================================================#