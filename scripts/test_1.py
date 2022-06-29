#!/usr/bin/env python
import rospy
import sys
import json
import math
import time
import pathlib
import numpy as np
import random

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

# AUXILIARY FUNC
def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = np.array(laser.ranges)
    return distances

def model_move_on(x: float, y: float):
    # Position
    model_state.pose.position.x = x
    model_state.pose.position.y = y

    # Publish
    pub_model_state.publish(model_state)

def compress_pos(data):
    range = range_max / hist_pos_limits
    hist = np.zeros(hist_pos_limits)

    for item in data:
        lim = math.floor(item / range)
        hist[lim] += 1 

    return hist

def compress_orient(data):
    hist = np.zeros(hist_orient_limits)
    ranges_in_sector = int(laser_limits / hist_orient_limits)
    
    for i in range(hist_orient_limits):
        index = int(i * ranges_in_sector)
        aver = 0.0
        for j in range(ranges_in_sector):
            aver += data[index + j]
        aver /= ranges_in_sector
        hist[i] = aver

    return hist

def match_pos(hist1, hist2):
    amount = laser_limits * 2

    sum = 0
    for i in range(len(hist1)):
        sum += abs(hist2[i] - hist1[i])

    error = sum / amount
    return error

# Search for needed position strict
def find_pos(hist, map, loc):
    min_error: float = 100.0
    best_index: int = 0

    for i in range(len(map)):
        error = match_pos(hist, map[i])
        if min_error > error:
            min_error = error
            best_index = i

    pos = loc[best_index]
    return pos

def do_void_moves():
    for i in range(10):
        model_move_on(0, 0)
        rate.sleep()
#=======================================================================================#

# PRE PROCESSING
def read_descriptor():
    with open(PATH_TO_MAP_DESCRIPTOR, 'r') as jsonfile:
        data = json.load(jsonfile)
        x_begin = float(data['x_begin'])
        x_end = float(data['x_end'])
        y_begin = float(data['y_begin'])
        y_end = float(data['y_end'])
        lowRes = float(data['lowRes'])
        lowRes_amount_x = int(data['lowRes_amount_x'])
        lowRes_amount_y = int(data['lowRes_amount_y'])
        lowRes_amount_total = int(data['lowRes_amount_total'])
        highRes = float(data['highRes'])
        highRes_amount_x = int(data['highRes_amount_x'])
        highRes_amount_y = int(data['highRes_amount_y'])
        highRes_amount_total = int(data['highRes_amount_total'])

    k = lowRes / highRes

    return (x_begin, x_end, y_begin, y_end, 
            lowRes, lowRes_amount_x, lowRes_amount_y, lowRes_amount_total,
            highRes, highRes_amount_x, highRes_amount_y, highRes_amount_total,
            k)

def raw_data_preprocessing(data):
    for i in range(len(data)):
        data[i] = range_min if data[i] == float('-inf') else data[i]
        data[i] = range_max if data[i] == float('inf') else data[i]
        data[i] = range_max - 0.001 if data[i] == range_max else data[i]

    return data

def buffer_map(path_to_map, total_lines):
    # 2D arrays with position, orientation histograms and real positions
    map_pos = np.zeros((total_lines, hist_pos_limits), dtype=np.int32)
    map_orient = np.zeros((total_lines, hist_orient_limits), dtype=np.float32)
    map_loc = np.zeros((total_lines, 3), dtype=np.float32)

    i = 0
    j = 0
    with open(path_to_map, 'r') as jsonfile:
        for index in range(int(total_lines)):
            line = jsonfile.readline()
            json_line = json.loads(line)
            raw_data = json_line["raw_data"]
            data = raw_data_preprocessing(raw_data)
            map_pos[index, :] = compress_pos(data)
            map_orient[index, :] = compress_orient(data)
            map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)

            i += 1
            if i == 1000:
                i = 0
                j += 1
                rospy.loginfo("Mapping %.2f procent", (j * 1000 / total_lines))

    return (map_pos, map_orient, map_loc)
#=======================================================================================#

# POST PROCESSING
def eliminate_values_out_of_range(data, res):
    max = np.sqrt(2) * res
    for i in range(len(data)):
        if data[i] > max:
            data[i] = np.random.random() * max
    
    return data
#=======================================================================================#

# STATISTIC TEST
# Generate random points on the map
def generate_points(size):
    points = np.zeros((size, 3), dtype=np.float32)

    for i in range(size):
        points[i, 0] = x_begin + np.random.random() * (x_end - x_begin) # x
        points[i, 1] = y_begin + np.random.random() * (y_end - y_begin) # y
        points[i, 2] = 0.0 + np.random.random() * 360.0                 # phi

    return points

# Move robot on those points and do histograms on them
def get_histograms(points):
    hist_pos = np.zeros((len(points), hist_pos_limits))
    hist_orient = np.zeros((len(points), hist_orient_limits))

    # Go threough all points
    for index in range(len(points)):
        x = points[index][0]
        y = points[index][1]

        # Move on needed point and read raw data
        model_move_on(x, y)
        # rate.sleep() # Is needed ?
        ranges = laser_topic_read()
        ranges = raw_data_preprocessing(ranges)

        # Compress data into position and orientation histograms
        hist_pos[index, :] = compress_pos(ranges)
        hist_orient[index, :] = compress_orient(ranges)

    return (hist_pos, hist_orient)

# Find errorcompared histograms to the maps
def find_errors(points, map, loc, histograms):
    errors = np.zeros(len(histograms))

    for i in range(len(histograms)):
        point = points[i]

        pos = find_pos(histograms[i], map, loc)
        error = np.sqrt((point[0] - pos[0])**2 + (point[1] - pos[1])**2)
        errors[i] = error

    return errors

# Find min, max errors, mean value and variance
def find_min_and_max_error(errors):
    min_e = min(errors)
    max_e = max(errors)
    return (min_e, max_e)

def find_mean_error(errors):
    mean = np.mean(errors)
    return mean

def find_variance_error(errors):
    var = np.var(errors)
    return var
#=======================================================================================#

# MAIN FUNCTION
def main():
    # To avoid situation where robot stay on the same place
    do_void_moves()

    # Generate given amount of points and get histograms from those places
    points = generate_points(sample_size)
    (hist_pos, hist_orient) = get_histograms(points)
    rospy.loginfo("Points generated and histograms created")
    rospy.loginfo("================================================================")

    # Low resolution map pos block
    rospy.loginfo("Low resolution map errors calculation")
    low_errors = find_errors(points, map_pos_low, map_loc_low, hist_pos)
    low_errors = eliminate_values_out_of_range(low_errors, lowRes)
    (low_min_e, low_max_e) = find_min_and_max_error(low_errors)
    low_mean = find_mean_error(low_errors)
    low_var = find_variance_error(low_errors)
    rospy.loginfo("================================================================")

    # High resolution map pos block
    rospy.loginfo("High resolution map errors calculation")
    high_errors = find_errors(points, map_pos_high, map_loc_high, hist_pos)
    high_errors = eliminate_values_out_of_range(high_errors, highRes)
    (high_min_e, high_max_e) = find_min_and_max_error(high_errors)
    high_mean = find_mean_error(high_errors)
    high_var = find_variance_error(high_errors)
    rospy.loginfo("================================================================")

    # Form json text and write it into file
    rospy.loginfo("Writing into json file")
    obj = {
        "sample_size": sample_size, 
        "low_errors": list(low_errors),
        "low_min_e": low_min_e, 
        "low_max_e": low_max_e, 
        "low_mean": low_mean, 
        "low_var": low_var,
        "high_errors": list(high_errors),
        "high_min_e": high_min_e, 
        "high_max_e": high_max_e, 
        "high_mean": high_mean, 
        "high_var": high_var,
        }

    with open(PATH_TO_STATISTIC, 'w+') as jsonfile:
        json.dump(obj, jsonfile, indent=4)
    rospy.loginfo("The end!")
    rospy.loginfo("================================================================")
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("statistic_test_1")
    rate = rospy.Rate(1)

    # Ros parameters read
    rospy.loginfo("Read all parameters")
    map_name = rospy.get_param("~map_name")
    gazebo_model_name = rospy.get_param("~gazebo_model_name")

    hist_pos_limits = rospy.get_param("~hist_pos_limits")
    hist_orient_limits = rospy.get_param("~hist_orient_limits")

    range_min = rospy.get_param("~laser_min")
    range_max = rospy.get_param("~laser_max")
    laser_limits = rospy.get_param("~laser_amount")

    stat_name = rospy.get_param("~stat_name")
    sample_size = rospy.get_param("~sample_size")

    # Set relatives path 
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")
    PATH_TO_STATISTIC_FOLDER = pathlib.Path(BASE_DIR.parent, "statistic")
    PATH_TO_STATISTIC = pathlib.Path(PATH_TO_STATISTIC_FOLDER, map_name + "_" + stat_name + ".txt")

    #  Read map descriptor
    (x_begin, x_end, y_begin, y_end, 
     lowRes, lowRes_amount_x, lowRes_amount_y, lowRes_amount_total,
     highRes, highRes_amount_x, highRes_amount_y, highRes_amount_total,
     k) = read_descriptor()

    # Buffer maps
    (map_pos_low, map_orient_low, map_loc_low) = buffer_map(PATH_TO_MAP_LOW_RES, lowRes_amount_total)
    (map_pos_high, map_orient_high, map_loc_high) = buffer_map(PATH_TO_MAP_HIGH_RES, highRes_amount_total)

    # Publishers and subscribers
        # Robot model
    model_state = ModelState()
    pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

     # Adjusting
        # Robot model
    model_state.model_name = gazebo_model_name
    model_state.reference_frame = ""
    model_state.pose.position.z = 0.1
    model_state.pose.orientation.w = 1
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = 0
    model_state.pose.orientation.z = 0

    # Main part
    main()
#=======================================================================================#
