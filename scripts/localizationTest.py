#!/usr/bin/env python
from xml.dom.pulldom import SAX2DOM
from xmlrpc.client import Boolean
import rospy
import json
import math
import time
import pathlib
import numpy as np

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan

import wallTest
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
        lowRes_x = int(data['lowRes_x'])
        lowRes_y = int(data['lowRes_y'])
        lowRes_total = int(data['lowRes_total'])
        highRes = float(data['highRes'])
        highRes_x = int(data['highRes_x'])
        highRes_y = int(data['highRes_y'])
        highRes_total = int(data['highRes_total'])

    k = lowRes / highRes

    return (x_begin, x_end, y_begin, y_end, 
            lowRes, lowRes_x, lowRes_y, lowRes_total,
            highRes, highRes_x, highRes_y, highRes_total,
            k)

""" 
Compress raw data from file and write it into the RAM as map. 
Every map contains of 3 elements: real localization, position and orient histograms
map - path to file with map
lines - total amiunt of lines in this file 
"""
def buffer_map(map: str, lines: int, 
                laser_count: int, range_min: np.float32, range_max: np.float32,
                hist_limit_pos: int, hist_limit_orient: int):

    # 2D arrays with position, orientation histograms and real positions
    map_pos = np.zeros((lines, hist_limit_pos), dtype=np.int32)
    map_orient = np.zeros((lines, hist_limit_orient), dtype=np.float32)
    map_loc = np.zeros((lines, 3), dtype=np.float32)
    map_existence = np.zeros((lines), dtype=Boolean)

    i = 0
    j = 0
    with open(map, 'r') as jsonfile:
        for index in range(int(lines)):
            line = jsonfile.readline()
            json_line = json.loads(line)
            raw_data = json_line["raw_data"]

            if raw_data == None:
                map_pos[index, :] = np.zeros((60))
                map_orient[index, :] = np.zeros((60))
                map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)
                map_existence[index] = False
            else:
                data = raw_data_preprocessing(raw_data, range_min, range_max)
                map_pos[index, :] = compress_pos(data, hist_limit_pos, range_max)
                map_orient[index, :] = compress_orient(data, hist_limit_orient, laser_count)
                map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)
                map_existence[index] = True

            i += 1
            if i == 1000:
                i = 0
                j += 1
                rospy.loginfo("Mapping %.2f procent", (j * 1000 / lines))

    return (map_loc, map_pos, map_orient, map_existence)

"""
Modify raw laser scans for further working process 
"""
def raw_data_preprocessing(data, range_min: np.float32, range_max: np.float32):
    for i in range(len(data)):
        if data[i] == None:
            continue

        data[i] = range_min if data[i] == float('-inf') else data[i]
        data[i] = range_max if data[i] == float('inf') else data[i]
        data[i] = range_max - 0.001 if data[i] == range_max else data[i]

    return data
#=======================================================================================#
def laser_get_parameters():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    return (laser.range_min, laser.range_max)
#=======================================================================================#

# HISTOGRAMS
"""
data - prepeared(withount inf and -inf) raw laser data
range_max - laser maximum range
hist_limit - amount of bars in histogram
laser_count - amount of laser scans
"""
def compress_pos(data, hist_limit: int, range_max: np.float32):
    range = range_max / hist_limit
    hist = np.zeros(hist_limit)

    for item in data:
        lim = math.floor(item / range)
        hist[lim] += 1 

    return hist

def compress_orient(data, hist_limit: int, laser_count: int):
    hist = np.zeros(hist_limit)
    ranges_in_sector = int(laser_count / hist_limit)
    
    for i in range(hist_limit):
        index = int(i * ranges_in_sector)
        aver = 0.0
        for j in range(ranges_in_sector):
            aver += data[index + j]
        aver /= ranges_in_sector
        hist[i] = aver

    return hist

def match_pos(hist1, hist2, laser_limit):
    amount = laser_limit * 2

    sum = 0
    for i in range(len(hist1)):
        sum += abs(hist2[i] - hist1[i])

    error = sum / amount
    return error
#=======================================================================================#

"""
Function for linear searching through all histograms.
hist - histogram I want to compare 
map_pos - 2d numpy array where all position histograms are listed  
count - amount of histograms in the array
"""
def find_pos(hist, map_pos, laser_limit: int):
    min_error: float = 100.0
    best_index: int = 0

    for i in range(len(map_pos)):
        if map_pos[i] == None:
            continue

        error = match_pos(hist, map_pos[i], laser_limit)
        if min_error > error:
            min_error = error
            best_index = i

    return best_index

def find_pos_on_region(hist, map_pos, region, amount, laser_limit: int):
    min_error: float = 100.0
    best_index: int = 0

    for i in range(len(region)):
        if region[i] < 0 or region[i] >= amount:
            continue

        if map_pos[region[i]] == None:
            continue

        error = match_pos(hist, map_pos[region[i]], laser_limit)
        if min_error > error:
            min_error = error
            best_index = region[i]

    return best_index

"""
Function for calculation index of the candidate on one map with given index from smaller map

return index on the more precise map
"""
def find_corresponding_pos(lowRes_index: int, lowRes_amount_x: int, lowRes_amount_y: int,
                            highRes_amount_x: int, highRes_amount_y: int,
                            k: float) -> int:

    x_low = int(lowRes_index % lowRes_amount_x)
    y_low = int(lowRes_index / lowRes_amount_y)
    y_high = int(y_low * k)
    x_high = int(x_low * k)
    highRes_index = y_high * highRes_amount_y + x_high

    return highRes_index

def find_region_to_search(index: int, k):
    region = np.zeros(121, dtype=np.int32)

    index = 0
    for i in range(-5, 5+1):
        for j in range(-5, 5+1):
            region[index] = np.int32(round(index + highRes_amount_y * i + j))
            index += 1

    return region
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_localizator")
    rate = rospy.Rate(1)

    laser_get_parameters()

    # Ros parameters read
    rospy.loginfo("Read all parameters")
    map_name = rospy.get_param("~map_name")
    model_name = rospy.get_param("~gazebo_model_name")

    hist_pos_limit = rospy.get_param("~hist_pos_limit")
    hist_orient_limit = rospy.get_param("~hist_pos_limit")

    laser_count = rospy.get_param("~laser_count")

    # Read laser parameters
    (range_min, range_max) = laser_get_parameters()

    # Set relatives paths
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

    (x_begin, x_end, y_begin, y_end, 
     lowRes, lowRes_amount_x, lowRes_amount_y, lowRes_amount_total,
     highRes, highRes_amount_x, highRes_amount_y, highRes_amount_total,
     k) = read_descriptor()

    # Buffer maps
    rospy.loginfo("Buffer map low res")
    (map_loc_low, map_pos_low, map_orient_low) = buffer_map(PATH_TO_MAP_LOW_RES, lowRes_amount_total, 
                                                            laser_count, range_min, range_max, 
                                                            hist_pos_limit, hist_orient_limit)
    rospy.loginfo("Buffer map high res")
    (map_loc_high, map_pos_high, map_orient_high) = buffer_map(PATH_TO_MAP_HIGH_RES, highRes_amount_total, 
                                                            laser_count, range_min, range_max, 
                                                            hist_pos_limit, hist_orient_limit)
    rospy.loginfo("Buffer maps from files done")
    rospy.loginfo("================================================================")

    # Publishers and subscribers (from wallTest)
    pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    model = wallTest.create_model(pub_model, name=model_name, reference="")

    # Set robot start position (from wallTest)
    wallTest.model_move_init_pose(model, pub_model, 0, 0)

    # Main loop
    while not rospy.is_shutdown():
        start_time = time.time()

        ranges = wallTest.laser_topic_read()
        ranges = raw_data_preprocessing(ranges, range_min, range_max)
        hist_pos = compress_pos(ranges, hist_pos_limit, range_max)
        hist_orient = compress_orient(ranges, hist_orient_limit, laser_count)
        rospy.loginfo("Raw data was read and compressed into hist_posograms")

        # Find best candidate on the low resolution map
        best_index_low = find_pos(hist_pos, map_pos_low, laser_count)
        best_low = np.array(map_loc_low[best_index_low])
        rospy.loginfo("Best candidate on the low resolution map:")
        rospy.loginfo("x_low = %.5f, y_high = %.5f", best_low[0], best_low[1])

        # Find nearest points on the high resolution map according to the best low resolution candidate
        highRes_corresponding_index = find_corresponding_pos(best_index_low)
        region_to_search = find_region_to_search(highRes_corresponding_index, k)
        best_index_high = find_pos_on_region(hist_pos, map_pos_high, region_to_search, highRes_amount_total, laser_count)
        best_high = np.array(map_loc_high[best_index_high])
        rospy.loginfo("Best candidate on the high resolution map:")
        rospy.loginfo("x_high = %.5f, y_high = %.5f", best_high[0], best_high[1])



        rospy.loginfo("This step took %.3f", (time.time() - start_time))
        rospy.loginfo("================================================================")
        rate.sleep()
#=======================================================================================#
