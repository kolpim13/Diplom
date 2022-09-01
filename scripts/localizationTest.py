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
#=======================================================================================#

def create_model(pub: rospy.Publisher, name: str, reference: str) -> ModelState:
    model = ModelState()

    model.model_name = name
    model.reference_frame = reference

    return model

laser_topic_name = "/laser/scan"

def laser_topic_read():
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = [round(item, 5) for item in laser.ranges]

    return distances
    
def laser_get_parameters():
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    return (laser.range_min, laser.range_max)
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
Compress raw data from file and write it into the RAM as map [4 different arrays]. 
Every map contains of 4 elements: real localization, position and orient histograms, feasibility of the point
map - path to file with map
records - total amount of lines/points/positions in this map-file
range_min_max - minimal and maximum laser distance
hist_limit_pos_orient - amount of sectors for every type of histogram

return (localization:[][], position:[][], orientation:[][], feasibility:bool
"""
def buffer_map(map: str, records: int, 
            range_min: np.float32, range_max: np.float32,
            hist_limit_pos: int, hist_limit_orient: int) -> tuple:
    
    # Arrays that are represent map
    map_pos = np.zeros((records, hist_limit_pos), dtype=np.int32)
    map_orient = np.zeros((records, hist_limit_orient), dtype=np.float32)
    map_loc = np.zeros((records, 3), dtype=np.float32)
    map_feas = np.zeros((records), dtype=Boolean)
    
    i = 0
    j = 0
    with open(map, 'r') as jsonfile:
        for index in range(records):
            # Read data from json
            line = jsonfile.readline()
            json_line = json.loads(line)
            raw_data = json_line["raw_data"]

            # Fill arrays
                # With zero data if raw data is None
            if raw_data == None:
                map_pos[index, :] = np.zeros((hist_limit_pos))
                map_orient[index, :] = np.zeros((hist_limit_orient))
                map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)
                map_feas[i] = False
                # With histograms if something is present
            else:
                data = raw_data_preprocessing(raw_data, range_min, range_max)
                map_pos[index, :] = compress_pos(data, range_max, hist_limit_pos)
                map_orient[index, :] = compress_orient(data, laser_count, hist_limit_orient)
                map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)
                map_feas[i] = True

            # For debugging purpose
            i += 1
            if i == 1000:
                i = 0
                j += 1
                rospy.loginfo("Mapping %.2f procent", (j * 1000 / records))

    return (map_loc, map_pos, map_orient, map_feas)

"""
Modify raw laser scans for further working process
data:[] - one dimensional array with raw laser data

return data prepeared for the compression into histograms
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

# HISTOGRAMS
"""
Compress prepeared raw laser data into positional histogram

return position histogram as numpy 1D array
"""
def compress_pos(data, range_max: np.float32, hist_limit: int):
    hist = np.zeros(hist_limit)
    range = range_max / hist_limit

    for item in data:
        lim = math.floor(item / range)
        hist[lim] += 1 

    return hist

"""
Compress prepeared raw laser data into priental histogram

return orient histogram as numpy 1D array
"""
def compress_orient(data, laser_count: int, hist_limit: int):
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

"""
hist1, hist2 - positional histograms to be compared. Given as numpy 1D array
laser_count - amount of lasers

return error(read differents) in two histograms
"""
def match_pos(hist1, hist2, laser_count: int):
    amount = laser_count * 2

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
map_feas - 1D boolean array with info if particular point exist 
records - amount of histograms in the array

return index of the best candidate on this map
"""
def find_pos(hist, map_pos, map_feas, records, laser_count: int) -> int:
    min_error: float = 100.0
    index: int = 0

    for i in range(records):
        if map_feas[i] == False:
            continue

        error = match_pos(hist, map_pos[i], laser_count)
        if min_error > error:
            min_error = error
            index = i

    return index

"""
Function search for the best candidate on specific region.
hist - histogram I want to compare 
map_pos - 2d numpy array where all position histograms are listed 
map_feas - 1D boolean array with info if particular point exist
records - amount of histograms in the array
region - 1D array of indecses that shows specific map positions I need to look on.

return index of the best candidate on the given map region
"""
def find_pos_on_region(hist, map_pos, map_feas, records: int, region, laser_count: int) -> int:
    min_error: float = 100.0
    best_index: int = 0
    
    for i in range(len(region)):
        if map_feas[i] == False:
            continue
        if region[i] < 0 or region[i] >= records:
            continue

        error = match_pos(hist, map_pos[region[i]], laser_count)
        if min_error > error:
            min_error = error
            best_index = region[i]

    return best_index

"""
Searching for position on a map with higher resolution that is corresponds with given indeks of a map with the smaller resolution
mapn_x/y - maximum records of one axis.

return map1_index of the corresponding position
"""
def find_corresponding_point(map1_index: int, k: float, map1_x: int, map1_y: int, map2_x: int, map2_y: int) -> int:
    x_1 = int(map1_index % map1_x)
    y_1 = int(map1_index / map1_y)
    x_2 = int(x_1 * k)
    y_2 = int(y_1 * k)
    map2_index = y_2 * map2_y + x_2

    return map2_index

"""
Find all points that are around given indexed point inside some radius
map_index - index 
radius - half side of a square

return 1D np array with indexes
"""
def find_region_to_search(map_index: int, radius: int, max_x: int, map_y: int):
    min = -radius
    max = radius + 1
    total = (max - min)**2

    region = np.zeros(total, dtype=np.int32)

    index = 0
    for i in range(min, max):
        for j in range(min, max):
            region[index] = np.int32(round(map_index + map_y * i + j))
            index += 1

    return region

"""
Find the best shifted orient position
hist: orient histogram of the current position we want apply to our map
map_orient: 2D np. array with compressed histograms
map_orient_limit: [int] Represents how many bars do havem orientation histogram 
map_orient_index: [int] Index on the orient map we want to compare to

return angle in degrees and radians [np.float]
"""
def find_orient(hist, map_orient, map_orient_limit: int, map_orient_index: int):
    degree_per_limit = 360.0 / map_orient_limit

    best_index = 0
    min_error = float('inf')
    for shift in range(map_orient_limit):
        error = 0.0
        for j in range(map_orient_limit):
            pos = j + shift
            pos = pos - map_orient_limit if pos >= map_orient_limit else pos
            error += abs(map_orient[map_orient_index, pos] - hist[j])
        if error < min_error:
            min_error = error
            best_index = shift

        orient_deg: np.float32 = degree_per_limit * best_index
        orient_rad: np.float32 = orient_deg * np.pi / 180

    return (orient_deg, orient_rad)
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

    hist_limit_pos = rospy.get_param("~hist_pos_limit")
    hist_limit_orient = rospy.get_param("~hist_orient_limit")

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

    # Buffer maps [Make maps from raw data in the file]
    rospy.loginfo("Buffer map low res")
    (map_loc_low, map_pos_low, map_orient_low, map_feas_low) = buffer_map(PATH_TO_MAP_LOW_RES, lowRes_amount_total, 
                                                            range_min, range_max, hist_limit_pos, hist_limit_orient)
    rospy.loginfo("Buffer map high res")
    (map_loc_high, map_pos_high, map_orient_high, map_feas_high) = buffer_map(PATH_TO_MAP_HIGH_RES, highRes_amount_total, 
                                                            range_min, range_max, hist_limit_pos, hist_limit_orient)
    rospy.loginfo("Buffer maps from files done")
    rospy.loginfo("================================================================")

    # Publishers and subscribers (from wallTest)
    pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    model = create_model(pub_model, name=model_name, reference="")

    # Main loop
    while not rospy.is_shutdown():
        start_time = time.time()

        # Read and process raw laser data
        rospy.loginfo("Read raw data: ...")
        raw_data = laser_topic_read() #probably
        data = raw_data_preprocessing(raw_data, range_min, range_max)
        hist_pos_current = compress_pos(data, range_max, hist_limit_pos)
        hist_orient_current = compress_orient(data, laser_count, hist_limit_orient)
        rospy.loginfo("Raw data was read and compressed into histograms")

        # Find best candidate on the low resolution map
        best_index_low = find_pos(hist_pos_current, map_pos_low, map_feas_low, lowRes_amount_total, laser_count)
        best_low = map_loc_low[best_index_low]
        
        # Find position on the high resolution map that is corelate to low res map 
        highRes_corresponding_index = find_corresponding_point(best_index_low, k, 
                                                                lowRes_amount_x, lowRes_amount_y,
                                                                highRes_amount_x, highRes_amount_y)
        highRes_region_to_search = find_region_to_search(highRes_corresponding_index, 5,
                                                        highRes_amount_x, highRes_amount_y)

        best_index_high = find_pos_on_region(hist_pos_current, map_pos_high, map_feas_high,
                                            highRes_amount_total, highRes_region_to_search, laser_count)
        best_high = map_loc_high[best_index_high]

        # Approximated position


        # Orientation calculating    
        (orient_deg, orient_rad) = find_orient(hist_orient_current, map_orient_high, hist_limit_orient, best_index_high)

        # Print all found data for debugging purposes
        rospy.loginfo("Best candidate on low resolution map:")
        rospy.loginfo("x_low = %.5f, y_low = %.5f", best_low[0], best_low[1])
        rospy.loginfo("Best candidate on high resolution map:")
        rospy.loginfo("x_high = %.5f, y_high = %.5f", best_high[0], best_high[1])
        rospy.loginfo("Orientation is equal")
        rospy.loginfo("Degrees = %.5f, Radians = %.5f", orient_deg, orient_rad)
    
        rospy.loginfo("This step took %.3f", (time.time() - start_time))
        rospy.loginfo("================================================================")
        rate.sleep()
#=======================================================================================#
