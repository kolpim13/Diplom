#!/usr/bin/env python
import rospy
import json
import math
import time
import pathlib
import numpy as np

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

# MAIN FUNCTION
def main():
    while not rospy.is_shutdown():
        start_time = time.time()

        # Read laser raw_data and convert them into histogram
        ranges = laser_topic_read()
        ranges = raw_data_preprocessing(ranges)
        hist_pos = compress_pos(ranges)
        hist_orient = compress_orient(ranges)
        rospy.loginfo("Raw data was read and compressed into hist_posograms")

        # Find best candidate on the low resolution map
        best_index_low = find_pos_lowRes(hist_pos)
        best_low = np.array(map_loc_low[best_index_low])
        rospy.loginfo("Best candidate on the low resolution map:")
        rospy.loginfo("x_low = %.5f, y_high = %.5f", best_low[0], best_low[1])

        # Find nearest points on the high resolution map according to the best low resolution candidate
        highRes_corresponding_index = find_corresponding_point(best_index_low)
        region_to_search = find_region_to_search(highRes_corresponding_index)

        best_index_high = find_pos_highRes(hist_pos, region_to_search)
        best_high = np.array(map_loc_high[best_index_high])
        rospy.loginfo("Best candidate on the high resolution map:")
        rospy.loginfo("x_high = %.5f, y_high = %.5f", best_high[0], best_high[1])

        approx_pos = approx_position_highRes(hist_pos, best_index_high, best_index_high)
        rospy.loginfo("Approximated position: x = %.5f, y = %.5f", approx_pos[0], approx_pos[1])

        (orient_aprox_deg, orient_aprox_rad) = find_orient_highRes(hist_orient, best_index_high)
        rospy.loginfo("Approximated orientation = %.2f degree == %.2f radians", orient_aprox_deg, orient_aprox_rad)

        rospy.loginfo("This step took %.3f", (time.time() - start_time))
        rospy.loginfo("================================================================")
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

# Convert raw array of data to the view to work with
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

# AUXILIARY FUNCTIONS
def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = np.array(laser.ranges)
    return distances

def find_neighbors(index):
    top = round(index - highRes_amount_y)
    right = round(index + 1)
    bot = round(index + highRes_amount_y)
    left = round(index - 1)
    top_right = round(index - highRes_amount_y + 1)
    bot_right = round(index + highRes_amount_y + 1)
    bot_left = round(index + highRes_amount_y - 1)
    top_left = round(index - highRes_amount_y - 1)

    return (top, right, bot, left, top_right, bot_right, bot_left, top_left)
#=======================================================================================#

# COMPRESSION RAW DATA TO HISTOGRAMS
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

# MATCH TWO HISTOGRAMS
def match_pos(hist1, hist2):
    amount = laser_limits * 2

    sum = 0
    for i in range(len(hist1)):
        sum += abs(hist2[i] - hist1[i])

    error = sum / amount
    return error
#=======================================================================================#

# SOLID POINT POS AND ORIENT SEARCHING
# Returns index of the best element in the arrays
def find_pos_lowRes(hist):
    min_error: float = 100.0
    best_index: int = 0

    for i in range(lowRes_amount_total):
        error = match_pos(hist, map_pos_low[i])
        if min_error > error:
            min_error = error
            best_index = i

    return best_index

# Returns aprroximated index that corresponds on the highRes map to the lowRes point
def find_corresponding_point(lowRes_index):
    x_low = int(lowRes_index % lowRes_amount_x)
    y_low = int(lowRes_index / lowRes_amount_y)
    y_high = int(y_low * k)
    x_high = int(x_low * k)
    highRes_index = y_high * highRes_amount_y + x_high

    return highRes_index

# Give a region og points to search around approximated point
def find_region_to_search(highRes_index):
    region = np.zeros(121, dtype=np.int32)

    index = 0
    for i in range(-5, 5+1):
        for j in range(-5, 5+1):
            region[index] = np.int32(round(highRes_index + highRes_amount_y * i + j))
            index += 1

    return region

# Find the best suited point from the region on the highRes map
def find_pos_highRes(hist, region):
    min_error: float = 100.0
    best_index: int = 0

    for i in range(len(region)):
        if region[i] < 0 or region[i] >= highRes_amount_total:
            continue

        error = match_pos(hist, map_pos_high[region[i]])
        if min_error > error:
            min_error = error
            best_index = region[i]

    return best_index
#=======================================================================================#

# APPROXIMATED POINT SEARCHING
# Find the best suited orient for the chosen highRes point
def find_orient_highRes(hist, highRes_index):
    degree_per_limit = 360.0 / hist_orient_limits
    
    best_index = 0
    min_error = float('inf')
    for shift in range(hist_orient_limits):
        error = 0.0
        for j in range(hist_orient_limits):
            pos = j + shift
            pos = pos - hist_orient_limits if pos >= hist_orient_limits else pos
            error += abs(map_orient_high[highRes_index, pos] - hist[j])
        if min_error > error:
            min_error = error
            best_index = shift
    
    orient_aprox_deg = degree_per_limit * best_index
    orient_aprox_rad = orient_aprox_deg * np.pi / 180
    return (orient_aprox_deg, orient_aprox_rad)

def approx_position_highRes(hist, index, highRes_index):
    # Get all neighbors
    (top, right, bot, left, top_right, bot_right, bot_left, top_left) = find_neighbors(highRes_index)

    # See all probabilities
    best_v = 100 - (match_pos(hist, map_pos_high[index]) if top > 0 and top < highRes_amount_total else 100.0)
    top_v = 100 - (match_pos(hist, map_pos_high[top]) if top > 0 and top < highRes_amount_total else 100.0)
    right_v = 100 - (match_pos(hist, map_pos_high[right]) if right > 0 and right < highRes_amount_total else 100.0)
    bot_v = 100 - (match_pos(hist, map_pos_high[bot]) if bot > 0 and bot < highRes_amount_total else 100.0)
    left_v = 100 - (match_pos(hist, map_pos_high[left]) if left > 0 and left < highRes_amount_total else 100.0)

    # Axis x best
    if left_v > right_v:
        axis_x = left
        axis_xv = left_v
    else:
        axis_x = right
        axis_xv = right_v
    # Axis y best
    if top_v > bot_v:
        axis_y = top
        axis_yv = top_v
    else:
        axis_y = bot
        axis_yv = bot_v

    # "Deep copy" of the current postion and the nearest to it
    approx_pos = [map_loc_high[highRes_index, 0], map_loc_high[highRes_index, 1]]
    x_pos = map_loc_high[axis_x, 0]
    y_pos = map_loc_high[axis_y, 1]
    shift_x_way = approx_pos[0] - x_pos
    shift_y_way = approx_pos[1] - y_pos

    #shift_x = find_shift(best_v, axis_xv)
    #shift_y = find_shift(best_v, axis_yv)
    #approx_pos[0] += shift_x * shift_x_way
    #approx_pos[1] += shift_y * shift_y_way

    ratio_x = find_shift(best_v, axis_xv)
    ratio_y = find_shift(best_v, axis_yv)
    approx_pos[0] += ratio_x * shift_x_way
    approx_pos[1] += ratio_y * shift_y_way

    return approx_pos

# Calculate shift for aproximation position
def find_shift(d1, d2):
    mult = 0.01

    #ratio1 = d1 / (d1 + d2)
    #ratio2 = d2 / (d1 + d2)
    #ratio = ratio1 - ratio2
    #shift = (ratio * mult)
    #return shift

    ratio = d1 / (d1 + d2)
    ratio *= mult
    return ratio
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_localizator")
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

    # Set relatives path 
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

    # Read descriptor file
    (x_begin, x_end, y_begin, y_end, 
     lowRes, lowRes_amount_x, lowRes_amount_y, lowRes_amount_total,
     highRes, highRes_amount_x, highRes_amount_y, highRes_amount_total,
     k) = read_descriptor()

    # Buffer maps
    rospy.loginfo("Buffer map low res")
    (map_pos_low, map_orient_low, map_loc_low) = buffer_map(PATH_TO_MAP_LOW_RES, lowRes_amount_total)
    rospy.loginfo("Buffer map high res")
    (map_pos_high, map_orient_high, map_loc_high) = buffer_map(PATH_TO_MAP_HIGH_RES, highRes_amount_total)
    rospy.loginfo("Buffer maps from files done")
    rospy.loginfo("================================================================")

     # Publishers and subscribers
        # Robot model
    model_state = ModelState()
    pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

     # Adjusting
        # Set start robot position
    model_state.model_name = gazebo_model_name
    model_state.reference_frame = ""
    model_state.pose.position.z = 0.1
    model_state.pose.orientation.w = 1
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = 0
    model_state.pose.orientation.z = 0
    model_state.twist.linear = 0
    model_state.twist.angular = 0

    # Main loop
    main()
#=======================================================================================#
