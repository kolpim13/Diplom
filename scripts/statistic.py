# All functions were taken from localizationTest.py

#!/usr/bin/env python
from xml.dom.pulldom import SAX2DOM
from xmlrpc.client import Boolean
import rospy
import json
import pathlib
import numpy as np
from math import sqrt

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

def create_model(pub: rospy.Publisher, name: str, reference: str) -> ModelState:
    model = ModelState()

    model.model_name = name
    model.reference_frame = reference
    model_move_init_pose(model, pub, 0, 0)

    return model

def model_move_init_pose(model: ModelState, pub: rospy.Publisher, x: float, y: float):
    model.twist.linear.x = 0
    model.twist.linear.y = 0
    model.twist.linear.z = 0
    model.twist.angular.x = 0
    model.twist.angular.y = 0
    model.twist.angular.z = 0

    model.pose.orientation.w = 1
    model.pose.orientation.x = 0
    model.pose.orientation.y = 0
    model.pose.orientation.z = 0

    model.pose.position.x = x
    model.pose.position.y = y
    model.pose.position.z = 0.5
    pub.publish(model)

def model_move(model: ModelState, pub: rospy.Publisher, x: float, y: float):
    model.pose.position.x = x
    model.pose.position.y = y
    model.pose.position.z = 0.5
    pub.publish(model)

def model_get_pos(model_name: str):
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pose = gms(model_name, "")
    return (pose.pose.position.x, pose.pose.position.y)
#=======================================================================================#

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

def raw_data_preprocessing(data):
    for i in range(len(data)):
        data[i] = range_min if data[i] == float('-inf') else data[i]
        data[i] = range_max if data[i] == float('inf') else data[i]
        data[i] = range_max - 0.001 if data[i] == range_max else data[i]

    return data
#=======================================================================================#

def generate_map_void(model: ModelState, pub: rospy.Publisher, file: str, model_name: str):
    with open(file, 'w+') as f:
        pass

    for i in range(10):
        model_move(model, pub, 0, 0)
        rate.sleep()
    (real_x, real_y) = model_get_pos(model_name)

def append_to_file(file: str, point, data):
    with open(file, 'a') as jsonfile:
        json.dump({"pos": point, "data": data}, jsonfile)
        jsonfile.write(str('\n'))
#=======================================================================================#

# SRARISTIC RELATED FUNCtIONS
def generate_points(model: ModelState, pub: rospy.Publisher,
                    file: str, amount: int, 
                    x_begin: int, x_end: int, y_begin: int, y_end: int):

    points = []
    
    i = 0
    while True:
        point = generate_point(x_begin, x_end, y_begin, y_end)
        if move_and_check_if_point_feasible(model, pub, point) == False:
            model_move_init_pose(model, pub, 0, 0)
            rospy.loginfo("rejected")
            continue

        raw_data = laser_topic_read()
        data = raw_data_preprocessing(raw_data)
        append_to_file(file, point, data)

        i += 1
        rospy.loginfo("i = %d", i)
        if i >= amount:
            break

def generate_point(x_begin: int, x_end: int, y_begin: int, y_end: int) -> np.array:
    point = [0, 0, 0]

    point[0] = x_begin + np.random.random() * (x_end - x_begin) # x
    point[1] = y_begin + np.random.random() * (y_end - y_begin) # y
    point[2] = 0.0 + np.random.random() * 360.0                 # phi
    
    return point

def move_and_check_if_point_feasible(model: ModelState, pub: rospy.Publisher, point: np.array) -> Boolean:
    x = point[0]
    y = point[1]
    admissible_diff = 0.05
    
    # Move robot
    model_move(model, pub, x, y)
    rate.sleep()

    # See where it is now
    (real_x, real_y) = model_get_pos(model_name)

    # Compare these two values
    var = sqrt((real_x - x)**2 + (real_y - y)**2)
    if var > admissible_diff:
        return False
    return True
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("statistic data creation")
    rate = rospy.Rate(100)

    # Parameters read
        # Map and model
    map_name = rospy.get_param("~map_name")
    model_name = rospy.get_param("~gazebo_model_name")
    reference_frame = rospy.get_param("~reference_frame")
        # Laser parameters
    laser_count = rospy.get_param("~laser_count")
    (range_min, range_max) = laser_get_parameters()
        # Statistic parameters
    statistic_name = rospy.get_param("~statistic_name")
    points_amount = rospy.get_param("~points_amount")

    # Set relatives paths
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")
    PATH_TO_STATISTIC_FOLDER = pathlib.Path(BASE_DIR.parent, "statistic")
    PATH_TO_STATISTIC = pathlib.Path(PATH_TO_STATISTIC_FOLDER, map_name + "_" + statistic_name + ".txt")

    # Map descriptor read
    (x_begin, x_end, y_begin, y_end, 
     lowRes, lowRes_amount_x, lowRes_amount_y, lowRes_amount_total,
     highRes, highRes_amount_x, highRes_amount_y, highRes_amount_total,
     k) = read_descriptor()

    # Publishers and subscribers (from wallTest)
    pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    model = create_model(pub_model, name=model_name, reference="")

    # Generate 1000 points and write them into file
    generate_map_void(model, pub_model, PATH_TO_STATISTIC, model_name)
    generate_points(model, pub_model, PATH_TO_STATISTIC, 
                    points_amount, x_begin, x_end, y_begin, y_end)

    rospy.loginfo("The end!")
    rospy.loginfo("================================================================")
