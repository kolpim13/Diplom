#!/usr/bin/env python
from math import sqrt
import rospy
import pathlib
import json
import numpy as np

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
#=======================================================================================#

def append_to_file(file: str, pos, distances):
    with open(file, 'a') as jsonfile:
        json.dump({"pos": pos, "raw_data": distances}, jsonfile)
        jsonfile.write(str('\n'))
#=======================================================================================#

def generate_map_void(model: ModelState, pub: rospy.Publisher, model_name: str, path: str):
    with open(path, 'w+') as jsonfile:
        for i in range(10):
            model_move(model, pub, 0, 0)
            rate.sleep()
        (real_x, real_y) = model_get_pos(model_name)
    return

def generate_map(model: ModelState, pub: rospy.Publisher, model_name: str, path: str, resolution: float, x_begin: float, y_begin: float, x_end: float, y_end: float):
    # Cooficiecnts
    admissible_diff = 0.05
    
    # Create or erase(if exist) file with map
    with open(path, 'w+') as jsonfile:
        pass

    # Last feasible position
    x_feas = x_begin
    y_feas = y_begin

    # Set start position
    x = x_begin
    y = y_begin

    # Init counters
    count: int = 0
    count_x: int = 0
    count_y: int = 0
    while y <= y_end:
        count_x = 0
        count_y += 1
        rospy.logdebug("y = %f start", y)
        while x <= x_end:
            model_move(model, pub, x, y)
            rate.sleep()

            # move robot and try to find a collision
            (real_x, real_y) = model_get_pos(model_name)
            var = sqrt((real_x - x)**2 + (real_y - y)**2)
            if var < admissible_diff:
                distances = laser_topic_read()
                append_to_file(path, (x, y, 0.0), distances)
            else:
                model_move_init_pose(model, pub, x_feas, y_feas)
                append_to_file(path, (x, y, 0.0), None)
            
            count += 1
            count_x += 1
            x = np.around(x + resolution, 1)

        x = x_begin
        y = np.around(y + resolution, 1)

    return (count, count_x, count_y)

def generate_descriptor(desc: str, x_begin: float, y_begin: float, x_end: float, y_end: float,
                        lowRes: float, lowRes_x: float, lowRes_y: float, lowRes_total: int,
                        highRes: float, highRes_x: float, highRes_y: float, highRes_total: int,):
    obj = {
        "x_begin": float(x_begin),
        "x_end": float(x_end),
        "y_begin": float(y_begin),
        "y_end": float(y_end),
        "lowRes": float(lowRes),
        "lowRes_x": int(lowRes_x),
        "lowRes_y": int(lowRes_y),
        "lowRes_total": int(lowRes_total),
        "highRes": float(highRes),
        "highRes_x": int(highRes_x),
        "highRes_y": int(highRes_y),
        "highRes_total": int(highRes_total),
    }

    with open(desc, 'w+') as jsonfile:
        json.dump(obj, jsonfile)
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_creater")
    rate = rospy.Rate(10)

    # Parameters read
        # Map and model
    map_name = rospy.get_param("~map_name")
    model_name = rospy.get_param("~gazebo_model_name")
    reference_frame = rospy.get_param("~reference_frame")
        # Map region and parameters
    x_begin = rospy.get_param("~x_begin")
    y_begin = rospy.get_param("~y_begin")
    x_end = rospy.get_param("~x_end")
    y_end = rospy.get_param("~y_end")
    lowRes = rospy.get_param("~lowRes")
    highRes = rospy.get_param("~highRes")   

    # Relative paths to the maps
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "maps")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

    # Publishers and subscribers
    pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    model = create_model(pub_model, name=model_name, reference=reference_frame)

    # Void map to avoid bag with not moving models in gazego for the first few times
    generate_map_void(model, pub_model, model_name, PATH_TO_MAP_LOW_RES)

    # Generate two maps: fast and detailed
    rospy.loginfo("Low resolution map start")
    (lowRes_total, lowRes_x, lowRes_y) = generate_map(model, pub_model, model_name, PATH_TO_MAP_LOW_RES,
                                                    lowRes, x_begin, y_begin, x_end, y_end)
    rospy.loginfo("Low resolution map end")

    rospy.loginfo("High resolution map start")
    (highRes_total, highRes_x, highRes_y) = generate_map(model, pub_model, model_name, PATH_TO_MAP_HIGH_RES,
                                                    highRes, x_begin, y_begin, x_end, y_end)
    rospy.loginfo("High resolution map end")
    
    # Create descriptor for maps
    generate_descriptor(PATH_TO_MAP_DESCRIPTOR, x_begin, y_begin, x_end, y_end,
                        lowRes, lowRes_x, lowRes_y, lowRes_total,
                        highRes, highRes_x, highRes_y, highRes_total)
#=======================================================================================#
