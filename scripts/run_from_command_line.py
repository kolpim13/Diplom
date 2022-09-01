from __future__ import print_function
import roslib; roslib.load_manifest('diplom')
import rospy
import sys
import numpy as np

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan

FUNC_RAW_DATA_SHOT = 0
FUNC_ROTATE_MODEL = 1

def raw_data_shot():
    print("Yes")

def create_model(pub: rospy.Publisher, name: str, reference: str) -> ModelState:
    model = ModelState()
    model.model_name = name
    model.reference_frame = reference
    return model
def model_turn(model: ModelState, pub: rospy.Publisher, phi: float):
    z = np.sin(phi / 2)
    w = np.cos(phi / 2)
    model.pose.orientation.z = z
    model.pose.orientation.w = w
    pub.publish(model)

    # gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # pose = gms("lidar", "")
    # print(pose.pose.position.x, pose.pose.position.y,
    # pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

if __name__ == '__main__':
    rospy.init_node("command_line")

    if len(sys.argv) < 2:
        print("Usage: Minimum 1 arg: function code")
        exit()

    pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    model = create_model(pub_model, name="lidar", reference="willowgarage")

    function = int(sys.argv[1])
    if function == FUNC_RAW_DATA_SHOT:
        raw_data_shot
    elif function == FUNC_ROTATE_MODEL:
        phi = float(sys.argv[2])
        print(phi)
        model_turn(model, pub_model, phi)
    else:
        print("Unknown function")