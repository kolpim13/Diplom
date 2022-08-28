from __future__ import print_function
import roslib; roslib.load_manifest('diplom')
import rospy
import sys

FUNC_RAW_DATA_SHOT = 0

def raw_data_shot():
    print("Yes")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: Minimum 1 arg: function code")
        exit()

    function = int(sys.argv[1])
    if function == FUNC_RAW_DATA_SHOT:
        raw_data_shot
    else:
        print("Unknown function")