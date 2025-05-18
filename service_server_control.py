#!/usr/bin/python3
import time
import rospy
import subprocess
import rospy
import time
from std_srvs.srv import Trigger

def launch_gripper_tracker():
    subprocess.Popen(["/usr/bin/python3", "_class_gripper_right_position.py"])
    time.sleep(3) 

def get_gripper_disposition():
    rospy.wait_for_service("/get_gripper_right_error")
    try:
        error = rospy.ServiceProxy("/get_gripper_right_error", Trigger)
        response = error()

        if response.success:
            err = float(response.message)
            return err
        else:
            return None
    except rospy.ServiceException as e:
        return None

def launch_tracker():
    subprocess.Popen(["/usr/bin/python3", "_class_object_tracker_copy.py"])
    time.sleep(3) 

def get_object_position():
    rospy.wait_for_service("/get_object_position")
    try:
        get_position = rospy.ServiceProxy("/get_object_position", Trigger)
        response = get_position()
        if response.success:
            x1, y1, z1 = map(float, response.message.split(', '))
            return x1, y1, z1
        else:
            return None, None, None
    except rospy.ServiceException as e:
        return None, None, None

def launch_euclid_distance():
    subprocess.Popen(["/usr/bin/python3", "_class_euclidean_distance_calculator.py"])
    time.sleep(3) 


def get_euclidean_distance():
    rospy.wait_for_service("/get_gripper_distance")
    try:
        get_distance = rospy.ServiceProxy("/get_gripper_distance", Trigger)
        response = get_distance()
        if response.success:
            return float(response.message)
        else:
            return None
    except rospy.ServiceException as e:
        return None