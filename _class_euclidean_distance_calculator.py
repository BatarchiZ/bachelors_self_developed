#!/usr/bin/python3

import rospy
import tf
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from __class_gym_continuous import get_object_position

OBJECT_SIZE_X = 0.02  
OBJECT_SIZE_Y = 0.02  
OBJECT_SIZE_Z = 0.2   

class GripperDistanceService:
    def __init__(self):
        rospy.init_node("gripper_distance_service", anonymous=True)
        self.listener = tf.TransformListener()
        rospy.sleep(1)
        self.service = rospy.Service("/get_gripper_distance", Trigger, self.compute_distance_callback)

    def get_tf_position(self, parent_frame, child_frame):
        try:
            (trans, rot) = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            return np.array(trans)  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def compute_midpoint(self, base_position):
        if base_position is None:
            return None
        
        midpoint = np.copy(base_position)
        midpoint[2] += OBJECT_SIZE_Z / 2 
        return midpoint

    def compute_distance_callback(self, request):
        gripper_midpoint = self.get_tf_position("world", "grippers_midpoint")
        object_pick_base = get_object_position()

        if gripper_midpoint is None or object_pick_base is None:
            rospy.logwarn("Failed to get positions.")
            return TriggerResponse(success=False, message="Failed to retrieve positions.")

        object_pick_midpoint = self.compute_midpoint(object_pick_base)
        object_pick_midpoint[2] = 0.7
        distance = np.linalg.norm(gripper_midpoint - object_pick_midpoint)

        return TriggerResponse(success=True, message=f"{distance:.4f}")

if __name__ == "__main__":
    GripperDistanceService()
    rospy.spin()  
