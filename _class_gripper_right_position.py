#!/usr/bin/python3

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Trigger, TriggerResponse

class GripperRightTracker:
    def __init__(self):
        rospy.init_node("gripper_right_tracker", anonymous=True)

        self.model_states = None
        self.last_update_time = rospy.Time.now()

        self.subscriber = rospy.Subscriber("/cobot/gripper_controller/state", JointTrajectoryControllerState, self.gripper_state_callback)

        self.service = rospy.Service("/get_gripper_right_error", Trigger, self.get_position_callback)

        rospy.sleep(1)  

    def gripper_state_callback(self, data):
        self.last_update_time = rospy.Time.now()
        self.model_states = data

    def get_position_callback(self, request):
        gripper_name = "gripper_right_joint"

        if self.model_states is None:
            rospy.logwarn("No gripper state messages received yet.")
            return TriggerResponse(success=False, message="No gripper state received.")

        try:
            index = self.model_states.joint_names.index(gripper_name)
            shift = self.model_states.error.positions[index]
            return TriggerResponse(success=True, message=f"{shift:.6f}")

        except ValueError:
            return TriggerResponse(success=False, message=f"Gripper {gripper_name} not found.")

if __name__ == "__main__":
    tracker = GripperRightTracker()
    rospy.spin()  



