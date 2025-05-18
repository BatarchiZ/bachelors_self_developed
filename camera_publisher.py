#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUTPUT_DIR = "/home/is/catkin_ws/src/z_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

class CameraSubscriber:
    def __init__(self):
        rospy.init_node("camera_subscriber", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.callback)
        self.image_counter = 0

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            image_path = os.path.join(OUTPUT_DIR, f"recent_frame.jpg")
            cv2.imshow("frame", cv_image)
            cv2.waitKey(1)
            cv2.imwrite(image_path, cv_image)
        except Exception as e:
            pass
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    pass