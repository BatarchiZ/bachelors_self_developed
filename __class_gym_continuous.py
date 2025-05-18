#!/usr/bin/python3

import gym
import numpy as np
import cv2
from gym import spaces
import time
import os

from main_copy import kill_all_ros_processes, start_world
from cobot_controls import move_cobot, control_gripper
from service_server_control import *



class GraspEnv(gym.Env):
    def __init__(self, image_path):
        super(GraspEnv, self).__init__()
        self.image_path = image_path  
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)  
        self.action_space = spaces.Box(low=np.array([0, 0.05]), high=np.array([1, 0.7]), dtype=np.float32)  
    def reset(self):
        kill_all_ros_processes()  
        start_world()  

        launch_tracker() 
        launch_euclid_distance()
        launch_gripper_tracker()

        self.min_distance = 2

        self.obj_x1, self.obj_y1, self.obj_z1 = get_object_position()
        time.sleep(2)  
        
        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Image file not found: {self.image_path}")
        
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            raise ValueError("Failed to load image from path.")
        return self.image 
    
    def _my_reset(self):
        self.image = self.reset()
        return self.image

    def step(self, action):
        reward = 0
        z = 0.5
        x, y = action 
        fail = move_cobot(x, y, z)
        if fail == -1: 
            reward = -1
            return self.image, reward, True, {}
        
        obj_x2, obj_y2, obj_z2 = get_object_position()
        displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 + (obj_y2 - self.obj_y1) ** 2 + (obj_z2 - self.obj_z1) ** 2)
        threshold = 0.055
        if displacement > threshold:
            reward = -10
            return self.image, reward, True, {}
        
        control_gripper(0.055)
        distance = get_euclidean_distance()

        success = self.check_grasp_success(x, y, z, distance)
        if success:
            reward = 10
            return self.image, reward, True, {}
        else:
            if 0 <= distance <= (0.1**2 + 0.1**2 + 0**2) ** (0.5):
                reward = 0
            else :
                reward = (self.min_distance - distance)

            self.min_distance = min(self.min_distance, max(distance, (0.1**2 + 0.1**2 + 0**2)**(1/2)))
        control_gripper(0.0)
        return self.image, reward, False, {}

    def check_grasp_success(self, x, y, z, distance):
        error = get_gripper_disposition()
        if (error >= 0.0075): 
            pass
        threshold_error = 0.0075
        if error >= threshold_error: 
            move_cobot(x, y, z + 0.1)
            obj_x2, obj_y2, obj_z2 = get_object_position()
            displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 + (obj_y2 - self.obj_y1) ** 2 + (obj_z2 - self.obj_z1) ** 2)
            threshold_displacement = 0.055
            if displacement >= threshold_displacement: 
                return True


        return False
