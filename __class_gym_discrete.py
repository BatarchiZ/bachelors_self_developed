#!/usr/bin/python3

import gym
import numpy as np
from gym import spaces
import time

from main_copy import kill_all_ros_processes, start_world
from cobot_controls import move_cobot, control_gripper
from service_server_control import *
from kill import kill
import cv2
import math 

def compute_angle_to_object(xg, yg, xo, yo):
    dx = xo - xg
    dy = yo - yg
    angle = math.atan2(dy, dx)
    return round(math.degrees(angle))

def time_penalty(steps, reward):
    return reward * np.log(np.sqrt(0.75 * steps))

def gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"):
    img = cv2.imread(path)
    top_crop = 0
    bottom_crop = 270
    left_crop = 200
    right_crop = 200

    height, width, _ = img.shape 

    img = img[top_crop:height - bottom_crop, left_crop:width - right_crop, :]
    mask = cv2.inRange(img, (50, 0, 0), (255, 60, 60))
    filtered_gripper = cv2.bitwise_and(img, img, mask=mask)
    gray = cv2.cvtColor(filtered_gripper, cv2.COLOR_BGR2GRAY)
    gray[gray > 50] = 0
    gray = cv2.resize(gray, (110, 84))
    gray[gray != 0] = 150

    cv2.imshow('cropped_img', gray)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

    return np.expand_dims(gray, -1)


class GraspEnvDiscrete(gym.Env):

    def __init__(self):

        super(GraspEnvDiscrete, self).__init__()
        self.action_space = spaces.Discrete(4)

        """Object position is x=1, y=0, z=0.7"""
        self.low = np.array([-0.01, -0.49])
        self.high = np.array([1.06, 0.51])
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(84, 110, 1), dtype=np.uint8)

        self.position = np.array([0.0, 0.0])

        self.min_distance = 2  
        self.steps = 0

        self.min_x, self.min_y, self.min_z = 0,0,0
        self.angle = 0


    def _pre_reset(self):
        kill_all_ros_processes()  
        start_world()             
        launch_tracker()          
        launch_euclid_distance()  
        launch_gripper_tracker()  
        self.obj_x1, self.obj_y1, self.obj_z1 = get_object_position()
        time.sleep(2)  


    def reset(self):
        self._pre_reset()
        self.position = np.array([0.0, 0.0])
        self.angle = 0
        self.steps = 0
        self.min_distance = 2 
        return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")

    def _my_reset(self):
        self._pre_reset()
        move_cobot(self.position[0], self.position[1], 0.7)
        return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")

    def step(self, action):
        if self.steps == 59:
            reward = -40
            done = True
            return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"), reward, done, {}
            
        self.steps += 1
        delta_x, delta_y = 0, 0

        if action == 0:
            delta_x = 0.05
        elif action == 1:
            delta_x = -0.05
        elif action == 2:
            delta_y = 0.05
        elif action == 3:
            delta_y = -0.05

        angle = compute_angle_to_object(self.position[0] + delta_x, self.position[1] + delta_y, self.obj_x1, self.obj_y1)
        if not (self.low[0] <= self.position[0] + delta_x <= self.high[0]) or \
           not (self.low[1] <= self.position[1] + delta_y <= self.high[1]) or \
            not (-90 <= angle <= 0):

            reward = -5
            done = False
            return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"), reward, done, {}
        
        self.position[0] += delta_x
        self.position[1] += delta_y

        fail = move_cobot(self.position[0], self.position[1], 0.7)
        if fail == -1:
            self._my_reset()

        obj_x2, obj_y2, obj_z2 = get_object_position()
        displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 +
                               (obj_y2 - self.obj_y1) ** 2 +
                               (obj_z2 - self.obj_z1) ** 2)
        threshold = 0.1
        if displacement > threshold:
            reward = -35
            done = True
            self.min_distance = 2
            return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"), reward, done, {}
        

        distance = get_euclidean_distance()
        if distance < 0.2: 
            control_gripper(0.055)

        if self.check_grasp_success(self.position[0], self.position[1], 0.07, distance):
            reward = 40
            done = True
            self.min_distance = 2
            return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"), reward, done, {}
        else:
            gripper_length = (0.1**2 + 0.1**2)**0.5 + 0.01
            gripper_length_adj = (0.1**2 + 0.1**2)**0.5 + 0.06
            reward = 0
            if angle == -90:
                reward += 5
            if angle < self.angle:
                reward += 2
                self.angle = angle
            if distance < self.min_distance:
                reward += 1
                self.min_distance = distance
            if (gripper_length_adj > distance) and angle != -90:
                reward -= 5
                

        if distance < 0.2:
            control_gripper(0.0)
        done = False
        return gather_image(path="/home/is/catkin_ws/src/z_output/recent_frame.jpg"), reward, done, {}

    def check_grasp_success(self, x, y, z, distance):
        error = get_gripper_disposition()
        threshold_error = 0.0075
        if (error >= threshold_error) and distance < 0.2:
            move_cobot(x, y, z + 0.1)
            obj_x2, obj_y2, obj_z2 = get_object_position()
            displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 +
                                   (obj_y2 - self.obj_y1) ** 2 +
                                   (obj_z2 - self.obj_z1) ** 2)
            threshold_displacement = 0.055
            if displacement >= threshold_displacement:
                return True

        return False
