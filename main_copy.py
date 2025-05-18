#!/usr/bin/python3

import subprocess
import os
import time
import cv2
import sys

processes = [] 

def kill_all_ros_processes():
    subprocess.run(["pkill", "-f", "ros"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gazebo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    process_list = subprocess.run(
        ["pgrep", "-f", "python3"],
        stdout=subprocess.PIPE,
        text=True
    ).stdout.strip().split("\n")

    for pid in process_list:
        if pid.isdigit():
            cmdline = subprocess.run(
                ["ps", "-p", pid, "-o", "cmd="],
                stdout=subprocess.PIPE,
                text=True
            ).stdout.strip()

            if ("__class_gym_continuous" not in cmdline) \
            and ("train" not in cmdline) \
            and ("__class_gym_discrete" not in cmdline) \
            and ("successfull_trajectory" not in cmdline):
                subprocess.run(["kill", "-9", pid])

    subprocess.run(["pkill", "-f", "pick_place_simple_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    subprocess.run(["pkill", "-f", "object_tracker"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(2)

    subprocess.run(["killall", "-9", "rosmaster", "roslaunch", "gzserver", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    os.system("rosnode kill -a") 
    os.system("pkill -f ros")  
    os.system("pkill -f gazebo") 
    os.system("pkill -9 gzserver") 
    os.system("pkill -9 gzclient") 

    return 0

def start_world():

    sim_process = subprocess.Popen(
    ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch cobot_sim spawn_robot_2Dcamera_traj_controller_world.launch"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    preexec_fn=os.setsid 
    )

    processes.append(sim_process)

    camera_process = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && python3 /home/is/catkin_ws/src/camera_publisher.py"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(camera_process)
    time.sleep(5) 
    
    return 0


import signal
import subprocess
import sys
import time
import select 


if __name__ == "__main__":
    pass