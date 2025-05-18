import subprocess
import time
import select

def control_gripper(position, duration=1.0):
    command = f"""rostopic pub -1 /cobot/gripper_controller/command trajectory_msgs/JointTrajectory '
joint_names: ["gripper_right_joint"]
points:
- positions: [{position}]
  time_from_start: {{secs: {int(duration)}, nsecs: 0}}'"""
    
    subprocess.run(command, shell=True)


def move_cobot(x, y, z, log_file="/home/is/catkin_ws/src/____logs/cobot_log.txt", timeout=130):
    with open(log_file, "a", buffering=1) as log:
        try:
            pick_place = subprocess.Popen(
                ["bash", "-c", f"export PYTHONUNBUFFERED=1; source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && stdbuf -oL rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1  
            )

            start_time = time.time()

            while True:
                if time.time() - start_time > timeout:
                    pick_place.terminate()
                    try:
                        pick_place.wait(timeout=5)  
                    except subprocess.TimeoutExpired:
                        pick_place.kill() 
                    return -1 

                ready_to_read, _, _ = select.select([pick_place.stdout, pick_place.stderr], [], [], 0.1)

                for stream in ready_to_read:
                    line = stream.readline()
                    if line:
                        log.write(line)
                        log.flush()

                if pick_place.poll() is not None:
                    break

            pick_place.wait()  
            return 0 

        except Exception as e:
            return -1 