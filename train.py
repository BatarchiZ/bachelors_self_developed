from terminate import kill
import gymnasium as gym
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.vec_env import DummyVecEnv
from class_gym_continuous import GraspEnv 
import pandas as pd
import time
from stable_baselines3 import A2C
from class_save_training import SaveMetricsCallback


if __name__ == "__main__":
    try:
        env = GraspEnv(image_path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")
        model = A2C("MlpPolicy", env, verbose=1, n_steps=1)
        callback = SaveMetricsCallback(log_dir="/home/is/catkin_ws/src/____logs/")
        model.learn(total_timesteps=100, callback=callback)
        model.save("z_models/a2c_testing_110225.zip")
        env.close()
        kill()
    except KeyboardInterrupt:
        model.save("z_models/a2c_testing_110225.zip")
        kill()
    