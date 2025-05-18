import time
import os
from terminate import kill
import gymnasium as gym
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack
from stable_baselines3.dqn import CnnPolicy
from stable_baselines3.common.callbacks import CheckpointCallback
from class_grasp_gym_discrete import GraspEnvDiscrete 
from class_save_training import SaveMetricsCallback

CHECKPOINT_PATH = "/home/is/catkin_ws/src/_o_output/dqn_checkpoint_3000_steps.zip"  

if __name__ == "__main__":
    try:
        env = DummyVecEnv([lambda: GraspEnvDiscrete()])
        env = VecFrameStack(env, n_stack=2)
        if os.path.exists(CHECKPOINT_PATH):
            model = DQN.load(CHECKPOINT_PATH, env=env)  # Load with env
        else:
            model = DQN(CnnPolicy, env, verbose=1, buffer_size=10000)
        metrics_callback = SaveMetricsCallback(log_dir="/home/is/catkin_ws/src/____logs/")
        checkpoint_callback = CheckpointCallback(
            save_freq=120,               # Save every 100 steps
            save_path='./models/',     # Save directory
            name_prefix='dqn_checkpoint' # Checkpoint file prefix
        )
        start_time = time.time()
        model.learn(total_timesteps=3000, callback=[metrics_callback, checkpoint_callback])
        end_time = time.time()
        model.save("models/dqn_testing_160225.zip")
        env.close()
        kill()

    except KeyboardInterrupt:
        model.save("models/dqn_interrupted.zip")
        kill()
