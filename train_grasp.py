import time
from kill import kill
from stable_baselines3 import PPO, SAC, A2C, DQN
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback 
from __class_grasp_gym_discrete import GraspEnvDiscrete 
from _class_save_training import SaveMetricsCallback

from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack
from stable_baselines3.dqn import CnnPolicy

if __name__ == "__main__":
    try:
        env = DummyVecEnv([lambda: GraspEnvDiscrete()])
        env = VecFrameStack(env, n_stack=2)
        model = DQN(CnnPolicy, env, verbose=1, buffer_size=10000)
        metrics_callback = SaveMetricsCallback(log_dir="/home/is/catkin_ws/src/____logs/")
        checkpoint_callback = CheckpointCallback(
            save_freq=40,              
            save_path='./models/',     
            name_prefix='dqn_checkpoint' 
        )
        start_time = time.time()  
        model.learn(total_timesteps=1500, callback=[metrics_callback, checkpoint_callback])
        end_time = time.time()  
        elapsed_time = end_time - start_time  
        model.save("models/dqn_testing_160225.zip")
        env.close()
        kill()
    except KeyboardInterrupt:
        model.save("models/a2c_testing_160225.zip")
        kill()
