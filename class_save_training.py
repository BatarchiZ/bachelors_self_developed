import csv
import os
from stable_baselines3.common.callbacks import BaseCallback
import numpy as np

class SaveMetricsCallback(BaseCallback):
    def __init__(self, log_dir="logs", verbose=1):
        super().__init__(verbose)
        self.log_dir = log_dir
        self.csv_file = os.path.join(log_dir, "/home/is/catkin_ws/src/____logs/training_metrics.csv")
        self.first_write = True  

    def _on_step(self) -> bool:
        reward = self.locals["rewards"]  
        loss = self.model.logger.name_to_value.get("loss/value_loss", None) 
        position = self.training_env.get_attr("position")[0]
        angle = self.training_env.get_attr("angle")[0]
        x, y, z = position[0], position[1], 0.7

        os.makedirs(self.log_dir, exist_ok=True)
        with open(self.csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)

            if self.first_write:
                writer.writerow(["Step", "Reward", "Loss", "X", "Y", "ANGLE"])
                self.first_write = False

            writer.writerow([self.num_timesteps, np.round(reward,4), loss, x, y, angle])

        return True 
