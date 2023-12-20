import gymnasium as gym
import torch as th
import pybullet as p
import time
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from ar_bot_gym import ARBotGym

# Parallel environments
vec_env = ARBotGym()

model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log="./ppo_arbot_tensorboard/")
model.learn(total_timesteps=1000000, progress_bar=True)
model.save("ppo_arbot")

# Parallel environments
