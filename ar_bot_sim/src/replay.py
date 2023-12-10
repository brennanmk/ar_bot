import gymnasium as gym
import torch as th
import pybullet as p
import time
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from ar_bot_gym import ARBotGym
vec_env = make_vec_env(ARBotGym)

model = PPO.load("ppo_arbot")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
