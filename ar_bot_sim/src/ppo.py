import gymnasium as gym
import torch as th
import pybullet as p

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from ar_bot_gym import ARBotGym

# Parallel environments
vec_env = ARBotGym()

# Visualize
obs = vec_env.reset()
while True:
    keys = p.getKeyboardEvents()

    for k, v in keys.items():
        if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
            turn = -0.5
        if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED):
            turn = 0
        if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
            turn = 0.5
        if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED):
            turn = 0

        if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
            forward = 1
        if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
            forward = 0
        if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
            forward = -1
        if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
            forward = 0

    obs, rewards, dones, info = vec_env.step((forward, turn))
    print(f"rewards: {rewards}, {dones}")
