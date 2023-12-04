import gymnasium as gym
import torch as th

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from ar_bot_gym import ARBotGym

# Parallel environments
vec_env = ARBotGym()

model = PPO("MlpPolicy", vec_env, verbose=1)


model.learn(total_timesteps=25000)
model.save("arbot")

# Visualize
obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    vec_env.render("human")