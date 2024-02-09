import gymnasium as gym
import pybullet as p
import numpy as np
import random
import time
import os
from typing import Optional
from pybullet_utils import bullet_client
from ar_bot_sim.pybullet_sim.ar_bot_pybullet import ARBotPybullet
import rospkg

class ARBotTabletopGym(gym.Env):
    """
    Gym environment for ARBot
    """

    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        config
    ):
        """
        Setup Gym environment, start pybullet and call reset
        """

        self.discrete_action_mapping = config["discrete_action_mapping"]
        self.render_simulation = config["render_simulation"]
        self.number_of_obstacles = config["number_of_obstacles"] 
        self.action_space = config["actions"]
        self.max_timesteps_per_episode = config["max_timesteps_per_episode"]

        rospack = rospkg.RosPack()
        simulator_path = rospack.get_path("ar_bot_sim")
        robot_description_path = rospack.get_path("ar_bot_description")

        self.plane_path = os.path.join(simulator_path, "src/ar_bot_sim/environments/tabletop/maps/arena/arena.urdf")
        self.cube_path = os.path.join(simulator_path, "src/ar_bot_sim/environments/tabletop/obstacles/cube.urdf")
        self.goal_path = os.path.join(simulator_path, "src/ar_bot_sim/environments/tabletop/obstacles/goal.urdf")
        self.ar_bot_urdf_path = os.path.join(robot_description_path, "urdf/ar_bot.urdf")


        self.observation_space = gym.spaces.box.Box(
            low=np.array(
                [-1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            ),  # x, y distance to goal, and Lidar readings between 0 and 1
            high=np.array([1.5, 1.5, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
        )

        self.total_sum_reward_tracker = []
        self.total_timestep_tracker = []

        self.episode_reward_tracker = []

        self.client = bullet_client.BulletClient(
            p.GUI if self.render_simulation else p.DIRECT
        )

        self.client.setTimeStep(1 / 30)

        self.ar_bot = None
        self.goal = None

        self.count = 0
        self.reset(seed=config.worker_index * config.num_workers * int(time.time()))

    def step(self, action):
        """
        Take action and return observation

        :param action: action to take
        """
        action = self.discrete_action_mapping[action]

        self.ar_bot.apply_action(action)

        p.stepSimulation()

        robot_translation, _ = p.getBasePositionAndOrientation(self.ar_bot.arbot)
        reward = -0.1

        dist_to_goal_y = robot_translation[0] - self.goal[0]
        dist_to_goal_x = robot_translation[1] - self.goal[1]

        complete = False

        lidar = list(self.ar_bot.lidar())

        self.count += 1
        if self.count >= self.max_timesteps_per_episode:
            complete = True
            self.count = 0

        # check if goal reached, if so give large reward
        if -0.05 < dist_to_goal_y < 0.05 and -0.05 < dist_to_goal_x < 0.05:
            complete = True
            reward = 1000
            self.count = 0

        obs = [dist_to_goal_y, dist_to_goal_x] + lidar

        self.episode_reward_tracker.append(reward)

        return np.array(obs, dtype=np.float32), reward, complete, False, {}

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        """
        Reset robots posistion and goal posistion randomly
        """
        if seed is not None: random.seed(seed)

        p.resetSimulation()
        p.setGravity(0, 0, -10)

        self.count = 0

        _ = p.loadURDF(self.plane_path)
 
        for _ in range(random.randint(0, self.number_of_obstacles)):
            obstacle_x = random.uniform(-0.25, 0.25)
            obstacle_y = random.uniform(-0.4, 0.4)

            _ = p.loadURDF(self.cube_path, [obstacle_y, obstacle_x, 0.05])

        # Spawn random goal
        goal_x = random.uniform(-0.335, 0.335)
        goal_y = -0.585
        p.loadURDF(self.goal_path, [goal_y, goal_x, 0])

        # Spawn robot in at random location
        random_start = random.uniform(-0.35, 0.35)

        arbot = self.client.loadURDF(self.ar_bot_urdf_path, [0.575, random_start, 0.05])

        self.ar_bot = ARBotPybullet(self.client, arbot, self.render_simulation)

        self.goal = (goal_y, goal_x)

        robot_translation, _ = p.getBasePositionAndOrientation(self.ar_bot.arbot)

        dist_to_goal_y = robot_translation[0] - self.goal[0]
        dist_to_goal_x = robot_translation[1] - self.goal[1]

        lidar = list(self.ar_bot.lidar())

        obs = [dist_to_goal_y, dist_to_goal_x] + lidar

        return np.array(obs, dtype=np.float32), {}

    def close(self):
        """
        Close pybullet sim
        """

        self.client.disconnect()
