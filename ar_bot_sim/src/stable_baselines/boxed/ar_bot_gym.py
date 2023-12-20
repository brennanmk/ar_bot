import gymnasium as gym
import pybullet as p
import os
from rospkg import RosPack
from ar_bot_pybullet import ARBotPybullet
import numpy as np
from typing import Optional

class ARBotGym(gym.Env):
    '''
    Gym environment for ARBot
    '''

    metadata = {"render.modes": ["human"]}

    def __init__(self, render: bool = False):
        '''
        Setup Gym environment, start pybullet and call reset

        the provided constructor argument "render" determines wheter pybullet is run headlessly
        '''

        self.render = render

        self.action_space = gym.spaces.box.Box(
            low=np.array([-0.5, -0.5]),  # Linear x and yaw between 0 and 1.5 m/s
            high=np.array([0.5, 0.5]),
        )

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]), # x, y distance to goal, and Lidar readings between 0 and 1
            high=np.array([1.5, 1.5, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
        )

        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.GUI if self.render is not False else p.DIRECT)

        p.setTimeStep(1 / 30, self.client)

        self.ar_bot = None
        self.goal = None

        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.count = 0
        self.reset()

    def step(self, action):
        '''
        Take action and return observation

        :param action: action to take
        '''

        self.ar_bot.apply_action(action)

        p.stepSimulation()

        robot_translation, _ = p.getBasePositionAndOrientation(
            self.ar_bot.arbot
        )
        reward = -0.1

        dist_to_goal_y = robot_translation[0] - self.goal[0]
        dist_to_goal_x = robot_translation[1] - self.goal[1]

        complete = False

        lidar = list(self.ar_bot.lidar())

        # check if goal reached, if so give large reward
        if -0.05 < dist_to_goal_y < 0.05 and -0.05 < dist_to_goal_x < 0.05:
            complete = True
            reward = 1000
            self.count = 0

        self.count += 1
        if self.count > 1500:
            complete = True
            self.count = 0

        obs = [dist_to_goal_y, dist_to_goal_x] + lidar

        return np.array(obs, dtype=np.float32), reward, complete, False, {}

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        '''
        Reset robots posistion and goal posistion randomly
        '''

        p.resetSimulation()
        p.setGravity(0, 0, -10)

        rp = RosPack()
        plane_path = os.path.join(
            rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf"
        )
        _ = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")

        # Spawn random goal
        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        goal_x = np.random.uniform(-0.35, 0.35)
        goal_y = -0.585
        p.loadURDF(goal_path, [goal_y, goal_x, 0])
        
        # Spawn robot randomly
        self.ar_bot = ARBotPybullet(self.client, self.render)

        self.goal = (goal_y, goal_x)

        robot_translation, _ = p.getBasePositionAndOrientation(
            self.ar_bot.arbot
        )

        dist_to_goal_y = robot_translation[0] - self.goal[0]
        dist_to_goal_x = robot_translation[1] - self.goal[1]

        lidar = list(self.ar_bot.lidar())

        obs = [dist_to_goal_y, dist_to_goal_x] + lidar

        return np.array(obs, dtype=np.float32), {}

    def close(self):
        '''
        Close pybullet sim
        '''
    
        p.disconnect(self.client)
