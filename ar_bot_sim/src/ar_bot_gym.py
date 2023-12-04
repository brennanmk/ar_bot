import gymnasium as gym
import pybullet as p
import os
from rospkg import RosPack
from ar_bot_pybullet import ARBotPybullet
import numpy as np
from typing import Optional
import rospy
class ARBotGym(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([0, 0]),  # Linear x and yaw between 0 and 1.5 m/s
            high=np.array([1.5, 1.5]),
        )

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-1, -1, 0, 0, 0, 0, 0, 0, 0, 0]), # x, y distance to goal, and Lidar readings between 0 and 1
            high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.GUI)

        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1 / 30, self.client)

        self.ar_bot = None
        self.goal = None
        self.obstacles = None

        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()

    def step(self, action):
        self.ar_bot.apply_action(action)

        p.stepSimulation()

        robot_translation, _ = p.getBasePositionAndOrientation(
            self.ar_bot.arbot
        )

        # reward is a function of the change in distance to goal
        dist_to_goal = np.sqrt(
            ((robot_translation[0] - self.goal[0]) ** 2 + (robot_translation[1] - self.goal[1]) ** 2)
        )
        reward = max(self.prev_dist_to_goal - dist_to_goal, 0)
        self.prev_dist_to_goal = dist_to_goal

        # goal reached
        if dist_to_goal < 0.05:
            complete = True
            reward = 50

        dist_to_obstacles = [np.sqrt(
            ((robot_translation[0] - obstacle[0]) ** 2 + (robot_translation[1] - obstacle[1]) ** 2)) for obstacle in self.obstacles]

        # hit obstacle
        if [dist_to_obstacle < 0.05 for dist_to_obstacle in dist_to_obstacles]:
            complete = True
            reward = -100

        return [(robot_translation[0] - self.goal[0]), (robot_translation[1] - self.goal[1])] + list(self.ar_bot.lidar()), reward, complete, False, {}

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        p.resetSimulation()
        p.setGravity(0, 0, -10)

        rp = RosPack()
        plane_path = os.path.join(
            rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf"
        )
        _ = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")
        self.obstacles = []
        # Spawn random obstacles
        for _ in range(3):
            obstacle_x = np.random.uniform(-0.25, 0.25)
            obstacle_y = np.random.uniform(-0.485, 0.485)

            self.obstacles.append([obstacle_x, obstacle_y])

            _ = p.loadURDF(cube_path, [obstacle_y, obstacle_x, 0.05])

        # Spawn random goal
        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        goal_x = np.random.uniform(-0.35, 0.35)
        goal_y = -0.585
        p.loadURDF(goal_path, [goal_x, goal_y, 0])
        
        # Spawn robot randomly
        self.ar_bot = ARBotPybullet(self.client)

        self.goal = (goal_x, goal_y)

        robot_translation, _ = p.getBasePositionAndOrientation(
            self.ar_bot.arbot
        )

        dist_to_goal = np.sqrt(
            ((robot_translation[0] - self.goal[0]) ** 2 + (robot_translation[1] - self.goal[1]) ** 2)
        )
        self.prev_dist_to_goal = dist_to_goal


        obs = [(robot_translation[0] - self.goal[0]), (robot_translation[1] - self.goal[1])] + list(self.ar_bot.lidar())

        return obs, {}

    def close(self):
        p.disconnect(self.client)
