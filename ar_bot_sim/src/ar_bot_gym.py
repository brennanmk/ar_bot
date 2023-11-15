import gymnasium as gym
import pybullet as p
import os
from rospkg import RosPack
from ar_bot_pybullet import ARBotPybullet
import numpy as np

class ARBotGym(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        p.connect(p.GUI)

        self.action_space = gym.spaces.box.Box(
            low=np.array([0, 0]),  # Linear x and yaw between 0 and 1.5 m/s
            high=np.array([1.5, 1.5]),
        )

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-1, -1, -1, -1, -1, -1, -1, -1]), # Lidar readings between -1 and 1
            high=np.array([1, 1, 1, 1, 1, 1, 1, 1]),
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)

        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1 / 30, self.client)

        self.ar_bot = None
        self.goal = None

        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()

    def step(self, action):
        # Feed action to the car and get observation of car's state
        self.car.apply_action(action)

        p.stepSimulation()

        robot_translation, _ = p.getBasePositionAndOrientation(
            self.arbot
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

        return self.ar_bot.lidar(), reward, complete, False, {}

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -10)

        rp = RosPack()
        plane_path = os.path.join(
            rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf"
        )
        _ = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")

        # Spawn random obstacles
        for _ in range(3):
            obstacle_x = np.random.uniform(-0.25, 0.25)
            obstacle_y = np.random.uniform(-0.485, 0.485)

            _ = p.loadURDF(cube_path, [obstacle_y, obstacle_x, 0.05])

        # Spawn random goal
        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        goal_x = np.random.uniform(-0.35, 0.35)
        goal_y = -0.585
        p.loadURDF(goal_path, [goal_x, goal_y, 0])

        # Spawn robot randomly
        self.ar_bot = ARBotPybullet(self.client)

        self.goal = (goal_x, goal_y)

        return self.ar_bot.lidar()

    def close(self):
        p.disconnect(self.client)
