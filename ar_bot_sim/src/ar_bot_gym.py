import gymnasium as gym
from gymnasium import error, spaces, utils
from gymnasium.utils import seeding
import numpy as np
import pybullet as p
import os
from rospkg import RosPack
import random
from ar_bot_pybullet import ARBotPybullet


class ARBotGym(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-40,
            cameraTargetPosition=[0.55, -0.35, 0.2],
        )
        self.action_space = gym.spaces.box.Box(
            low=np.array([0, 0]),  # Linear x and yaw between 0 and 1.5 m/s
            high=np.array([1.5, 1.5]),
        )
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-10, -10, -1, -1, -5, -5, -10, -10]),
            high=np.array([10, 10, 1, 1, 5, 5, 10, 10]),
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)
        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1 / 30, self.client)

        self.car = None
        self.goal = None
        self.done = False
        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()

    def step(self, action):
        # Feed action to the car and get observation of car's state
        self.car.apply_action(action)
        p.stepSimulation()
        car_ob = self.car.get_observation()

        # Compute reward as L2 change in distance to goal
        dist_to_goal = math.sqrt(
            ((car_ob[0] - self.goal[0]) ** 2 + (car_ob[1] - self.goal[1]) ** 2)
        )
        reward = max(self.prev_dist_to_goal - dist_to_goal, 0)
        self.prev_dist_to_goal = dist_to_goal

        # Done by running off boundaries
        if car_ob[0] >= 10 or car_ob[0] <= -10 or car_ob[1] >= 10 or car_ob[1] <= -10:
            self.done = True
        # Done by reaching goal
        elif dist_to_goal < 1:
            self.done = True
            reward = 50

        ob = np.array(car_ob + self.goal, dtype=np.float32)
        return ob, reward, self.done, dict()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -10)

        rp = RosPack()
        plane_path = os.path.join(
            rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf"
        )
        plane = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")

        # Spawn random obstacles
        for obstacle in range(3):
            obstacle_x = np.random.uniform(-0.25, 0.25)
            obstacle_y = np.random.uniform(-0.485, 0.485)

            obstacle = p.loadURDF(cube_path, [obstacle_y, obstacle_x, 0.05])

        # Spawn random goal
        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        goal_x = np.random.uniform(-0.35, 0.35)
        goal_y = -0.585
        goal = p.loadURDF(goal_path, [goal_x, goal_y, 0])

        # Spawn robot randomly
        arbot = ARBotPybullet(self.client)

        self.goal = (goal_x, goal_y)
        self.done = False

        p.configureDebugVisualizer(
            p.COV_ENABLE_RENDERING, 1
        )  # rendering's back on again
        return observation

    def render(self, mode="human"):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        car_id, client_id = self.car.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=80, aspect=1, nearVal=0.01, farVal=100
        )
        pos, ori = [list(l) for l in p.getBasePositionAndOrientation(car_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(0.00001)

    def close(self):
        p.disconnect(self.client)
