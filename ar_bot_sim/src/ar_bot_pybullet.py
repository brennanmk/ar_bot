#!/usr/bin/env python3

"""
Brennan Miller-Klugman

Based off of
    - https://github.com/erwincoumans/pybullet_robots/blob/master/turtlebot.py
    - https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e
Resources used for lidar: 
    - https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py
    - https://github.com/axelbr/racecar_gym/blob/master/racecar_gym/bullet/sensors.py
Resources used for camera:
    - https://www.programcreek.com/python/example/122153/pybullet.computeViewMatrixFromYawPitchRoll

Simulator for AR Bot in PyBullet
"""

import pybullet as p
from rospkg import RosPack
import os
import time
import numpy as np


class ARBotPybullet:
    def __init__(self, client: int) -> None:
        """class to spawn in and control arbot

        :param client: physics sim client ID
        """
        self.client = client

        rp = RosPack()
        urdf_path = os.path.join(rp.get_path("ar_bot_description"), "urdf/ar_bot.urdf")

        random_start = np.random.uniform(-0.35, 0.35)

        self.arbot = p.loadURDF(
            urdf_path, [0.59, random_start, 0.05], physicsClientId=client
        )

        self.speed = 10

    def apply_action(self, action: tuple) -> None:
        """
        Performs action

        :param action: tuple consisting of translation and rotation
        """
        linear, angular = action

        left_wheel_vel = (linear - angular) * self.speed
        right_wheel_vel = (linear + angular) * self.speed

        p.setJointMotorControl2(
            self.arbot,
            0,
            p.VELOCITY_CONTROL,
            targetVelocity=left_wheel_vel,
            force=1000,
            physicsClientId=self.client,
        )
        p.setJointMotorControl2(
            self.arbot,
            1,
            p.VELOCITY_CONTROL,
            targetVelocity=right_wheel_vel,
            force=1000,
            physicsClientId=self.client,
        )

    def lidar(self) -> list:
        """simulate lidar measurement
        """

        ray_from = []
        ray_to = []
        num_rays = 8

        lidar_range = 1

        robot_translation, robot_orientation = p.getBasePositionAndOrientation(
            self.arbot
        )

        # Cast rays and get measurements
        for ray_angle in np.linspace(120, 240, num_rays):
            ray_angle = (
                np.radians(ray_angle) + p.getEulerFromQuaternion(robot_orientation)[2]
            )

            ray_direction = np.array([np.cos(ray_angle), np.sin(ray_angle), 0])

            lidar_end_pos = robot_translation + lidar_range * ray_direction

            ray_from.append(robot_translation)
            ray_to.append(lidar_end_pos)

        result = p.rayTestBatch(ray_from, ray_to)
        return np.array(result, dtype=object)[:, 2]

    def camera(self):
        """Produces top down camera image of environment
        """

        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=50,
            yaw=0,
            pitch=-90,
            roll=0,
            upAxisIndex=2,
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=1, aspect=float(1920) / 1080, nearVal=0.1, farVal=100.0
        )
        (_, _, px, _, _) = p.getCameraImage(
            width=1920,
            height=1080,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        return px


class teleoperate:
    def __init__(self) -> None:
        """helper class to allow teleoperation of the arbot"""
        self.client = p.connect(p.GUI)

        rp = RosPack()
        plane_path = os.path.join(
            rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf"
        )
        plane = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")

        for obstacle in range(3):
            obstacle_x = np.random.uniform(-0.25, 0.25)
            obstacle_y = np.random.uniform(-0.485, 0.485)

            obstacle = p.loadURDF(cube_path, [obstacle_y, obstacle_x, 0.05])

        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        random_goal = np.random.uniform(-0.35, 0.35)
        goal = p.loadURDF(goal_path, [-0.585, random_goal, 0])

        arbot = ARBotPybullet(self.client)

        p.setRealTimeSimulation(1)
        p.setGravity(0, 0, -10)

        forward = 0
        turn = 0

        while 1:
            p.stepSimulation()
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

            arbot.apply_action((forward, turn))
            arbot.lidar()

            time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    teleoperate()
