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

import numpy as np
from pybullet_utils import bullet_client


class ARBotPybullet:
    def __init__(self, client: bullet_client, arbot: object, gui: bool) -> None:
        """class to spawn in and control arbot

        :param client: physics sim client ID
        """
        self.client = client
        self.gui = gui
        self.arbot = arbot

        self._hit_color = [1, 0, 0]
        self._miss_color = [0, 1, 0]
        self._ray_ids = []

        self.speed = 15

    def apply_action(self, action: tuple) -> None:
        """
        Performs action

        :param action: tuple consisting of translation and rotation
        """
        linear, angular = action

        left_wheel_vel = (linear - angular) * self.speed
        right_wheel_vel = (linear + angular) * self.speed

        self.client.setJointMotorControl2(
            self.arbot,
            0,
            self.client.VELOCITY_CONTROL,
            targetVelocity=left_wheel_vel,
            force=0.001,
        )
        self.client.setJointMotorControl2(
            self.arbot,
            1,
            self.client.VELOCITY_CONTROL,
            targetVelocity=right_wheel_vel,
            force=0.001,
        )

    def lidar(self) -> list:
        """simulate lidar measurement"""

        ray_from = []
        ray_to = []
        num_rays = 9

        lidar_range = 1

        robot_translation, robot_orientation = (
            self.client.getBasePositionAndOrientation(self.arbot)
        )

        # Cast rays and get measurements
        for i, ray_angle in enumerate(np.linspace(120, 240, num_rays)):
            ray_angle = (
                np.radians(ray_angle)
                + self.client.getEulerFromQuaternion(robot_orientation)[2]
            )

            ray_direction = np.array([np.cos(ray_angle), np.sin(ray_angle), 0])

            lidar_end_pos = robot_translation + lidar_range * ray_direction

            ray_from.append(robot_translation)
            ray_to.append(lidar_end_pos)

            if self.gui and len(self._ray_ids) < num_rays:
                self._ray_ids.append(
                    self.client.addUserDebugLine(
                        ray_from[i], ray_to[i], self._miss_color
                    )
                )

        result = self.client.rayTestBatch(ray_from, ray_to)

        if self.gui:
            for i in range(num_rays):
                hitObjectUid = result[i][0]

                if hitObjectUid < 0:
                    self.client.addUserDebugLine(
                        ray_from[i],
                        ray_to[i],
                        self._miss_color,
                        replaceItemUniqueId=self._ray_ids[i],
                    )
                else:
                    hit_location = result[i][3]
                    self.client.addUserDebugLine(
                        ray_from[i],
                        hit_location,
                        self._hit_color,
                        replaceItemUniqueId=self._ray_ids[i],
                    )

        return np.array(result, dtype=object)[:, 2]

    def camera(self):
        """Produces top down camera image of environment"""

        view_matrix = self.client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=50,
            yaw=0,
            pitch=-90,
            roll=0,
            upAxisIndex=2,
        )
        proj_matrix = self.client.computeProjectionMatrixFOV(
            fov=1, aspect=float(1920) / 1080, nearVal=0.1, farVal=100.0
        )
        (_, _, px, _, _) = self.client.getCameraImage(
            width=1920,
            height=1080,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=self.client.ER_BULLET_HARDWARE_OPENGL,
        )
        return px
