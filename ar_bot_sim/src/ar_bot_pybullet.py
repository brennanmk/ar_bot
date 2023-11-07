#!/usr/bin/env python3

'''
Brennan Miller-Klugman

Based off of
    - https://github.com/erwincoumans/pybullet_robots/blob/master/turtlebot.py
    - https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e

Simulator for AR Bot in PyBullet
'''

import pybullet as p
from rospkg import RosPack
import os
import time
import numpy as np
import cv2
import math
class ARBotPybullet:
    def __init__(self, client: int) -> None:
        '''class to spawn in and control arbot

        :param client: physics sim client ID
        '''
        self.client = client

        rp = RosPack()
        urdf_path = os.path.join(rp.get_path("ar_bot_description"), "urdf/ar_bot.urdf")
        
        random_start = np.random.uniform(-0.35, 0.35)

        self.arbot = p.loadURDF(urdf_path,[0.59, random_start, 0.05], physicsClientId=client)

        self.speed = 10

        self.rayIds = []

    def apply_action(self, action: tuple):
        linear, angular = action

        left_wheel_vel = (linear-angular)*self.speed
        right_wheel_vel = (linear+angular)*self.speed

        p.setJointMotorControl2(self.arbot,0,p.VELOCITY_CONTROL,targetVelocity=left_wheel_vel,force=1000, physicsClientId=self.client)
        p.setJointMotorControl2(self.arbot,1,p.VELOCITY_CONTROL,targetVelocity=right_wheel_vel,force=1000, physicsClientId=self.client)

    def lidar(self):
        '''simulate lidar measurement

        https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py
        https://github.com/axelbr/racecar_gym/blob/master/racecar_gym/bullet/sensors.py
        '''
        rayFrom = []
        rayTo = []
        numRays = 8

        rayLen = 1

        rayHitColor = [1, 0, 0]
        rayMissColor = [0, 1, 0]
        cubeLinear, cubeAngle = p.getBasePositionAndOrientation(self.arbot)

        for i in range(numRays):
            rayFrom.append(cubeLinear)
            rayTo.append([
                rayLen * math.sin(2. * math.pi * float(i) / numRays),
                rayLen * math.cos(2. * math.pi * float(i) / numRays), 
                cubeLinear[2]
            ])
            self.rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))


        results = p.rayTestBatch(rayFrom, rayTo)
        p.removeAllUserDebugItems()

        for i in range(numRays):
            hitObjectUid = results[i][0]

        if (hitObjectUid < 0):
            hitPosition = [0, 0, 0]
            p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=self.rayIds[i])
        else:
            hitPosition = results[i][3]
            p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=self.rayIds[i])


    def camera(self):
        '''based of of https://www.programcreek.com/python/example/122153/pybullet.computeViewMatrixFromYawPitchRoll
        '''
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0,0,0],
            distance=50,
            yaw=0,
            pitch=-90,
            roll=0,
            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=1, aspect=float(1920)/1080,
            nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(
            width=1920, height=1080, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        return px
class teleoperate:
    def __init__(self) -> None:
        '''helper class to allow teleoperation of the arbot
        '''
        self.client = p.connect(p.GUI)

        rp = RosPack()
        plane_path = os.path.join(rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf")
        plane = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")
        
        for obstacle in range(3):
                obstacle_x = np.random.uniform(-0.25, 0.25)
                obstacle_y = np.random.uniform(-0.485, 0.485)
                
                obstacle = p.loadURDF(cube_path, [obstacle_y,obstacle_x,0.05])

        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        random_goal = np.random.uniform(-0.35, 0.35)
        goal = p.loadURDF(goal_path, [-0.585,random_goal,0])

        arbot = ARBotPybullet(self.client)

        p.setRealTimeSimulation(1)
        p.setGravity(0,0,-40)

        forward=0
        turn=0

        while (1):
            time.sleep(1./30.)
            keys = p.getKeyboardEvents()
            leftWheelVelocity=0
            rightWheelVelocity=0
            speed=10
            
            for k,v in keys.items():

                        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                                turn = -0.5
                        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                                turn = 0
                        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                                turn = 0.5
                        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                                turn = 0

                        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                                forward=1
                        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                                forward=0
                        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                                forward=-1
                        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                                forward=0

            arbot.apply_action((forward, turn))
            arbot.lidar()


if __name__ == '__main__':
    teleoperate()