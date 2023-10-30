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

    def apply_action(self, action: tuple):
        linear, angular = action

        left_wheel_vel = (linear-angular)*self.speed
        right_wheel_vel = (linear+angular)*self.speed

        p.setJointMotorControl2(self.arbot,0,p.VELOCITY_CONTROL,targetVelocity=left_wheel_vel,force=1000, physicsClientId=self.client)
        p.setJointMotorControl2(self.arbot,1,p.VELOCITY_CONTROL,targetVelocity=right_wheel_vel,force=1000, physicsClientId=self.client)



class teleoperate:
    def __init__(self) -> None:
        '''helper class to allow teleoperation of the arbot
        '''
        self.client = p.connect(p.GUI)

        rp = RosPack()
        plane_path = os.path.join(rp.get_path("ar_bot_sim"), "src/maps/arena/arena.urdf")
        plane = p.loadURDF(plane_path)

        cube_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/cube.urdf")

        number_of_obstacles = np.random.randint(15)
        for obstacle in range(number_of_obstacles):
                obstacle_x = np.random.uniform(-0.25, 0.25)
                obstacle_y = np.random.uniform(-0.485, 0.485)
                obstacle = p.loadURDF(cube_path, [obstacle_y,obstacle_x,0.05])

        goal_path = os.path.join(rp.get_path("ar_bot_sim"), "src/obstacles/goal.urdf")

        random_goal = np.random.uniform(-0.35, 0.35)
        goal = p.loadURDF(goal_path, [-0.585,random_goal,0])

        arbot = ARBotPybullet(self.client)

        p.setRealTimeSimulation(1)
        p.setGravity(0,0,-10)

        linear=0
        angular=0

        while (1):
            time.sleep(1/30)
            keys = p.getKeyboardEvents()
            
            for k,v in keys.items():
                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        angular = -0.5
                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        angular = 0
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        angular = 0.5
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        angular = 0

                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        linear=1
                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                        linear=0
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        linear=-1
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                        linear=0

            arbot.apply_action((linear, angular))


if __name__ == '__main__':
    teleoperate()