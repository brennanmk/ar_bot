# ARBot Sim

## Overview

This package contains a pybullet simulation for the ARBot as well as an OpenAI Gynasium environment

The package has two main components, agents and environments. The agents, which at the moment consists only of a PyBullet simulator for the ARBot. Future augmentation could see the addition of a different simulated agent (such as gazebo). Environments are example environments that the ARBot can be spawned into. At the moment only a tabletop environment is included. In this environment, the agent is spawned in a random location, a goal is spawned in a random location, and obstacles are spawned randomly.

## Getting started

`pip install -r requirements.txt` can be ran to install all of the packages needed to run the nodes provided in this package.

Additionally, your machine will likely need ROS Noetic, the install instructions for which can be found at http://wiki.ros.org/noetic/Installation/Ubuntu.

## Maps

The src/maps directory holds two maps that can be deployed

    - The plane map was provided from https://github.com/erwincoumans/pybullet_robots/tree/master
