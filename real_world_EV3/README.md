# Real-World EV3 Robot

## Description
    This project provides scripts for controlling an EV3 robot. 
    It includes modules for loading a QTable, controlling the robot's movements, and recording the results.

    Results recorded in `results.txt` and `results - no obstacles.txt` are analyzed in order to evaluate the performance of the robot control algorithm in reality VS simulated enviorment. 


## Files

1. `QTable.json`
    - This file contains a QTable, which is used by ev3control.py to determine the robot's actions based on the current state.

2. `ev3control.py`
    - This script is responsible for controlling the EV3 robot. 
        It loads the QTable from QTable.json and uses it to determine the robot's actions, based on its sensors states.
        The script interfaces with the robot's motors and sensors to execute movements and navigate it towards the goal.

3. `screen_utils.py`
    - This module provides utility functions for displaying information on the screen of the EV3 robot. 
        It is used by ev3control.py to provide feedback and status updates during operation.

4. `run.py`
    - This script is the entry point for running the robot control program. 
        It imports and executes functions from ev3control.py to control the robot's behavior.

5. 
    * `results - no obstacles.txt`
        This file contains recorded results of robot movements when no obstacles are present in the environment.

    * `results.txt`
        This file contains recorded results of robot movements, including obstacle avoidance and navigation.


## Usage
    - Ensure that the EV3 robot is connected and properly configured.
    - Run the run.py script to start the robot control program.

## Dependencies
    - Python 3.x.
    - ev3dev Python library (for interfacing with the EV3 robot).
    - JSON


