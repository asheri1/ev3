# Webots Robot Simulation

This repository contains files related to a robot simulation in Webots. The simulation involves a robot navigating an environment, learning through Q-Learning, and interacting with various objects.

## Files

1. **`ev3_bot.py`**: This Python script controls the EV3 robot in the simulation. It includes functions for motor control, sensor integration, and Q-Learning.

2. **`world.wbt`**: The Webots world file defines the simulated environment. It contains colored boxes, a robot, and various sensors.

3. **`QTable.json`**: The Q-table output from the training process. It contains learned Q-values for state-action pairs.

## Features

- **Robot Control**: The `ev3_bot.py` script provides methods for controlling the robot's motors and processing sensor data.
- **Sensor Integration**: The robot uses touch sensors, distance sensors, and a camera to perceive its environment.
- **Q-Learning**: The Q-table represents the learned policy for the robot.
- **Webots Environment**: The `world.txt` file defines the simulated world where the robot operates.

## Dependencies

The script requires the following Python libraries:

- numpy
- pandas
- cv2 (OpenCV)
- colorthief
- numpy
- pandas
- json
  
## Usage

1. Install Webots (version R2023b or later).
2. Open `world.txt` in Webots to visualize the environment.
3. Run the `ev3_bot.py` script to train the robot using Q-Learning.

Happy simulating! 🤖🚀
