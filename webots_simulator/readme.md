# Simulation Code
## Webots World Simulation

This repository contains a Webots world file `world.wbt` for simulating a robot in a virtual environment. The world includes various objects and a robot with multiple sensors and actuators.

### Features

- **World Setup**: The world consists of a rectangular arena with multiple colored boxes placed at different positions. The arena floor has a parquetry appearance.
- **Robot**: The robot in the simulation is equipped with touch sensors, distance sensors,and a camera. It also has two wheels, each controlled by a separate motor.
- **Sensors**: The robot uses its sensors to perceive its environment. The distance sensors and camera help the robot navigate the world and interact with the colored boxes.

### Dependencies

The simulation requires Webots R2023b or later.

### Usage

To run the simulation, open the `world.wbt` file in Webots. The robot's behavior can be controlled by modifying the `ev3_bot` controller script.

## EV3_Bot.py

This Python script is designed to control an EV3 robot using a variety of sensors and actuators. The robot is capable of performing actions such as moving forward, turning left or right, and reversing based on sensor inputs.

### Features

- **EV3 Robot Control**: The script includes a class `ev` that provides methods for controlling the robot's motors and processing sensor data.
- **Sensor Integration**: The robot uses a distance sensor, a touch sensor, and a camera to perceive its environment.
- **State and Action Spaces**: The script defines the state and action spaces for the robot, which can be used for reinforcement learning.
- **Q-Learning**: The script includes functions for training the robot using Q-learning and for testing the trained policy.
- **Environment Reset**: The script includes a function for resetting the robot's position in the environment.

### Dependencies

The script requires the following Python libraries:

- numpy
- pandas
- cv2 (OpenCV)
- colorthief

### Usage

To run the script, simply execute the `main` function at the bottom of the script. This will create an instance of the `ev` class and start the training process.


