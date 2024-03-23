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

Please note that this script is designed to be run in a specific environment with an EV3 robot. Make sure to adjust the parameters and functions as needed to fit your specific setup.
