#!/usr/bin/env python3
from screen_utils import *
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds

import json


DISTANCE_LIMIT = 10 #cm


def read_json_to_dict():
    # Load the JSON data from the file
    with open('QTable.json', 'r') as json_file:
        Q_dict_str_keys = json.load(json_file)

    # Convert the keys back to tuples and convert int64 values to Python int
    Q_dict = {tuple(map(int, key.split(','))): int(value) if isinstance(value, int) else value for key, value in Q_dict_str_keys.items()}
    return Q_dict




class EV3Robot:

    def __init__(self):
        
        #Q_table
        self.Q_table = read_json_to_dict()
        
        # Motors
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_B)
        self.tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

        # Sensors
        self.touch_sensor = TouchSensor(INPUT_1)
        self.ultrasonic_sensor = UltrasonicSensor(INPUT_2)
        self.color_sensor = ColorSensor(INPUT_4)
        

        # LEDs
        self.leds = Leds()

    def drive(self, left_speed, right_speed):
        # Drive the robot
        self.tank_drive.on(SpeedPercent(left_speed), SpeedPercent(right_speed))

    def stop_drive(self):
        # Stop drive
        self.tank_drive.off()

    
    def read_sensors_data(self, duration):
        if type(duration)!=int:
            return
        
        for i in range(duration):
            #print("Distance:", self.read_distance())
            debug_print("Distance:", self.read_distance())
            
            #print("color:", self.read_color_sensor())
            debug_print("color:", self.read_color_sensor())

            #print("touch:", self.read_touch_sensor())
            debug_print("touch:", self.read_touch_sensor())
            
            time.sleep(1)



    def drive_with_time_suspension(self, left_speed=70, right_speed=70, duration=1, read_data=0):
        """
        Drives the robot for a specified duration and then stops.
        :param left_speed: Speed of the left motor (percentage).
        :param right_speed: Speed of the right motor (percentage).
        :param duration: Time to drive in seconds.
        """
        self.drive(left_speed, right_speed)

        if read_data:
            self.read_sensors_data(duration) #duration is implemented inside read_sensors_data method.

        else:
            time.sleep(duration)
        
        self.stop_drive()


    def turn_right(self, speed, turn_rate=50, duration=1):
        """
        Turns the robot right.
        :param speed: Base speed of the motors.
        :param turn_rate: Percentage to reduce the speed of the right motor.
        """
        percentage = (100 - turn_rate) / 100
        right_speed = speed * percentage
        self.drive_with_time_suspension(speed, right_speed, duration)


    def turn_left(self, speed, turn_rate=50, duration=1):
        """
        Turns the robot left.
        :param speed: Base speed of the motors.
        :param turn_rate: Percentage to reduce the speed of the left motor.
        """
        percentage = (100 - turn_rate) / 100
        left_speed = speed * percentage
        self.drive_with_time_suspension(left_speed, speed, duration)


    def turn_back(self, speed=100, duration=1):
        """
        Turns the robot backwards.
        :param speed: Base speed of the motors.
        :param turn_rate: Percentage to reduce the speed of the left motor.
        """
        left_speed = 0
        self.drive_with_time_suspension(left_speed, speed, duration)



    def read_touch_sensor(self):
        # Read the touch sensor
        return self.touch_sensor.is_pressed

    def read_color_sensor(self):
        # Read the color sensor
        return self.color_sensor.color

    def read_distance(self):
        # Read the ultrasonic sensor
        return self.ultrasonic_sensor.distance_centimeters
    

    def reaching_an_object(self):
        return self.read_distance() < DISTANCE_LIMIT



    def get_current_state(self):
        # Use the robot's sensor methods to get the current state
        touch_state = self.read_touch_sensor()
        distance_state = int(self.read_distance())
        color_state = self.read_color_sensor()
        state = (touch_state, distance_state, color_state)
        return state
    
    
    def get_next_action(self, current_state):
        # Assuming current_state is a tuple that matches the Q-table's multi-index
        action = self.Q_table[current_state]
        return action


    def execute(self, next_action):
        if next_action == 0:
           self.drive_with_time_suspension(left_speed=30, right_speed=30, duration=0.5)
        elif next_action == 1:
            self.turn_left(speed=50,duration=0.5)
        elif next_action == 2:
            self.turn_right(speed=50,turn_rate=50,duration=0.5)



    def step_action(self):
        state = self.get_current_state() 
        next_action = self.get_next_action(state)
        self.execute(next_action)


    def goal_reached(self):
        # color is Red, goal reached
        return self.read_color_sensor() == 5



