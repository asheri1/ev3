#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor, UltrasonicSensor

# Initialize motors
tank = MoveTank(OUTPUT_A, OUTPUT_B)

# Initialize sensors
color_sensor = ColorSensor(INPUT_4)
touch_sensor = TouchSensor(INPUT_1)
distance_sensor = UltrasonicSensor(INPUT_2)

# Function to move forward and print sensor values
def move_forward():
    tank.on(50, 50)  # Adjust the speed as needed
    print("Color Sensor Value:", color_sensor.color)
    print("Touch Sensor Value:", touch_sensor.is_pressed)
    print("Distance Sensor Value:", distance_sensor.distance_centimeters)

try:
    # Move forward for 5 seconds (adjust as needed)
    tank.on_for_seconds(50, 50, 5)

    # Stop the motors
    tank.off()

except KeyboardInterrupt:
    # Stop the motors if the program is interrupted
    tank.off()
    

# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # print something to the screen of the device
    print('Hello World!')

    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    # wait a bit so you have time to look at the display before the program
    # exits
    time.sleep(5)

if __name__ == '__main__':
    main()
