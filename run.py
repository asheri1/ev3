#!/usr/bin/env python3

from screen_utils import *
from ev3control import *


# This allows the module to be imported without executing any code
if __name__ == "__main__":

    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')
    robot = EV3Robot()


    while(True):
        # color is Red, goal reached
        if robot.read_color_sensor() == 5:
            break

        state = robot.get_current_state()
        next_action = robot.get_next_action(state)
        debug_print(time.time(), " ", state, " action=", next_action)
        robot.execute_action(next_action)

    
