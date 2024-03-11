#!/usr/bin/env python3

from screen_utils import *
from ev3control import *
import time

TIME_LIMIT = 120
START_TIME  = time.time()


def time_condition():
    current_time = time.time()
    elapsed_time = current_time - START_TIME
    condition =   (elapsed_time <= TIME_LIMIT)
    return condition


def debug_print_run_time():
    end_time = time.time()
    elapsed_time = end_time - START_TIME
    formatted_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    debug_print("Time elapsed during the test: %s", formatted_time)
    


# This allows the module to be imported without executing any code
if __name__ == "__main__":

    set_screen()
    robot = EV3Robot()
     

    condition = time_condition()

    while(condition):
         
         # color is Red, goal reached
        if robot.goal_reached():
            break
        
        if robot.reaching_an_object():
            robot.turn_back()
        
        robot.step_action()

        condition = time_condition()


    debug_print_run_time()

        
    
