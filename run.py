#!/usr/bin/env python3

from screen_utils import *
from ev3control import *
import time

TIME_LIMIT = 120


def time_condition(start_time):
    current_time = time.time()
    elapsed_time = current_time - start_time
    condition =   (elapsed_time <= TIME_LIMIT)
    return condition



def get_run_time(start_time):
    end_time = time.time()
    elapsed_time = end_time - start_time
    formatted_time = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    return formatted_time



def debug_print_run_time(test_num, start_time):
    formatted_time = get_run_time(start_time)
    debug_print("test num: ", test_num)
    debug_print("Time elapsed during the test: ", formatted_time)



def write_results_to_file(test_num, start_time):
    with open('results.txt', 'a') as f:
            formatted_time = get_run_time(start_time)
            f.write(f"Test num: {i+1},  run time: {formatted_time}")

    


# This allows the module to be imported without executing any code
if __name__ == "__main__":

    set_screen()
    robot = EV3Robot()
    
    for i in range(30):
        start_time  = time.time()
        condition = time_condition(start_time)

        while(condition):
    
            # color is Red, goal reached
            if robot.goal_reached():
                break
            
            if robot.reaching_an_object():
                robot.drive_back()
                robot.turn_back_with_random_angle()
            
            robot.step_action()
            condition = time_condition(start_time)

        debug_print_run_time(i, start_time)
        write_results_to_file(i, start_time)
        time.sleep(10)

        


        
    
