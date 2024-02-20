#!/usr/bin/env python3

from screen_utils import *
from ev3control import *
import threading



# def monitor_touch_sensor(robot):
#     while True:
#         if robot.read_touch_sensor():
#             print("Touch sensor pressed!")
#             break
#     robot.stop_drive()


# This allows the module to be imported without executing any code
if __name__ == "__main__":

    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')
    robot = EV3Robot()

    # sensor_thread = threading.Thread(target=monitor_touch_sensor, args=(robot,))
    # sensor_thread.start()
    # robot.drive(50, 50)  # Drive the robot while the sensor is being monitored

    # sensor_thread.join()  # Wait for the touch sensor thread to finish

    # robot.drive_with_time_suspension(duration=2, read_data=1)
    
    # while not robot.read_touch_sensor():
    #     pass
    # robot.stop()
    # while not robot.read_distance() <= 3:
    #     pass



    # robot.turn_right(50)
    # debug_print("turned right")
    # robot.drive_with_time_suspension(duration=0.5,read_data=1)
    # robot.turn_left(50)
    # debug_print("turned left")
    # robot.drive_with_time_suspension(duration=0.5, read_data=1)
    # robot.turn_right(50)
    # robot.turn_right(50)
    # debug_print("turned back")
    # time.sleep(5)


    while(True):
        # color is Red, goal reached
        if robot.read_color_sensor() == 5:
            break

        state = robot.get_current_state()
        next_action = robot.get_next_action(state)
        robot.execute_action(next_action)

    
