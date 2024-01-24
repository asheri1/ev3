from screen_utils import *
from ev3control import *
import _thread


# This allows the module to be imported without executing any code
if __name__ == "__main__":

    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')
    robot = EV3Robot()
    
    robot.drive_with_time_suspension(3)
    
    # while not robot.read_touch_sensor():
    #     pass
    # robot.stop()
    # while not robot.read_distance() <= 3:
    #     pass

    robot.turn_right(20)
    print("turned right")
    robot.drive_with_time_suspension(duration=2)
    robot.turn_left(50)
    print("turned left")
    robot.drive_with_time_suspension(duration=2)
    robot.turn_right(50)
    robot.turn_right(50)
    print("turned back")
    time.sleep(5)
    print("Color:", robot.read_color_sensor())
