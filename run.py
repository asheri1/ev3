from ev3control import EV3Robot

robot = EV3Robot()

distance = robot.read_distance()
print("Distance to object:", distance, "cm")
