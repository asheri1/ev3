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
