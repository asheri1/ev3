from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds

class EV3Robot:
    def __init__(self):
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

    def stop(self):
        # Stop the robot
        self.tank_drive.off()

    def read_touch_sensor(self):
        # Read the touch sensor
        return self.touch_sensor.is_pressed

    def read_color_sensor(self):
        # Read the color sensor
        return self.color_sensor.color

    def read_distance(self):
        # Read the ultrasonic sensor
        return self.ultrasonic_sensor.distance_centimeters

    def set_led(self, color):
        # Set LED color
        self.leds.set_color("LEFT", color)
        self.leds.set_color("RIGHT", color)

# This allows the module to be imported without executing any code
if __name__ == "__main__":
    robot = EV3Robot()
    # Example usage
    robot.drive(50, 50)
    while not robot.read_touch_sensor():
        pass
    robot.stop()
    print("Color:", robot.read_color_sensor())
    print("Distance:", robot.read_distance())
