#import turn_aside
import sys
import time
import random
import numpy as np
import pandas as pd
from controller import Supervisor, Robot, DistanceSensor, Motor, TouchSensor, Camera
from colorthief import ColorThief
import cv2  # Import OpenCV
import os
import json

class ev:

    timeStep = 32
    maxSpeed = 10.0
    motors = []
    velocities  = []

    def __init__(self,robot,ds, touch_sensor, camera,left_motor,right_motor,supervisor):
        self.supervisor = supervisor
        self.robot = robot
        self.camera = camera
        self.touch_sensor =  touch_sensor
        self.distance_sensor = ds

        self.motors.append(left_motor)
        self.motors.append(right_motor)
        
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))


    def move_l_motor(self, speed):
        speed_l = self.boundSpeed(speed)
        self.motors[0].setVelocity(speed_l)


    def move_r_motor(self, speed):
        speed_r = self.boundSpeed(speed)
        self.motors[1].setVelocity(speed_r)
    

    def break_l_motor(self):
        self.motors[0].setVelocity(0.0)

    def break_r_motor(self):
        self.motors[1].setVelocity(0.0)


    def break_motors(self):
        self.break_l_motor()
        self.break_r_motor()


    def drive_motors(self,speeds):
        # case speed is not a list, but a number for both
        if type(speeds) == int or type(speeds) == float:
            speed   = float(speeds)
            self.move_l_motor(speed)
            self.move_r_motor(speed)
        
        if type(speeds) == list:
        # case its a list
            self.move_l_motor(speeds[0])
            self.move_r_motor(speeds[1])
    
        else:
            return
    

    def wait(self, time_seconds):
        num_steps = int(time_seconds * 1000 / self.timeStep)  # convert time to simulation steps
        for _ in range(num_steps):
            if self.supervisor.step(self.timeStep) == -1:  # simulation ended
                return
        
    def move_for_time(self, speeds, time_seconds):
        self.drive_motors(speeds)    
        self.wait(time_seconds)
        self.break_motors()
        

    def change_direction(self, direction):
        if direction == 'left':
            self.move_for_time([0.0,self.maxSpeed], 0.5) # turn left
        elif direction == 'right':
            self.move_for_time([self.maxSpeed,0.0], 0.5) # turn right
        else:
            print("Invalid direction. Please specify 'left' or 'right'.")
    
    # a function to make the robot to move in reverse
    def move_reverse(self):
        self.move_for_time([-self.maxSpeed,-self.maxSpeed], 1)
        
    def move_mini_reverse(self):
        self.move_for_time([-self.maxSpeed,-self.maxSpeed], 0.5)
            
    # a function to make the robot to turn 180 degree using change direction function
    def turn_180(self):
        self.change_direction('left')
        self.change_direction('left')
        self.change_direction('left')
        self.change_direction('left')
        self.break_motors()


# State and action spaces
NUM_TOUCH_STATES = 2
NUM_DISTANCE_STATES = 256
NUM_COLOR_STATES = 8
NUM_ACTIONS = 3

NUM_STATES = NUM_TOUCH_STATES * NUM_DISTANCE_STATES * NUM_COLOR_STATES

MAX_MOVEs = 40  # Maximum number of moves per episode
TIME_STEP = 32

def reset_robot_position(supervisor):

    # Get the robot node by its name
    robot_node = supervisor.getFromDef("ROBOT1")

    # Set the initial position of the robot to (x=0, y=0, z=0)
    robot_node.getField("translation").setSFVec3f([0.0, 0.0, 0.0])
    
    # Set the initial rotation of the robot to (x=0, y=0, z=1,rad=0)
    robot_node.getField("rotation").setSFRotation([0, 0, 1, 0])
    
    left_wheel = supervisor.getFromDef("LEFT_WHEEL")

    # Set the initial position of the robot to (x=0, y=0, z=0)
    left_wheel.getField("translation").setSFVec3f([0.0, 0.045, 0.025])
    
    # Set the initial rotation of the robot to (x=0, y=0, z=1,rad=0)
    left_wheel.getField("rotation").setSFRotation([0, 1, 0, 0.215])
    
    right_wheel = supervisor.getFromDef("RIGHT_WHEEL")

    # Set the initial position of the robot to (x=0, y=0, z=0)
    right_wheel.getField("translation").setSFVec3f([0.0, -0.045, 0.025])
    
    # Set the initial rotation of the robot to (x=0, y=0, z=1,rad=0)
    right_wheel.getField("rotation").setSFRotation([0, -1, 0, 0.215])

    # Get the floor nodes by thier name
    red_square = supervisor.getFromDef("redendpoint")
    blue_square = supervisor.getFromDef("Blue")
    green_square = supervisor.getFromDef("Green")
    black_square = supervisor.getFromDef("Black")
    white_square = supervisor.getFromDef("White")
    yellow_square = supervisor.getFromDef("Yellow")
    brown_square = supervisor.getFromDef("Brown")
    
    # Set the range of the floor positions
    min_x, max_x = -0.6, 0.6
    min_z, max_z = -0.6, 0.6

    # Generate random positions for the floor within the floor range
    x = random.uniform(min_x, max_x)
    z = random.uniform(min_z, max_z)

    # Set the initial position of the floors
    red_square.getField("translation").setSFVec3f([x, z , -0.04])  # Set a small elevation to place it on the floor
    blue_square.getField("translation").setSFVec3f([x-0.5, z , -0.04])
    green_square.getField("translation").setSFVec3f([x-0.5, z-0.5 , -0.04])
    black_square.getField("translation").setSFVec3f([x-0.5, z+0.5 , -0.04])
    white_square.getField("translation").setSFVec3f([x+0.5, z-0.5 , -0.04])
    yellow_square.getField("translation").setSFVec3f([x+0.5, z , -0.04])
    brown_square.getField("translation").setSFVec3f([x+0.5, z+0.5 , -0.04])

def get_state(touch_sensor, ds, camera):
    # Modify the state representation based on sensor values
    # You can discretize or normalize the values as needed
    if (touch_sensor.getValue()!=0):
        touch_value = 1
    else :
        touch_value = 0
    distance_value = int(ds.getValue()*100)
    color_value = get_color(camera)
    state = [touch_sensor.getValue(), distance_value, color_value]
    return state

def get_color(camera):
    # Get camera image
    camera_image = camera.getImage()
    
    # Convert camera image to numpy array
    image_array = np.frombuffer(camera_image, np.uint8).reshape((32, 32, 4))
    
    # Create a temporary image file for ColorThief to read
    temp_image_path = "temp_image.png"
    cv2.imwrite(temp_image_path, cv2.cvtColor(image_array, cv2.COLOR_RGBA2BGR))
    
    # Use ColorThief to get the dominant color
    color_thief = ColorThief(temp_image_path)
    dominant_color = color_thief.get_color(quality=1)
    # Delete the temporary image file
    os.remove(temp_image_path)
        
    # Determine the color index based on the dominant color
    color_index = get_color_index(*dominant_color)
    
    return color_index


def get_color_index(b, g, r):
    # Map RGB values to color indices based on thresholds
    if r > 200 and g > 200 and b > 200:  # White
        return 6
    elif r < 50 and g > 100 and b < 50:  # Green
        return 3
    elif r > 150 and g < 100 and b < 100:  # Red
        return 5
    elif r < 50 and g < 50 and b < 50:  # Black
        return 1
    elif r > 150 and g > 100 and b > 50:  # Brown
        return 7
    elif r > 200 and g > 200 and b < 100:  # Yellow
        return 4
    elif r < 50 and g < 100 and b > 150:  # Blue
        return 2
    else:  # No Color
        return 0



def train(robot,redfloor, ds, touch_sensor, camera, supervisor,NUM_EPISODEs,q_table,EPSILOn,ALPHa, GAMMa,collision_reward,goal_reward):
    won = 0
    stuck = 0
    failcount =0 
    decay_rate = 0.001  # Decay rate for the exploration rate
    min_epsilon = 0.01  # Minimum exploration rate
    max_epsilon = 1.0  # Maximum exploration rate
    epsilon_var = EPSILOn
    left_motor = supervisor.getDevice("left wheel motor")
    right_motor = supervisor.getDevice("right wheel motor")
    robot_controller= ev(robot,ds, touch_sensor, camera,left_motor,right_motor,supervisor)

    for episode in range(NUM_EPISODEs):
        if episode % 100 == 0:
            print("Episode:", episode)

        # Reset the robot's position
        reset_robot_position(supervisor)
        supervisor.step(TIME_STEP)
        # Initialize sensor values
        state = get_state(touch_sensor, ds, camera)
        stucked = False
        moves = 0
        end = False
        reward = 0
        while moves < MAX_MOVEs and not stucked:
            # Choose an action using epsilon-greedy policy
            rand = np.random.uniform()
            if rand < epsilon_var:
                action = np.random.randint(NUM_ACTIONS)  # Exploration
            else:
                action = q_table.loc[state[0],state[1],state[2]].argmax()  # Exploitation
            if(state[1]<10):
                stuck+=1
                stucked = True
            # Perform the chosen action
            if action == 0:
                robot_controller.move_for_time(10.0, 1)                
            elif action == 1:
                # Turn left
                robot_controller.change_direction("left")
            elif action == 2:
                # Turn right
                robot_controller.change_direction("right")

            # Wait for the next time step
            supervisor.step(TIME_STEP)
            # Update sensor values and get the next state
            next_state = get_state(touch_sensor, ds, camera)

            # Check if the episode has ended (e.g., reaching the red floor)
            if next_state[2] == 5:  
                reward = goal_reward  # Modify the reward calculation based on your task
                won+=1
                end = True     

            robot_node = robot
            robot_position = robot_node.getPosition()
            box_position = redfloor.getPosition()
            distance = np.linalg.norm(np.array(box_position) - np.array(robot_position))
            reward =  -1  # Modify the reward calculation based on your task

            # Penalize collisions
            if state[0] > 0 or state[1]<10 and not end:
                reward = collision_reward
            #print("simple", Q.loc[tuple(state), action])
            
            # Update the Q-table using the Q-learning equation
            q_table.loc[tuple(state), action] = (1-ALPHa) *q_table.loc[tuple(state), action]+ ALPHa*(reward + GAMMa * q_table.loc[tuple(next_state)].argmax())
            if end:
                break
            
            # Update the current state
            state = next_state

            moves += 1

        if moves >= MAX_MOVEs:
            failcount+=1
        # Save Q-table
        np.save('q_table.npy', q_table)
        # Decay the exploration rate
        epsilon_var = min_epsilon + (max_epsilon - min_epsilon) * np.exp(-decay_rate * episode)
    
    Q_dict = {}
    for i_touch in range(NUM_TOUCH_STATES):
        for i_distance in range(NUM_DISTANCE_STATES):
            for i_color in range(NUM_COLOR_STATES):
                action = q_table.loc[i_touch,i_distance,i_color].argmax()
                Q_dict[(i_touch, i_distance, i_color)] = action
    
   # Convert int64 values to Python int
    Q_dict_converted = {','.join(map(str, key)): int(value) if isinstance(value, np.int64) else value for key, value in Q_dict.items()}

    # Write the JSON data to a file
    with open('QTable.json', 'w') as json_file:
        json.dump(Q_dict_converted, json_file)       
    return (won,failcount,epsilon_var)
    
def check(robot,redfloor, ds, touch_sensor, camera, supervisor,NUM_EPISODEs):
    won = 0
    stuck = 0
    failcount = 0
    left_motor = supervisor.getDevice("left wheel motor")
    right_motor = supervisor.getDevice("right wheel motor")
    robot_controller= ev(robot,ds, touch_sensor, camera,left_motor,right_motor,supervisor)
    # Load the JSON data from the file
    with open('QTable.json', 'r') as json_file:
        Q_dict_str_keys = json.load(json_file)

    # Convert the keys back to tuples and convert int64 values to Python int
    Q = {tuple(map(int, key.split(','))): int(value) if isinstance(value, int) else value for key, value in Q_dict_str_keys.items()}

    for episode in range(NUM_EPISODEs):
        # Reset the robot's position
        reset_robot_position(supervisor)
        supervisor.step(TIME_STEP)
        # Initialize sensor values
        state = get_state(touch_sensor, ds, camera)
        stucked = False
        moves = 0
        while moves < MAX_MOVEs and not stucked:
            # Choose an action using epsilon-greedy policy
            action = Q[(state[0],state[1],state[2])] # Exploitation
            if(state[1]<10):
                #print("Episode:", episode, " Got Stuck")
                stuck+=1
                #turn around
                robot_controller.move_reverse()
                robot_controller.turn_180()
                
            # Perform the chosen action
            if action == 0:
                # Move forward
                robot_controller.move_for_time(10.0, 1)                
            elif action == 1:
                # Turn left
                robot_controller.change_direction("left")
            elif action == 2:
                # Turn right
                robot_controller.change_direction("right")

            # Wait for the next time step
            supervisor.step(TIME_STEP)
            # Update sensor values and get the next state
            next_state = get_state(touch_sensor, ds, camera)

            # Check if the episode has ended (e.g., reaching the red floor)
            if next_state[2] == 5:  
                won+=1     
                break
            
            # Update the current state
            state = next_state
            moves += 1

        if moves >= MAX_MOVEs:
            failcount+=1
            #print("Episode:", episode, "Maximum moves reached")
    return (won,failcount)

def main():
    # Create a supervisor instance
    supervisor = Supervisor()
    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)

    # Get the robot node by its name
    robot = supervisor.getFromDef("ROBOT1")
    #robot = ev()

    # Get the floor node by its name
    redfloor = supervisor.getFromDef("redendpoint")

    # Get the distance sensor device
    ds = supervisor.getDevice("distance")
    ds.enable(TIME_STEP)

    # Get the touch sensor device
    touch_sensor = supervisor.getDevice("touch sensor")
    touch_sensor.enable(TIME_STEP)
    
    # Get the camera device
    camera = supervisor.getDevice("Color")
    camera.enable(TIME_STEP)

    # Q-learning parameters
    EPSILOn = 1  # Exploration rate
    ALPHa = 0.1  # Learning rate
    GAMMa = 0.99  # Discount factor
    NUM_EPISODEs = 1000  # Number of training episodes
    MAX_MOVEs = 50
    result_dict = {}
    # Create a multi-index for the Q-table using pandas
    index = pd.MultiIndex.from_product(
        [range(NUM_TOUCH_STATES), range(NUM_DISTANCE_STATES), range(NUM_COLOR_STATES)],
        names=['touch', 'distance', 'color']
    )
    collision_reward = -10
    goal_reward = 100
    q_table = pd.DataFrame(index=index, columns=range(NUM_ACTIONS)).fillna(0.0)
    won,stuck,EPSILOn = train(robot,redfloor, ds, touch_sensor, camera, supervisor,NUM_EPISODEs,q_table,EPSILOn,ALPHa, GAMMa,collision_reward,goal_reward)
    print("won:" +str(won)+"fail:"+str(stuck) +" with NUM_EPISODES = "+str(NUM_EPISODEs)+"EPSILON = "+str(EPSILOn) + "ALPHA = "+str(ALPHa) + "GAMMA = "+str(GAMMa) + "COLLISION_REWARD = "+str(collision_reward) + "GOAL_REWARD = "+str(goal_reward))
    won,stuck = check(robot,redfloor, ds, touch_sensor, camera, supervisor,100)
    print("won:" +str(won)+"fail:"+str(stuck) +" with NUM_EPISODES = "+str(100))
    result_dict[NUM_EPISODEs] = (won,stuck,(won/(won+stuck)*100),ALPHa,GAMMa,EPSILOn)
        
    # print result
    print(result_dict)
    exit(1);
    
if __name__ == "__main__":
    main()





            
