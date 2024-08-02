"""Maze_Left controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
import cv2
import numpy as np

def color_detection(image):
    # Define color thresholds in RGB format
    red_lower = np.array([0, 0, 100], dtype=np.uint8)
    red_upper = np.array([50, 50, 255], dtype=np.uint8)
    green_lower = np.array([0, 100, 0], dtype=np.uint8)
    green_upper = np.array([50, 255, 50], dtype=np.uint8)
    blue_lower = np.array([100, 0, 0], dtype=np.uint8)
    blue_upper = np.array([255, 50, 50], dtype=np.uint8)
    yellow_lower = np.array([0, 100, 100], dtype=np.uint8)
    yellow_upper = np.array([50, 255, 255], dtype=np.uint8)
   
    # Threshold the image to get only desired colors
    red_mask = cv2.inRange(image, red_lower, red_upper)
    green_mask = cv2.inRange(image, green_lower, green_upper)
    blue_mask = cv2.inRange(image, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(image, yellow_lower, yellow_upper)
    
    # Bitwise-AND mask and original image
    red_result = cv2.bitwise_and(image, image, mask=red_mask)
    green_result = cv2.bitwise_and(image, image, mask=green_mask)
    blue_result = cv2.bitwise_and(image, image, mask=blue_mask)
    yellow_result = cv2.bitwise_and(image, image, mask=yellow_mask)
    
    # Normalize the color arrays to range from 0 to 1
    red_array = red_result.astype(np.float32) / 255.0
    green_array = green_result.astype(np.float32) / 255.0
    blue_array = blue_result.astype(np.float32) / 255.0
    yellow_array = yellow_result.astype(np.float32) / 255.0

    # Check which color has the most pixels
    red_count = cv2.countNonZero(red_mask)
    green_count = cv2.countNonZero(green_mask)
    blue_count = cv2.countNonZero(blue_mask)
    yellow_count = cv2.countNonZero(yellow_mask)
    
    # Determine the dominant color
    if red_count > green_count and red_count > blue_count:
        return [1, 0, 0]  # Red
    elif green_count > red_count and green_count > blue_count:
        return [0, 1, 0]  # Green
    elif blue_count > red_count and blue_count > green_count:
        return [0, 0, 1]  # Blue
    elif yellow_count >= green_count and yellow_count >= red_count and yellow_count > blue_count:
        return [1, 1, 0]
    else:
        return "No dominant color detected"



def run_robot(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    max_speed = 6.28
    speeds = [max_speed, max_speed]
    
    # Enable the motors
    motors = []
    left_motor = robot.getMotor('motor_1')
    right_motor = robot.getMotor('motor_2')
    motors.append(left_motor)
    motors.append(right_motor)
    for i in range(2):
        motors[i].setPosition(float('inf'))
        motors[i].setVelocity(0.0)
    
    # Enable Distance Sensors    
    sensors = []
    for i in range(5):
        sensor = robot.getDevice(f'ds{i}')
        sensors.append(sensor)
        sensors[i].enable(timestep)
        
    # Enable the camera
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    cam = robot.getDevice("cam")
    cam.enable(timestep)
    
    # Define the colors of the balls
    ball_colors = ["red", "blue", "green", "yellow"]

    # Initialize variables
    current_ball_index = 0
    balls_found = []

    # Set robot's speed
    max_speed = 6.28

    n = 0;
    m = 0;
    R = 1;
        
    # Main loop:    
    while robot.step(timestep) != -1:
    
        # Get camera image
        image = camera.getImageArray()
        imageI = cam.getImageArray()
        col = "None"
        
        if image:
            # Convert image to numpy array
            image = np.array(image, dtype=np.uint8)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # Perform color detection
            detected_color = color_detection(image)
            if len(balls_found) < len(ball_colors):
                if (detected_color == [1, 0, 0]) and (ball_colors[current_ball_index] == "red"):# and (front_wall):
                    balls_found.append("red")
                    current_ball_index += 1
                    col = "Red"
                    print("First Ball (Red) Found!")
                elif (detected_color == [0, 0, 1]) and (ball_colors[current_ball_index] == "blue"):# and (front_wall):
                    balls_found.append("blue")
                    current_ball_index += 1
                    col = "Blue"
                    print("Second Ball (Blue) Found!")
                elif (detected_color == [0, 1, 0]) and (ball_colors[current_ball_index] == "green"):# and (front_wall):
                    balls_found.append("green")
                    current_ball_index += 1
                    col = "Green"
                    print("Third Ball (Green) Found!")
                elif (detected_color == [1, 1, 0]) and (ball_colors[current_ball_index] == "yellow"):
                    balls_found.append("yellow")
                    current_ball_index += 1
                    col = "yellow"
                    print("Fourth Ball (yellow) Found!")
           
            #print("Detected color:", col)
            # Display the image
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)  # Keep the window open until a key is pressed            

        # Check if all balls have been found
        if len(balls_found) == len(ball_colors):
            print("All four balls found!")
    
        # Read the sensors.
        #for ind in range(5):
            #print("ind: {}, val: {}".format(ind, sensors[ind].getValue()))
                
        # Process sensors data.
        front_wall = sensors[2].getValue() > 200
        front_wall_r = sensors[3].getValue() > 400
        front_wall_l = sensors[4].getValue() > 400
        left_wall = sensors[0].getValue() > 200
        right_wall = sensors[1].getValue() > 200
        
        if (front_wall) or ((n>0) and (n<35)):
            front_wall = True
            n = n+1
            #print(n)
        elif n >= 35:
            n = 0
        
        if front_wall:
            print("Turn Right in Place")
            speeds[0] = max_speed
            speeds[1] = -max_speed
            #print(front_wall)
            print(balls_found)
  
        else:
            if left_wall:
                print("Drive Forward")
                speeds[0] = max_speed
                speeds[1] = max_speed
            else:
                print("Turn Left")
                speeds[0] = max_speed/3.5
                speeds[1] = max_speed
        
        if imageI:
            imageI = np.array(imageI, dtype=np.uint8)
            imageI = cv2.cvtColor(imageI, cv2.COLOR_RGB2BGR)
            
            # Perform color detection
            detected_colorI = color_detection(imageI)
        
            #print(detected_colorI)
            if (detected_colorI == [1, 0, 0]):
                if len(balls_found) == len(ball_colors):
                    print("GOAL END REACHED !")
                    for i in range(2):
                        speeds[0] = 0.0
                        speeds[1] = 0.0
                else:
                    front_wall = True
                    n = n+1
            elif (detected_colorI == [0, 0, 1]):
                front_wall = True
                n = n+1
                        
            # Display the image
            #cv2.imshow("Camera Image", imageI)
            #cv2.waitKey(1)  # Keep the window open until a key is pressed            

        
        for i in range(2):
            motors[i].setVelocity(speeds[i])
        if (detected_colorI == [1, 0, 0]) and (speeds[0] == 0) and (speeds[1] == 0):
            break
            

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)


