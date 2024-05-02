import pygame
import math
import random
import numpy
import csv
import os  # Import the os module
#from matplotlib.animation import FuncAnimation
#import matplotlib.pyplot as plt

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FOREARM_LENGTH = 100
HAND_LENGTH = 100

TOUCH_SENSOR_GRID_SIZE = 3  # Number of touch sensors in each dimension
PROPRIO_SENSOR_NUMBER = 5  # Number of proprio sensors for each angle.


# Define the body square
BODY_X = 50  # X-coordinate of the top-left corner of the body square
BODY_Y = -100  # Y-coordinate of the top-left corner of the body square
BODY_SIZE = 150 # Side length of the body square #its actually a bit larger because of the spacing
TOUCH_SENSOR_SPACING = BODY_SIZE / TOUCH_SENSOR_GRID_SIZE

# Initialize arm angles
theta1 = 0  # Angle of the forearm (in radians)
theta2 = 0  # Angle of the hand (in radians)

# Initialize random angle trajectory parameters
min_angle1 = -math.pi/2   # Minimum angle in radians
max_angle1 = math.pi/2   # Maximum angle in radians
min_angle2 = -math.pi   # Minimum angle in radians
max_angle2 = math.pi   # Maximum angle in radians
angle_step = 0.05         # Angle step for animation

# Define population code receptive field
#tactile receptive field
RECEPTIVE_FIELD_SIZE_TOUCH = BODY_SIZE/3

#proprio receptive field
PROPRIO_SENSOR_GRID_SIZE_2 = (max_angle2 - min_angle2) / (PROPRIO_SENSOR_NUMBER - 1) # distance between centers of receptive field
PROPRIO_SENSOR_GRID_SIZE_1 = (max_angle1 - min_angle1) / (PROPRIO_SENSOR_NUMBER - 1) #
RECEPTIVE_FIELD_SIZE_PROPRIO_1 = PROPRIO_SENSOR_GRID_SIZE_1 * .6 #receptive field size
RECEPTIVE_FIELD_SIZE_PROPRIO_2 = PROPRIO_SENSOR_GRID_SIZE_2 * .6 #receptive field size

# simulation length
max_iteration = 1e5

# interface
visual = 1 #show the simulated agent
verbose = 0 #print the sm code
store_csv = 1 #store data

# Store data
data = []

if visual:
    # Colors
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)

    # Initialize the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("2D Serial Arm Simulation")

    # Create a quit button rectangle
    quit_button_rect = pygame.Rect(10, 10, 100, 50)  # (x, y, width, height)
    quit_button_color = (255, 0, 0)  # Red color for the button


def calculate_touch_sensors(end_effector_x, end_effector_y):
    touch_sensors = []
    for i in range(TOUCH_SENSOR_GRID_SIZE):
        for j in range(TOUCH_SENSOR_GRID_SIZE):
            x = i * TOUCH_SENSOR_SPACING + BODY_X
            y = j * TOUCH_SENSOR_SPACING + BODY_Y
            distance = math.hypot(end_effector_x - x, end_effector_y - y)
            if BODY_X - TOUCH_SENSOR_SPACING/2 <= end_effector_x <= BODY_X + BODY_SIZE - TOUCH_SENSOR_SPACING/2 and BODY_Y - TOUCH_SENSOR_SPACING/2 <= end_effector_y <= BODY_Y + BODY_SIZE - TOUCH_SENSOR_SPACING/2:
                touch_sensors.append(distance)
            else:
                touch_sensors.append(float('inf'))  # Set to infinity when outside the body square
    return touch_sensors

def calculate_sm_activations(angle1,angle2,touch_sensors):
    #tactile pop code
    touch_activation = [numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_TOUCH) for distance in touch_sensors]
    #population code of proprio
    proprio_code = []

    #first part of prioprio vector is angle coding from shoulder angle
    for i in range(PROPRIO_SENSOR_NUMBER):
        x = i * PROPRIO_SENSOR_GRID_SIZE_1 + min_angle1
        distance = numpy.abs(angle1 - x)
        activation = numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_PROPRIO_1)
        proprio_code.append(activation)
    #second part of proprio vector is angle coding from elbow angle
    for i in range(PROPRIO_SENSOR_NUMBER):
        x = i * PROPRIO_SENSOR_GRID_SIZE_2 + min_angle2
        distance = numpy.abs(angle2 - x)
        activation = numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_PROPRIO_2)
        proprio_code.append(activation)

    return proprio_code,touch_activation
for iteration_counter in range(int(max_iteration)):
    if iteration_counter % numpy.round(max_iteration // 100) == 0:
        percentage_completed = (iteration_counter / max_iteration) * 100
        print(f"{percentage_completed:.1f}% completed")

    # Generate random but continuous angle trajectory
    theta1 += random.uniform(-angle_step, angle_step)
    theta2 += random.uniform(-angle_step, angle_step)

    # Clamp angles within the specified range
    theta1 = max(min(theta1, max_angle1), min_angle1)
    theta2 = max(min(theta2, max_angle2), min_angle2)

    # Calculate end-effector position (tip of the hand segment)
    end_effector_x = FOREARM_LENGTH * math.cos(theta1) + HAND_LENGTH * math.cos(theta1 + theta2)
    end_effector_y = FOREARM_LENGTH * math.sin(theta1) + HAND_LENGTH * math.sin(theta1 + theta2)

    # Calculate touch sensors
    touch_sensors = calculate_touch_sensors(end_effector_x, end_effector_y)
    [proprio_code, touch_activation] = calculate_sm_activations(theta1, theta2, touch_sensors)

    # store data
    data.append(proprio_code + touch_activation)
    # some verbose
    if verbose:
        print('touch')
        print(touch_activation)
        print('proprio')
        print(proprio_code)

    if visual:
        # Draw everything
        screen.fill(WHITE)

        # Draw body square
        pygame.draw.rect(screen, RED, (BODY_X - TOUCH_SENSOR_SPACING/2 + SCREEN_WIDTH // 2, BODY_Y - TOUCH_SENSOR_SPACING/2 + SCREEN_HEIGHT // 2, BODY_SIZE, BODY_SIZE), 2)

        # Draw forearm (from the body to the elbow)
        elbow_x = FOREARM_LENGTH * math.cos(theta1)
        elbow_y = FOREARM_LENGTH * math.sin(theta1)
        pygame.draw.line(screen, RED, (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2), (elbow_x+ SCREEN_WIDTH // 2, elbow_y+ SCREEN_HEIGHT // 2), 5)

        # Draw hand (from the elbow to the end-effector)
        pygame.draw.line(screen, RED, (elbow_x + SCREEN_WIDTH // 2, elbow_y + SCREEN_HEIGHT // 2), (end_effector_x+ SCREEN_WIDTH // 2, end_effector_y+ SCREEN_HEIGHT // 2), 5)

        # Draw finger
        end_effector_screen_x = end_effector_x + SCREEN_WIDTH // 2
        end_effector_screen_y = end_effector_y + SCREEN_HEIGHT // 2
        pygame.draw.circle(screen, (0, 0, 255), (int(end_effector_screen_x), int(end_effector_screen_y)), 5)

        # Draw touch sensors
        for i in range(TOUCH_SENSOR_GRID_SIZE):
            for j in range(TOUCH_SENSOR_GRID_SIZE):
                x = i * TOUCH_SENSOR_SPACING + BODY_X + SCREEN_WIDTH // 2
                y = j * TOUCH_SENSOR_SPACING + BODY_Y + SCREEN_HEIGHT // 2
                distance = touch_sensors[i * TOUCH_SENSOR_GRID_SIZE + j]
                if distance < BODY_SIZE/3:
                    pygame.draw.circle(screen, RED, (int(x), int(y)), 3)
                else:
                    pygame.draw.circle(screen, (125,125,125), (int(x), int(y)), 3)

        # Draw the iteration counter on the screen
        pygame.draw.rect(screen, quit_button_color, quit_button_rect)
        font = pygame.font.Font(None, 36)
        quit_text = font.render("Quit", True, (255, 255, 255))
        screen.blit(quit_text, (10, 10))
        #counter_text = font.render(f"Iteration: {iteration_counter}", True, (0, 0, 0))
        #screen.blit(counter_text, (10, 10))

        pygame.display.flip()

        #check for stopping the loop
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                break

            # check for mouse click events
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    if quit_button_rect.collidepoint(event.pos):  # Check if the mouse click is inside the quit button
                        pygame.quit()  # Quit the application if the button is clicked
                        break
        else: #this is so crazy code
            continue
        break
if visual:
    pygame.quit()

if store_csv:
    # Define the CSV file path
    csv_file_path = os.path.expanduser(os.path.join(os.getcwd(),"pygame_data.csv"))
    # Write data to the CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write header row (you can customize this based on your data)
        header = ["Proprio_Code_" + str(i) for i in range(10)] + ["Touch_Activation_" + str(i) for i in range(9)]
        writer.writerow(header)

        # Write data rows
        for row in data:
            writer.writerow(row)

    print("Data saved to", csv_file_path)
