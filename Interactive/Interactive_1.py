
import pygame
import math
import numpy
import csv
import os  # Import the os module
import numpy as np
# import time
import pygame_chart as pyc
import Functions as FN
from Create_Neurons import *
from Create_Synapses import *
from Create_Monitors import *
#from matplotlib.animation import FuncAnimation
#import matplotlib.pyplot as plt


# Initialize Pygame
pygame.init()

# Visualization Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FOREARM_LENGTH = 100
HAND_LENGTH = 100

TOUCH_SENSOR_GRID_SIZE = 3  # Number of touch sensors in each dimension
PROPRIO_SENSOR_NUMBER = 10  # Number of proprio sensors for each angle.


# Define the body square
BODY_X = 50  # X-coordinate of the top-left corner of the body square
BODY_Y = -100  # Y-coordinate of the top-left corner of the body square
BODY_SIZE = 150 # Side length of the body square #its actually a bit larger because of the spacing
TOUCH_SENSOR_SPACING = BODY_SIZE / TOUCH_SENSOR_GRID_SIZE


# Define constants for the heatmap
CELL_SIZE = 30
NUM_ROWS = 3    #SCREEN_HEIGHT // CELL_SIZE
NUM_COLS = 3    #SCREEN_WIDTH // CELL_SIZE

# Initialize random angle trajectory parameters
min_angle1 = -math.pi/3   # Minimum angle in radians
max_angle1 = math.pi/3   # Maximum angle in radians
min_angle2 = -math.pi   # Minimum angle in radians
max_angle2 = math.pi   # Maximum angle in radians
angle_step = 0.5         # Angle step for animation

# Define population code receptive field
#tactile receptive field
RECEPTIVE_FIELD_SIZE_TOUCH = BODY_SIZE/5

#   proprio receptive field
PROPRIO_SENSOR_GRID_SIZE_2 = (max_angle2 - min_angle2) / (PROPRIO_SENSOR_NUMBER - 1) # distance between centers of receptive field
PROPRIO_SENSOR_GRID_SIZE_1 = (max_angle1 - min_angle1) / (PROPRIO_SENSOR_NUMBER - 1) #
RECEPTIVE_FIELD_SIZE_PROPRIO_1 = PROPRIO_SENSOR_GRID_SIZE_1 * .6 #receptive field size
RECEPTIVE_FIELD_SIZE_PROPRIO_2 = PROPRIO_SENSOR_GRID_SIZE_2 * .6 #receptive field size

# Initialize Target Touch Activation
touch_activation_target = [0,0,0,0,0,0,0,0,0]

# Initialize arm angles
theta1 = -1  # Angle of the forearm (in radians)
theta2 = -1  # Angle of the hand (in radians)

# Initialize random angle trajectory parameters
min_angle1 = -math.pi/3   # Minimum angle in radians
max_angle1 = math.pi/3   # Maximum angle in radians
min_angle2 = -math.pi   # Minimum angle in radians
max_angle2 = math.pi   # Maximum angle in radians
angle_step = 0.01         # Angle step for animation

x1 = 50
x2 = 150
x3 = 100
y1 = 400
y2 = 450
bar_width = 5
void_width = 1


# Define population code receptive field
#tactile receptive field
RECEPTIVE_FIELD_SIZE_TOUCH = BODY_SIZE/5

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
store_csv = 0 #store data

#Initialize SNN

#   Define Network Parameters
N_Prop_1 = 10   # Number of Proprioception Neurons for joint 1
N_Prop_2 = 10   # Number of Proprioception Neurons for joint 2
N_Tactile = 9   # Number of Tactile Neurons on Torso
N_GF = N_Prop_1 * N_Prop_2  # Number of Gain-FIeld Neurons for arm's Spatial Position
N_GF_M1 = N_Prop_1 * N_Prop_1  # Number of Gain-FIeld Neurons for Joint 1 reach command
N_GF_M2 = N_Prop_2 * N_Prop_2  # Number of Gain-FIeld Neurons for Joint 2 reach command
Run_Duration = 100 * ms        # Each Input Value's Run Duration

#   Create Neuron, Synapse, and Monitor 
Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, Tactile_Neurons, GF_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons = Create_Neurons(N_Prop_1,N_Prop_2, N_Tactile, N_GF, N_GF_M1, N_GF_M2)
Prop1_GF, Prop2_GF,GF_Prop1, GF_Prop2, GF_Tactile, Tactile_GF, Prop1_MGF, Prop2_MGF, TProp1_MGF, TProp2_MGF, MGF1_DM1, MGF2_DM2, GF_GF_Inh, GF_GF_Ex = Load_Synapses(Prop_Neurons_1, Prop_Neurons_2, Prop_Neurons_T1, Prop_Neurons_T2, GF_Neurons, Tactile_Neurons, MGF_Neurons_1, MGF_Neurons_2, Directional_M1_Neurons, Directional_M2_Neurons)
SP_Prop_1, SP_Prop_2, SP_Prop_T1, SP_Prop_T2, SP_GF, SP_Tactile, SP_M1, SP_M2 = Create_SP_Monitors(Prop_Neurons_1, Prop_Neurons_2,Prop_Neurons_T1, Prop_Neurons_T2, GF_Neurons, Tactile_Neurons,Directional_M1_Neurons,Directional_M2_Neurons)


##################################
# Prepare Input data for Network
##################################
Props_1 = np.zeros((N_Prop_1, 100))
Props_2 = np.zeros((N_Prop_2, 100))
Tactile = np.zeros((N_Tactile, 100))
Tac = np.zeros((N_Tactile, 100))


Tac = np.sum(Tactile, axis = 0)
# Tac[:] = 1
Tac = np.tile(Tac,(100,1))
Tac[:,:] = 0

##################################
# Prepare Timed Arrays for Network
##################################

# Proprioception Signals

I_Prop_1 = TimedArray(np.transpose(Props_1), dt=100 * ms) # This is not used in this simulation
I_Prop_2 = TimedArray(np.transpose(Props_2), dt=100 * ms) # This is not used in this simulation
# Tactile Signals
I_Tactile = TimedArray(np.transpose(Tactile), dt=100 * ms)
I_Tac = TimedArray(np.transpose(Tac), dt=100 * ms)

net1 = Network(collect())
net1.store()

# Dummy Variables to calculate neural activities in each Run Duration
delta_theta1 = np.zeros((2,1))
delta_theta2 = np.zeros((2,1))
curr_M1_count = np.array([0, 0])
curr_M2_count = np.array([0, 0])
prev_M1_count = np.array([0, 0])
prev_M2_count = np.array([0, 0])
curr_tar_p1 = np.zeros(N_Prop_1)
curr_tar_p2 = np.zeros(N_Prop_2)
prev_tar_p1 = np.zeros(N_Prop_1)
prev_tar_p2 = np.zeros(N_Prop_2)
curr_GF = np.zeros(N_GF)
prev_GF = np.zeros(N_GF)

# Store data
data = []
circles = []

#   Initialize Visualization
if visual:
    # Colors
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    # Initialize the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("2D Serial Arm Simulation")

    # Create a quit button rectangle
    quit_button_rect = pygame.Rect(10, 10, 100, 50)  # (x, y, width, height)
    quit_button_color = (255, 0, 0)  # Red color for the button
    figure1 = pyc.Figure(screen, 100, 400, 100, 100)
    
    #  Plots' Lables
    font1 = pygame.font.Font(None, 14)
    font2 = pygame.font.Font(None, 20)
    xlable_text = font1.render("neuron index", True, (0, 0, 0))
    ycur_text = font1.render("Current", True, (0, 0, 0))
    ytar_text = font1.render("Target", True, (0, 0, 0))
    joint1_text = font2.render("Joint 1", True, (0, 0, 0))
    joint2_text = font2.render("Joint 2", True, (0, 0, 0))
    CurAct_text = font2.render("Current Tactile Activation", True, (0, 0, 0))
    TarAct_text = font2.render("Target Tactile Activation", True, (0, 0, 0))
    GF_text = font2.render("Gain FIeld Neuron's Activation", True, (0, 0, 0))

def calculate_touch_sensors(end_effector_x, end_effector_y):
    touch_sensors = []
    for k in range(TOUCH_SENSOR_GRID_SIZE):
        for jj in range(TOUCH_SENSOR_GRID_SIZE):
            x = k * TOUCH_SENSOR_SPACING + BODY_X
            y = jj * TOUCH_SENSOR_SPACING + BODY_Y
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
    for k in range(PROPRIO_SENSOR_NUMBER):
        x = k * PROPRIO_SENSOR_GRID_SIZE_1 + min_angle1
        distance = numpy.abs(angle1 - x)
        activation = numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_PROPRIO_1)
        proprio_code.append(activation)
    #second part of proprio vector is angle coding from elbow angle
    for k in range(PROPRIO_SENSOR_NUMBER):
        x = k * PROPRIO_SENSOR_GRID_SIZE_2 + min_angle2
        distance = numpy.abs(angle2 - x)
        activation = numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_PROPRIO_2)
        proprio_code.append(activation)

    return proprio_code,touch_activation


for iteration_counter in range(int(max_iteration)):#range((columns.shape)[1])):

    # Calculate end-effector position (tip of the hand segment)
    end_effector_x = FOREARM_LENGTH * math.cos(theta1) + HAND_LENGTH * math.cos(theta1 + theta2)
    end_effector_y = FOREARM_LENGTH * math.sin(theta1) + HAND_LENGTH * math.sin(theta1 + theta2)
    # print(end_effector_x, end_effector_y)

    # Calculate touch sensors
    touch_sensors = calculate_touch_sensors(end_effector_x, end_effector_y)
    [proprio_code, touch_activation] = calculate_sm_activations(theta1, theta2, touch_sensors)
    

    ###### Run The simulation and retrive data

    Prop_Neurons_1.I_P1[:]=np.squeeze(proprio_code[:10])       # Proprioceptive Input values of joint 1 for this simulation
    Prop_Neurons_2.I_P1[:]=np.squeeze(proprio_code[10:])       # Proprioceptive Input values of joint 2 for this simulation


    #    Run the network for one time duration based on the input data
    net1.run(Run_Duration,report='text')

    #   Calculate Neural activity Motor Neurons in the current run
    curr_M1_count = SP_M1.count - prev_M1_count
    prev_M1_count[0] = SP_M1.count[0]
    prev_M1_count[1] = SP_M1.count[1]
    curr_M2_count = SP_M2.count - prev_M2_count
    prev_M2_count[0] = SP_M2.count[0]
    prev_M2_count[1] = SP_M2.count[1]

    #   Calculate and translate motor neurons' spikes into movement of joints
    theta1 += (curr_M1_count[1]-curr_M1_count[0]) * angle_step
    theta2 += (curr_M2_count[1]-curr_M2_count[0]) * angle_step

    ##########
    #   Calculate Neural activity Proprioceptive Neurons in the current run
    curr_tar_p1 = (SP_Prop_T1.count - prev_tar_p1) * 5
    for k in range(N_Prop_1):
        prev_tar_p1[k] = SP_Prop_T1.count[k]

    curr_tar_p2 = (SP_Prop_T2.count - prev_tar_p2) * 5
    for k in range(N_Prop_2):
        prev_tar_p2[k] = SP_Prop_T2.count[k]

    ##########
    #   Calculate Neural activity GainField Neurons in the current run
    curr_GF = (SP_GF.count - prev_GF) * 5
    for k in range(N_GF):
        prev_GF[k] = SP_GF.count[k]

    # store data
    # data.append(proprio_code + touch_activation)
    # some verbose
    if verbose:
        print('touch')
        print(touch_activation)
        print('proprio')
        print(proprio_code)

    if visual:
        # Draw everything
        screen.fill(WHITE)

        # Draw clicked locs
        for pos in circles:
            pygame.draw.circle(screen, RED, pos, 10)  # Radius 20

        # Draw body square
        Torso_rect = pygame.draw.rect(screen, RED, (BODY_X - TOUCH_SENSOR_SPACING/2 + SCREEN_WIDTH // 2, BODY_Y - TOUCH_SENSOR_SPACING/2 + SCREEN_HEIGHT // 2, BODY_SIZE, BODY_SIZE), 2)

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

        #   Just change list to array and multiplyit by a value to draw bar chart
        joint_1_cur = np.array(proprio_code)[:10]*30
        joint_2_cur = np.array(proprio_code)[10:]*30

        x1 = 50
        x2 = 150

        for k in range(len(joint_1_cur)):
            pygame.draw.rect(screen, RED, (x1,y1-joint_1_cur[k] , bar_width, joint_1_cur[k]))
            pygame.draw.rect(screen, BLUE, (x1,y2-curr_tar_p1[k] , bar_width, curr_tar_p1[k]))
            pygame.draw.rect(screen, RED, (x2,y1-joint_2_cur[k] , bar_width, joint_2_cur[k]))
            pygame.draw.rect(screen, BLUE, (x2,y2-curr_tar_p2[k] , bar_width, curr_tar_p2[k]))
            x1 += bar_width + void_width
            x2 += bar_width + void_width

        # Plot Heatmaps
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                # Plot heat map of the TARGET tactile Sensors
                color1 = FN.rgb(0, 1, np.array(touch_activation_target).reshape((3,3), order='F')[row,col])
                pygame.draw.rect(screen, color1, (90+col * CELL_SIZE, 220+row * CELL_SIZE, CELL_SIZE, CELL_SIZE))
                # Plot heat map of the CURRENT tactile Sensors
                color1 = FN.rgb(0, 1, np.array(touch_activation).reshape((3,3), order='F')[row,col])
                pygame.draw.rect(screen, color1, (90+col * CELL_SIZE, 100+row * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        
        # Plot heat map of the GainField Neurons
        for row in range(N_Prop_1):
            for col in range(N_Prop_2):
                color1 = FN.rgb(0, max(curr_GF), np.array(curr_GF).reshape((N_Prop_1,N_Prop_2), order='F')[row,col])
                pygame.draw.rect(screen, color1, (450+col * CELL_SIZE/3, 450+row * CELL_SIZE/3, CELL_SIZE/3, CELL_SIZE/3))
                
        # # Write the Plots' Lables
        screen.blit(xlable_text, (x3, y2+10))
        screen.blit(ycur_text, (5, y1-10))
        screen.blit(ytar_text, (5, y2-10))
        screen.blit(joint1_text, (x3-30, y1-50))
        screen.blit(joint2_text, (x2-50, y1-50))
        screen.blit(CurAct_text, (65, 70))
        screen.blit(TarAct_text, (65, 200))
        screen.blit(GF_text, (400, 430))

        # Draw touch sensors
        for k in range(TOUCH_SENSOR_GRID_SIZE):
            for jj in range(TOUCH_SENSOR_GRID_SIZE):
                x = k * TOUCH_SENSOR_SPACING + BODY_X + SCREEN_WIDTH // 2
                y = jj * TOUCH_SENSOR_SPACING + BODY_Y + SCREEN_HEIGHT // 2
                distance = touch_sensors[k * TOUCH_SENSOR_GRID_SIZE + jj]
                if distance < BODY_SIZE/3:
                    pygame.draw.circle(screen, RED, (int(x), int(y)), 3)
                    pygame.draw.circle(screen, GREEN, (int(x), int(y)), radius=8, width=3)
                else:
                    pygame.draw.circle(screen, (125,125,125), (int(x), int(y)), 3)

        # Draw the iteration counter on the screen
        pygame.draw.rect(screen, quit_button_color, quit_button_rect)
        font = pygame.font.Font(None, 36)
        quit_text = font.render("Quit", True, (255, 255, 255))
        screen.blit(quit_text, (10, 10))
        

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
                    elif Torso_rect.collidepoint(event.pos):  # Check if the mouse click is inside the quit button
                        circles.append(event.pos)# = (event.pos[0], event.pos[1])
                        circles = [(event.pos[0], event.pos[1])]
                        print((circles))
                        print((event.pos))
                        # Calculate touch sensors
                        xx = event.pos[0]
                        yy = event.pos[1]
                        XX = xx - SCREEN_WIDTH // 2
                        YY = yy - SCREEN_HEIGHT // 2
                        #   After detecting a click on Torso (External touch), we calculate tacile activation fro each tactile neuron and and updtae the input current
                        touch_sensors_tar2 = calculate_touch_sensors(XX,YY)
                        touch_activation_target = [numpy.exp(-distance / RECEPTIVE_FIELD_SIZE_TOUCH) for distance in touch_sensors_tar2]
                        print(touch_sensors_tar2)
                        Tactile[:, :] = np.transpose(np.tile(np.transpose(np.array(touch_activation_target)), (100,1)))
                        I_Tactile = TimedArray((Tactile), dt=100 * ms)
        else: #this is so crazy code
            continue
        break
if visual:
    pygame.quit()

if store_csv:
    # Define the CSV file path
    csv_file_path = os.path.expanduser("C:/Users/pabdollahzadeh/Desktop/Codes/Tactile_Prop/Valentin_simple_2d/simulation_test_data_1.csv")
    # Write data to the CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write header row (you can customize this based on your data)
        header = ["Proprio_Code_" + str(k) for k in range(PROPRIO_SENSOR_NUMBER*2)] + ["Touch_Activation_" + str(k) for k in range(TOUCH_SENSOR_GRID_SIZE**2)]
        writer.writerow(header)
        print(header)
        # Write data rows
        for row in data:
            writer.writerow(row)

    print("Data saved to", csv_file_path)