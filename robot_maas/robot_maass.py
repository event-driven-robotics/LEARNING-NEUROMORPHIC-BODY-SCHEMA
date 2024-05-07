import sys
import os
import math
import serial
import torch
import torch.nn as nn
pi=math.pi
import random #for generating random angles
# Append the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
sys.path.append(parent_dir+"/DexArm_API")
from pydexarm.pydexarm import Dexarm

import numpy as np
import matplotlib.pyplot as plt

import csv
from datetime import datetime

## import the maas agent

from robomaas import Agent
# file name where you want the data to be stored
csv_filename = os.path.join(os.getcwd(), "tactile_events_robot.csv")
## display
display=1
if display:
    import cv2
    # windows parameters
    scale_percent = 5000
    img = np.zeros((12, 21))
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
#################### ROBOT ############################################################
try:
    dexarm = Dexarm(port="/dev/ttyACM0")
except:
    try:
        dexarm = Dexarm(port="/dev/ttyACM1")
    except:
        print('Robot is probably not connected or powered')
#################### SKIN ############################################################
port1 = "/dev/ttyUSB0"  # Change this to your COM ports
port2 = "/dev/ttyUSB1"  # Change this to your COM ports

baud_rate = 250000
calibration_time = .5  # seconds
threshold = 20*4#20*5 # threshold for detecting tactile events
N = 10 #buffer size for filtering
threshold_buffer = [[] for _ in range(252)] # Initialize a buffer to store values below the threshold
#################### EXPERIMENTATION PARAMETERS #################################
znotouch=-80
ztouch=-84
z=ztouch
INITIAL_POSITION = [60,250,ztouch]
SCALING = 20 #scaling from action to euclidean position
ROBOT_SPEED = 30000

ACTION_SPACE=[[0,1],[1,0],[0,-1],[-1,0]]

N_exploration = 10000
###################### DEFINE WORKING SPACE ##################################


corner1=[-20,280,z]
corner2=[179,215,z]
corner3=[206,298,z]
# Define number of points along each axis
num_points_x = 21
num_points_y = 11
# Define axis vectors
axis_x = np.subtract(corner2,corner1)
axis_y = np.subtract(corner3, corner2)

corner4=corner1 + axis_y
corners = [corner1,corner2,corner3,corner4]

########## SKIN FUNCTIONS #########
def init_skin():
    skinser = None  # Initialize skinser variable
    port = None  # Initialize port variable

    print("Initializing Skin ...\n")
    try: # get serial port
        skinser = serial.Serial(port1, baud_rate, timeout=1)
        port=port1
    except:
        try:
            skinser = serial.Serial(port2, baud_rate, timeout=1)
            port=port2
        except:
            print("No port detected")
    #ser.write(b'init\n')  # Send command to initialize skin
    #ser.write(b'data\n')
    if skinser is not None:  # Check if skinser is successfully initialized
        while True:
            try:
                response = skinser.readline().decode('utf-8').strip().split(',')
            except UnicodeDecodeError:
                response = skinser.readline().decode('latin-1').strip().split(',')
            if len(response)==252:
                print("\rInitialization successful\n")
                break
            else:
                print("\rInitialization failed, received len", len(response), "\n")
    return skinser, port

def get_raw_data():
    #ser.write(b'data\n')  # Send command to request raw data
    ser.reset_input_buffer()
    data = ser.readline().decode().strip()
    cnt=0
    while True:
        if len(data.split(','))==252:
            values = [int(value) for value in data.split(',')]
            break
        else:
            values=None
            print('Wrong data size')
            data = ser.readline().decode().strip()
            cnt+=1
            if cnt>10:
                print("recalibrate")
                ser.reset_input_buffer()
                data_mean, threshold_buffer = calibrate_skin()
                cnt=0
    return values  # Split the received data by comma

def get_touch_position(values_2d):
    if np.all(values_2d == 0):
        return None
    # Find the indices of the maximum value in the flattened array
    max_index_flat = np.argmax(values_2d)

    # Convert the flat index to 2D coordinates
    index_touch_2d = np.unravel_index(max_index_flat, values_2d.shape)
    return index_touch_2d

def calibrate_skin():
    # this function get tactile data compute the mean of the values to be removed. Each sensors has a bias.
    counter=0
    data_buffer=[]
    while counter<N:
        raw_data = get_raw_data()
        if raw_data and len(raw_data) == 252:
            counter+=1
            try:
                # Split data into individual values and convert to integers
                values = [int(value) for value in raw_data]
            except ValueError:
                # If conversion to integers fails, continue reading from serial until valid data is received
                print("\rInvalid data received. Waiting for valid data...\n")
                continue


        data_buffer.append(values)
    data_mean = np.round(np.mean(data_buffer, axis = 0))
    threshold_buffer_init = [[] for _ in range(252)]  # Initialize an empty list of lists
    for i in range(252):
        for j in range(N):
            threshold_buffer_init[i].append(data_mean[i])
    return data_mean, threshold_buffer_init

def get_tactile(values,buffer):
    # this function the mean
    values_mean = [[] for _ in range(252)]
    # Compute the mean value
    for i in range(252):
        values_mean[i]=np.mean(buffer[i])
    #compute touch events
    abs_diff = np.subtract(values_mean,values) #get the activation -> a press decreases the touch value
    #abs_diff[abs_diff<0]=0
    isTouch = np.any(abs_diff < threshold) #flag if touch
    below_threshold_indices = np.where(abs_diff < threshold)[0] #values under the threshold to be pushed to the buffer

    # Update previous mean buffers for values not above the threshold
    if True:
        for i in below_threshold_indices:
            buffer[i].append(values[i])
            if len(buffer[i]) > N:
                buffer[i].pop(0)
            values_mean[i]=np.mean(buffer[i]) #recompute the mean

    abs_diff[abs_diff<0]=0#data under the mean is noise.

    # reshape to get the tactile image
    values_2d = np.reshape(abs_diff, (21, 12)).astype(np.uint8).T
    values_2d[values_2d < threshold] = 0  # Apply thresholding

    return isTouch,abs_diff, values_2d, values_mean, buffer

########## ROBOT FUNTIONS #########
def forward_kinematics_2d(theta):
    global ARM_LENGTHS,ROBOT_ORIGIN
    #get arm lengths
    UPPERARM_LENGTH = ARM_LENGTHS[0]
    FOREARM_LENGTH = ARM_LENGTHS[1]
    #get joint angles
    theta1 = theta[0]+math.pi/2-math.pi/9
    theta2 = theta[1]
    #forward kinematics
    elbow_x = UPPERARM_LENGTH * math.cos(theta1)
    elbow_y = UPPERARM_LENGTH * math.sin(theta1)
    end_effector_x = UPPERARM_LENGTH * math.cos(theta1) + FOREARM_LENGTH * math.cos(theta1 + theta2)+ROBOT_ORIGIN[0]
    end_effector_y = UPPERARM_LENGTH * math.sin(theta1) + FOREARM_LENGTH * math.sin(theta1 + theta2)+ROBOT_ORIGIN[1]
    return end_effector_x, end_effector_y, elbow_x, elbow_y

def random_action():
    global MAX_ANGLE1, MAX_ANGLE2
    action=[[0],[0]]
    action[0] = random.uniform(-ANGLE_STEP, ANGLE_STEP)
    action[1] = random.uniform(-ANGLE_STEP, ANGLE_STEP)
    return action

def move_to_next_angle(theta,action,corners):
    global ANGLE_STEP, MAX_ANGLE1, MAX_ANGLE2, MIN_ANGLE1, MIN_ANGLE2

    theta1 = theta[0]+action[0]
    theta2 = theta[1]+action[1]
    # Clamp angles within the specified range
    theta1 = max(min(theta1, MAX_ANGLE1), MIN_ANGLE1)
    theta2 = max(min(theta2, MAX_ANGLE2), MIN_ANGLE2)

    theta=[theta1,theta2]

    [end_effector_x, end_effector_y, elbow_x, elbow_y] = forward_kinematics_2d(theta)
    return end_effector_x, end_effector_y, theta, elbow_x, elbow_y
def sample_point_on_skin(corners):
# Define number of points along each axis
    num_points_x = 21
    num_points_y = 11

    # Define axis vectors
    axis_x = np.subtract(corners[1],corners[0])
    axis_y = np.subtract(corners[2], corners[1])

    # Generate grid of points
    waypoints = []
    for i in range(num_points_x):


        for j in range(num_points_y):
            # Calculate coordinates of each point
            x_coord = i / (num_points_x - 1) * axis_x
            y_coord = j / (num_points_y - 1) * axis_y
            # Append the point to the list of waypoints
            waypoints.append(corners[0]+x_coord+y_coord)
    random_index = np.random.randint(0, len(waypoints))
    target_3d = waypoints[random_index][:]
    return target_3d
###### UTILS
def point_inside_corners(x, y, corners):
    n = len(corners)
    inside = False

    p1x, p1y = corners[0][:2]
    for i in range(n + 1):
        p2x, p2y = corners[i % n][:2]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def distance(p1, p2):
    """Calculate distance between two points."""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def closest_point_on_edge(p, edge_start, edge_end):
    """Find the closest point on an edge to a given point."""
    # Vector from edge_start to p
    v = np.array(p) - np.array(edge_start)
    # Vector from edge_start to edge_end
    e = np.array(edge_end) - np.array(edge_start)
    # Calculate the parameter t
    t = np.dot(v, e) / np.dot(e, e)
    # If t is less than 0, closest point is edge_start
    if t <= 0:
        return edge_start
    # If t is greater than 1, closest point is edge_end
    elif t >= 1:
        return edge_end
    # Otherwise, closest point is on the edge
    else:
        return edge_start + t * e

def closest_point_on_polygon_border(p, corners):
    """Find the closest point on the border of a polygon to a given point."""
    # Check if the point is inside the polygon
    if point_inside_corners(p[0],p[1], corners):
        return p, 0  # Point is already inside the polygon, return itself and distance 0

    min_distance = float('inf')
    closest_point = None

    # Iterate through each edge of the polygon
    for i in range(len(corners)):
        start = corners[i][:2]
        end = corners[(i + 1) % len(corners)][:2]  # Wrap around to the first corner

        # Find the closest point on the edge to the given point
        closest = closest_point_on_edge(p, start, end)

        # Calculate the distance between the given point and the closest point on the edge
        dist = distance(p, closest)

        # Update the closest point and its distance if necessary
        if dist < min_distance:
            min_distance = dist
            closest_point = closest

    return closest_point, min_distance
##### AGENT FUNCTION


# first init skin
ser,port=init_skin()
data_mean, threshold_buffer = calibrate_skin()
## INIT ROBOT
dexarm.go_home()

# init agent
norm=False
model = Agent(o_size=2, a_size=4, s_dim=1000)
device = 'cpu'
loss_record = []

target_position= INITIAL_POSITION[:]

plt.figure()
for n in range(1000):

    target_position = sample_point_on_skin(corners) #sample position on skin

    dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True) #move robot
    dexarm.dealy_ms(20)
    #get touched position indexes
    values=get_raw_data()
    isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
    values_2d=np.reshape(values1d, (21, 12)).astype(np.uint8).T
    max_index_flat = np.argmax(values_2d)
    index_touch_2d = np.unravel_index(max_index_flat, values2d.shape)

    o_pre = torch.tensor(index_touch_2d).float() #get observation
    print('O_pre debug' , o_pre.shape)

    #target_position[-1]=znotouch
    #dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True)
    print('Exploration step:', n, 'touch id',index_touch_2d)
    #sample an action
    inside=False
    while not inside:
        action = np.random.randint(0, len(ACTION_SPACE))
        action_performed = ACTION_SPACE[action]
        inside = point_inside_corners(target_position[0]+SCALING*action_performed[0], target_position[1]+SCALING*action_performed[1], corners)

    target_position[:2] = target_position[:2] + SCALING*np.array(action_performed)
    target_position[-1] = ztouch

    print("choosen action:", action_performed)
    dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True)
    dexarm.dealy_ms(20)
    values=get_raw_data()
    isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
    values_2d=np.reshape(values1d, (21, 12)).astype(np.uint8).T
    max_index_flat = np.argmax(values_2d)
    o_next = torch.tensor(index_touch_2d).float() #get next observation
    index_touch_2d = np.unravel_index(max_index_flat, values2d.shape)
    # move up
    #target_position[-1]=znotouch
    #dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True)

    print('Exploration step:', n, 'touch id',index_touch_2d)
    # compute the loss
    with torch.no_grad():
        identity = torch.eye(model.a_size).to(device)
        state_diff = model.Q@o_next-model.Q@o_pre
        prediction_error = state_diff - model.V[:,action]
        desired = identity[action].T # TODO: maybe remove?

        # Core learning rules:
        model.Q += -0.1 * torch.outer(prediction_error, o_next)#TODO:o.T?
        model.V[:,action] += 0.01 * prediction_error
        if norm:
            model.V.data = model.V / torch.norm(model.V, dim=0)

        loss = nn.MSELoss()(prediction_error, torch.zeros_like(prediction_error))
        loss_record.append(loss.cpu().item())

    plt.plot(loss_record, c='r')
    plt.pause(0.001)
    plt.draw()


torch.save(model.state_dict(), './checkpoint')

# model = TheModelClass(*args, **kwargs)
# model.load_state_dict(torch.load(PATH))
# model.eval()

