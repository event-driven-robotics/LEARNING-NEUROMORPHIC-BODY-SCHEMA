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
name="final"
#csv_filename = os.path.join(os.getcwd(), "exploration_may8_1505.csv")
csv_filename = f"{os.getcwd()}/{name}.csv"

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
    robot_port = "/dev/ttyACM0"
except:
    try:
        dexarm = Dexarm(port="/dev/ttyACM1")
        robot_port = "/dev/ttyACM1"
    except:
        print('Robot is probably not connected or powered')
#################### SKIN ############################################################
port1 = "/dev/ttyUSB0"  # Change this to your COM ports
port2 = "/dev/ttyUSB1"  # Change this to your COM ports

baud_rate = 250000
calibration_time = .5  # seconds
threshold = 70#20*5 # threshold for detecting tactile events
N = 5 #buffer size for filtering
nfilt = 3 #number of filtering points
threshold_buffer = [[] for _ in range(252)] # Initialize a buffer to store values below the threshold
#################### EXPERIMENTATION PARAMETERS #################################
znotouch=-80
ztouch=-86
z=ztouch
INITIAL_POSITION = [60,250,ztouch]
SCALING = 1/5 #scaling from action to euclidean position
ROBOT_SPEED = 20000

ACTION_SPACE=[[0,1],[1,0],[0,-1],[-1,0]]

N_exploration = 5000
###################### DEFINE WORKING SPACE ##################################

# corner1=[-40,283,z]
# corner2=[179,215,z]
# corner3=[206,298,z]

corner1=[-20,280,z]
corner2=[179,220,z]
corner3=[201,303,z]

# corner1=[20,270,z]
# corner2=[119,245,z]
# corner3=[130,308,z]

# Define number of points along each axis
num_points_x = 21
num_points_y = 11
# Define axis vectors
axis_x = np.subtract(corner2,corner1)
axis_y = np.subtract(corner3, corner2)

ACTION_SPACE=[axis_x[:2],-axis_x[:2],axis_y[:2],-axis_y[:2]]

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
        skinser.flushInput()
        line = skinser.read()
        while not line:
            print('Fetch data')
            line = skinser.read()
        print(line)
        while not len(line)==252:

            line = skinser.readline().decode('latin-1').strip().split(',')
            print('Receive init info', len(line))
        while True:
            print('Receive data')
            data = skinser.readline().decode().strip().split(',')
            print('data',data)
            if len(data)==252:
                print("\rInitialization successful\n")
                break
            else:
                print("\rInitialization failed, received len", len(data), "\n")
    return skinser, port

def get_raw_data():
    #ser.write(b'data\n')  # Send command to request raw data
    #ser.flushInput()
    data = ser.readline().decode().strip()
    cnt=0
    while True:
        if len(data.split(','))==252:
            values = [int(value) for value in data.split(',')]
            break
        else:
            values=None
            #print('Wrong data size:',cnt+1)
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

    # Update previou21s mean buffers for values not above the threshold
    if True:
        for i in below_threshold_indices:
            buffer[i].append(values[i])
            if len(buffer[i]) > N:
                buffer[i].pop(0)
            values_mean[i]=np.mean(buffer[i]) #recompute the mean

    abs_diff[abs_diff<0]=0#data under the mean is noise.

    # reshape to get the tactile image
    values_2d = np.reshape(abs_diff, (21, 12)).astype(np.uint8).T
    values_2d[:,0]=0 # to remove problems on the border
    values_2d[:,-1]=0 # to remove problems on the border
    values_2d[values_2d < threshold] = 0  # Apply thresholding
    values1d = values_2d.T.flatten()

    # Create a 21 by 12 grid

    return isTouch,values1d, values_2d, values_mean, buffer
def pressure2touch(values1d):
    values_2d=np.reshape(values1d, (21, 12)).astype(np.uint8).T
    vmax_pre=np.max(values_2d)
    max_index_flat = np.argmax(values_2d)
    max_index_2d = np.unravel_index(max_index_flat, (12,21)) #or 12,21?
    #print('flat',id_block(max_index_flat))
    # touch = np.zeros(252,)
    # touch[max_index_flat]=1
    # o_pre = torch.tensor(touch).float() #get observation

    block_id_2d=[np.maximum((max_index_2d[0]-1),0)//3,np.maximum((max_index_2d[1]-1),0)//3]
    block_id_flat=block_id_2d[0]*7+block_id_2d[1]
    #print('block_touch', block_touch_pre, 'touche ', touch_pre)
    touch = np.zeros(28,)
    touch[block_id_flat]=1
    return block_id_flat,block_id_2d,touch

def flatten_index(index_2d, shape):
    """
    Flatten a 2D index into a 1D index given the shape of the array.

    Parameters:
        index_2d (tuple): The 2D index in the form (row_index, column_index).
        shape (tuple): The shape of the array in the form (num_rows, num_columns).

    Returns:
        int: The flattened index.
    """
    row_index, col_index = index_2d
    num_rows, num_cols = shape
    return row_index * num_cols + col_index
########## ROBOT FUNTIONS #########
def move_robot(robot, target, wait=True):
    """
    Move to a cartesian position. This will add a linear move to the queue to be performed after all previous moves are completed.

    Args:
        mode (string, G0 or G1): G1 by default. use G0 for fast mode
        x, y, z (int): The position, in millimeters by default. Units may be set to inches by G20. Note that the center of y axis is 300mm.
        feedrate (int): set the feedrate for all subsequent moves
    """
    mode="G1"
    feedrate=ROBOT_SPEED
    x = target[0]
    y = target[1]
    z = target[2]
    cmd = mode + "F" + str(feedrate)
    if x is not None:
        cmd = cmd + "X"+str(round(x))
    if y is not None:
        cmd = cmd + "Y" + str(round(y))
    if z is not None:
        cmd = cmd + "Z" + str(round(z))
    cmd = cmd + "\r\n"
    # send command
    robot.ser.write(cmd.encode())
    if not wait:
        robot.ser.reset_input_buffer()
        return
    cnt=0
    robot.ser.reset_input_buffer()
    if wait:
        cnt=0
        while True:
            cnt+=1
            print('cnt', cnt)
            serial_str = robot.ser.readline().decode("utf-8")
            print('Robot message:', serial_str)
            if len(serial_str) > 0:
                if serial_str.find("ok") > -1:
                    print("movement complete")
                    break
                else:
                    print("read：", serial_str)
            else:
                print("read nothing")
            # if check_movement_complete(robot):
            #     break

# Function to check if the "read ok" message is received
def check_movement_complete(robot):
    serial_str = robot.ser.readline().decode("utf-8")
    print('Robot message:', serial_str)
    if len(serial_str) > 0:
        if serial_str.find("ok") > -1:
            print("movement complete")
            return True
        else:
            print("read：", serial_str)
            return False
    else:
        print("read nothing")
        return False
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
    num_points_x = 100
    num_points_y = 50

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
    sample_target = waypoints[random_index][:]
    return sample_target
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
model = Agent(o_size=28, a_size=4, s_dim=1000)
device = 'cpu'
loss_record = []

target_position= INITIAL_POSITION[:]
counter_training=0

with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    # write Header row
    csv_writer.writerow(['SampledPos_x','SampledPos_y','ObservationPre_LinID','ObservationPre_row','ObservationPre_col','Action','ReachedPos_x','ReachedPos_y','ObservationPost_LinID','ObservationPost_row','ObservationPost_col','discrTouch','Loss'])
    #plt.figure()
    #for n in range(N_exploration):
    n=0
    while True:
        target_position_pre = sample_point_on_skin(corners) #sample position on skin
        dexarm.move_to(*target_position_pre,feedrate=ROBOT_SPEED,mode='G1', wait=True) #move robot
        # flter and get the touch
        ser.flushInput()

        v_buff=[[] for i in range(nfilt)]
        for i in range(nfilt):
            v_buff[i] = get_raw_data()
            #update buffer
            isTouch,abs_diff, values_2d, values_mean, threshold_buffer =get_tactile(v_buff[i],threshold_buffer)
            #compute mean
        values=v_buff[-1]#values=np.mean(v_buff,axis=0)

        isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
        block_touch_pre,touch_pre,touch = pressure2touch(values1d)
        vmax_pre=np.max(values_2d)

        o_pre = torch.tensor(touch).float() #get observation

        if display:
            resized = cv2.resize(values2d, dim, interpolation=cv2.INTER_AREA)
            cv2.imshow('Skin Patch', cv2.applyColorMap(resized, cv2.COLORMAP_VIRIDIS))
            cv2.waitKey(1)
        while True:
            n+=1

            #sample an action
            inside=False
            while not inside:
                action = np.random.randint(0, len(ACTION_SPACE))
                action_performed = ACTION_SPACE[action]
                inside = point_inside_corners(target_position_pre[0]+SCALING*action_performed[0], target_position_pre[1]+SCALING*action_performed[1], corners)

            target_position[:2] = target_position_pre[:2] + SCALING*np.array(action_performed)
            target_position[-1] = ztouch

            dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True) #move robot

            # flter and get the touch
            ser.flushInput()
            v_buff=[[] for i in range(nfilt)]
            for i in range(nfilt):
                v_buff[i] = get_raw_data()

                #update buffer
                isTouch,values1d, values_2d, data_mean, threshold_buffer =get_tactile(v_buff[i],threshold_buffer)
                #compute mean
            values=v_buff[-1]#np.mean(v_buff,axis=0)
            isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)



            block_touch_next,touch_next,touch = pressure2touch(values1d)
            vmax_next=np.max(values_2d)
            o_next = torch.tensor(touch).float() #get observation

            if display:
                resized = cv2.resize(values2d, dim, interpolation=cv2.INTER_AREA)
                cv2.imshow('Skin Patch', cv2.applyColorMap(resized, cv2.COLORMAP_VIRIDIS))
                cv2.waitKey(1)

            # write data


            # move up
            #target_position[-1]=znotouch
            #dexarm.move_to(*target_position,feedrate=ROBOT_SPEED,mode='G1', wait=True)
            # compute the loss
            #print('pre',vmax_pre,'next',vmax_next)
            if vmax_pre>threshold and vmax_next>threshold:
                #write data

                #train
                counter_training+=1
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
                    print("trajectory:", n, "Epoch:", counter_training,"Loss ",loss.item())
                    #print data
                    csv_row=[]
                    csv_row.extend([target_position_pre[0],target_position_pre[1],block_touch_pre,touch_pre[0],touch_pre[1]])
                    csv_row.extend([action,target_position[0],target_position[1],block_touch_next,touch_next[0],touch_next[1],np.minimum(vmax_pre,vmax_next),loss.item()])
                    csv_writer.writerow(csv_row)
                #plt.clf
                #plt.plot(loss_record, c='r')
            o_pre = o_next
            touch_pre = touch_next[:]
            block_touch_pre = block_touch_next
            target_position_pre=target_position[:]
            vmax_pre=vmax_next
            #plt.pause(0.001)
            #plt.draw()
            if counter_training%200 == 0:
                torch.save(model.state_dict(), f'./checkpoint{name}{counter_training}.pt')
            if not n%10:
                break #new trajectory

        if n>N_exploration:
            break




torch.save(model.state_dict(), './checkpoint')

# model = TheModelClass(*args, **kwargs)
# model.load_state_dict(torch.load(PATH))
# model.eval()

