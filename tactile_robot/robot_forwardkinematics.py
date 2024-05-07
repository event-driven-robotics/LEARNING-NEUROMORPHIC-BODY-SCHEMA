import sys
import os
import math
import serial
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

# file name where you want the data to be stored
csv_filename = os.path.join(os.getcwd(), "tactile_events_robot.csv")

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
threshold = 20*5 # threshold for detecting tactile events
N = 10 #buffer size for filtering
threshold_buffer = [[] for _ in range(252)] # Initialize a buffer to store values below the threshold
#################### EXPERIMENTATION PARAMETERS #################################
ARM_LENGTHS=[200,200]
ROBOT_ORIGIN = [0,0]
#ANGLE_STEP=math.pi/32
ANGLE_STEP=math.pi/32
ROBOT_SPEED = 5000
MAX_ANGLE1=math.pi/2
MAX_ANGLE2=math.pi
MIN_ANGLE1=-math.pi/2
MIN_ANGLE2=-math.pi
INIT_JOINT_ANGLES=[-math.pi/4,math.pi/4]
#theta=[0,0]
#dexarm.move_to(*current_position, feedrate=4000, mode="G1")
N_exploration = 10000
###################### DEFINE WORKING SPACE ##################################
znotouch=-80
z=-85
ztouch=-85

corner1=[-40,283,z]
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


# first init skin
ser,port=init_skin()
data_mean, threshold_buffer = calibrate_skin()
## INIT ROBOT
dexarm.go_home()
# init
# simulation
simu_points=[];

with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    # write Header row
    csv_writer.writerow(['Time','actionAngle1','actionAngle2','jointAngle1','jointAngle2','pos_x','pos_y','touch_id_row','touch_id_col','touch'])
    #starting time
    start_time_ms = int(datetime.now().timestamp() * 1e3)
    #launch exploration
    theta = INIT_JOINT_ANGLES[:] #init posture
    a=np.zeros(len(ARM_LENGTHS),)
    #where am I?
    [target_x, target_y, new_theta, elbow_x, elbow_y]=move_to_next_angle(theta,a,corners)
    #inside the box?
    inside = point_inside_corners(target_x, target_y, corners)
    closest_point, distance_to_border = closest_point_on_polygon_border([target_x,target_y], corners)

    print("Closest point on the border:", closest_point)
    print("Distance to the border:", distance_to_border)
    #get tactile data
    if inside:
        target_3d=[target_x,target_y,ztouch]
        values=get_raw_data()
        isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
        max_index_flat = np.argmax(values2d)
        index_touch_2d = np.unravel_index(max_index_flat, values2d.shape)
        current_time_ms = -start_time_ms+int(datetime.now().timestamp() * 1e3)
        csv_row = [current_time_ms,a[0],a[1], theta[0], theta[1], target_x, target_y,index_touch_2d[0],index_touch_2d[1]]
        val=np.reshape(values1d, (21, 12)).astype(np.uint8).T
        csv_row.extend(np.ravel(val))
        csv_writer.writerow(csv_row)
    else:
        target_3d=[closest_point[0],closest_point[1],znotouch]
        values=np.zeros(252,)
        values2d=np.reshape(values, (21, 12)).astype(np.uint8).T
        index_touch_2d=[-1,-1]
    #compute tactile activations

    # write data


    for i in range(N_exploration):
        # take a random action (joint angle change)
        a=random_action()
        # get the target position/posture inside the corner space
        [target_x, target_y, theta, elbow_x, elbow_y]=move_to_next_angle(theta,a,corners)
        # check if point inside corners
        inside = point_inside_corners(target_x, target_y, corners)
        if not(inside):
            target_3d[-1]=znotouch
            dexarm.move_to(*target_3d, feedrate=ROBOT_SPEED, mode="G1", wait=True) #move up
            # if not, find another action
            while not(inside):
                #store data
                if True: #artificial data notouch
                    #get_raw_data() #to update the sensor?
                    values=np.zeros(252,)
                    values2d=np.reshape(values, (21, 12)).astype(np.uint8).T
                    index_touch_2d=[-1,-1]
                else:  #real data but slow
                    values=get_raw_data()
                    isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
                    max_index_flat = np.argmax(values2d)
                    index_touch_2d = np.unravel_index(max_index_flat, values2d.shape)
                #
                if False:
                    current_time_ms = -start_time_ms+int(datetime.now().timestamp() * 1e3)
                    csv_row = [current_time_ms,a[0],a[1], theta[0], theta[1], target_x, target_y,index_touch_2d[0],index_touch_2d[1]]
                    csv_row.extend(np.ravel(values2d))
                    csv_writer.writerow(csv_row)
                # check closest point projected in the corners
                closest_point, distance_to_border = closest_point_on_polygon_border([target_x,target_y], corners)
                #target_3d=[closest_point[0],closest_point[1],-80]
                #dexarm.move_to(*target_3d, feedrate=ROBOT_SPEED, mode="G1", wait=True)

                print("Closest point on the border:", closest_point)
                print("Distance to the border:", distance_to_border)

                #continue moving
                a=random_action()
                print("oubound",theta,a, target_x, target_y)
                [target_x, target_y, theta, elbow_x, elbow_y]=move_to_next_angle(theta,a,corners)
                inside = point_inside_corners(target_x, target_y, corners)

        #variable with targets, elbows position and if inside
        simu_points.append([target_x,target_y,elbow_x,elbow_y,inside])

        #move the robot
        target_3d = [target_x,target_y,ztouch]
        #ser.reset_input_buffer()
        values=get_raw_data()
        dexarm.move_to(*target_3d, feedrate=ROBOT_SPEED, mode="G1", wait=True)
        dexarm.dealy_ms(10)
        values=get_raw_data()
        isTouch, values1d, values2d, data_mean, threshold_buffer = get_tactile(values,threshold_buffer)
        # Find the indices of the maximum value in the flattened array
        max_index_flat = np.argmax(values2d)
        index_touch_2d = np.unravel_index(max_index_flat, values2d.shape)

        current_time_ms = -start_time_ms+int(datetime.now().timestamp() * 1e3)
        print(np.reshape(values1d, (21, 12)).astype(np.uint8).T)

        #write data
        csv_row = [current_time_ms,a[0],a[1], theta[0], theta[1], target_x, target_y,index_touch_2d[0],index_touch_2d[1]]
        csv_row.extend(values1d.astype(np.uint8))
        csv_writer.writerow(csv_row)

        #theta = new_theta

    # Plot the positions of the upper arm and forearm

        print("Step",i,"going to ", target_3d, "angle: ", theta, "action:", a)
    # move the robot
    #dexarm.move_to(*target_3d, feedrate=ROBOT_SPEED, mode="G1")
    # get computer time for sync with touch.
        # write data
dexarm.go_home()
print(simu_points)
#real
if False:
    theta=[0,0]
    for i in range(N_exploration):
        a=random_action()
        print("next angle",a)

        [target_x, target_y, elbow_x, elbow_y]=move_to_next_angle(theta,a,corners)

        inside = point_inside_corners(target_x, target_y)
        if inside:
            target_3d = [target_x,target_y,z]
            theta = np.add(theta, a)
        else:
            print("outbound:",[target_x,target_y])

        print("going to ", target_3d, "angle: ", theta)
        # move the robot
        dexarm.move_to(*target_3d, feedrate=ROBOT_SPEED, mode="G1")
        # get computer time for sync with touch.
            # write data

# Separate points based on the true/false flag
true_points = [point[:4] for point in simu_points if point[4] is True]
false_points = [point[:4] for point in simu_points if point[4] is False]

# Extract x and y coordinates
true_x = [point[0] for point in true_points]
true_y = [point[1] for point in true_points]
false_x = [point[0] for point in false_points]
false_y = [point[1] for point in false_points]

plt.ion()
plt.figure()
# Plot the points with different colors
for point in simu_points:
    # Extract coordinates for each line
    x_values = [ROBOT_ORIGIN[0], point[2], point[0]]
    y_values = [ROBOT_ORIGIN[1], point[3], point[1]]

    # Plot the line
    plt.plot(x_values, y_values, color='green')
# plot endpoints
plt.scatter(true_x, true_y, color='blue', label='True')
plt.scatter(false_x, false_y, color='red', label='False')
# plot elbows
plt.scatter([points[2] for points in simu_points], [points[3] for points in simu_points], color='black')
# plt.plot([[ROBOT_ORIGIN[0], point[0], point[2]] for point in simu_points], [[ROBOT_ORIGIN[1], point[1], point[3]] for point in simu_points], color='green')
# plt.plot([[ROBOT_ORIGIN[0], point[0], point[2]] for point in simu_points], [[ROBOT_ORIGIN[1], point[1], point[3]] for point in simu_points], color='green')
# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory')
plt.legend()

# Show plot
plt.grid(True)
plt.axis('equal')
plt.pause(100)
# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory')
plt.legend()

# Show plot
plt.grid(True)
plt.axis('equal')
plt.show()

# Convert list of waypoints to numpy array
print(waypoints)
#dexarm.move_to(*current_position, feedrate=4000, mode="G1")
print(dexarm.get_current_position())
    # Define initial position
    # for i in range(-300,300,50):
    #     print("X",i)
    #     for j in range(0,500,20):
    #         current_position = [i, j, -84]
    #         print("Y",j)
    #         dexarm.move_to(*current_position, feedrate=4000, mode="G1")

    #dexarm.go_home()
