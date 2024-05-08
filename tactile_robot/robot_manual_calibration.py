import sys
import os
# Append the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
sys.path.append(parent_dir+"/DexArm_API")
from pydexarm.pydexarm import Dexarm
import numpy as np
'''windows'''
#dexarm = Dexarm(port="ttyACM0")
'''mac & linux'''
try:
    dexarm = Dexarm(port="/dev/ttyACM0")
except:
    try:
        dexarm = Dexarm(port="/dev/ttyACM1")
    except:
        print('Robot is probably not connected or powered')

dexarm.go_home()

ROBOT_SPEED=10000
z=-80
ztouch=-82

#corner1=[-20,280,z]
#corner2=[179,215,z]
#corner3=[206,298,z]
corner1=[20,270,z]
corner2=[119,245,z]
corner3=[130,308,z]
#corner4=[-3,374,z]


# Define number of points along each axis
num_points_x = 21
num_points_y = 11

# Define axis vectors
axis_x = np.subtract(corner2,corner1)
axis_y = np.subtract(corner3, corner2)

# Generate grid of points
waypoints = []
for i in range(num_points_x):


    for j in range(num_points_y):
        # Calculate coordinates of each point
        x_coord = i / (num_points_x - 1) * axis_x
        y_coord = j / (num_points_y - 1) * axis_y
        # Append the point to the list of waypoints
        waypoints.append(corner1+x_coord+y_coord)
# fourth corner depends on y axis. It should be a rectangle or close.
corner4=corner1 + axis_y

# init: move at the four corners
for i in range(1):
    dexarm.move_to(*corner1, feedrate=ROBOT_SPEED, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner2, feedrate=ROBOT_SPEED, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner3, feedrate=ROBOT_SPEED, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner4, feedrate=ROBOT_SPEED, mode="G1")
    dexarm.dealy_s(1)

# Convert list of waypoints to numpy array
waypoints = np.array(waypoints)
print(waypoints)
points_3d_up=[]
for points in waypoints[0:len(waypoints):6 ]:
    # go above the target point
    points_3d_up= points[:]
    dexarm.move_to(*points_3d_up, feedrate=ROBOT_SPEED, mode="G1")
    print(dexarm.get_current_position())
    # go to touch
    points_3d_touch= points[:]
    points_3d_touch[-1]=ztouch
    dexarm.move_to(*points_3d_touch, feedrate=ROBOT_SPEED, mode="G1")
    dexarm.dealy_s(.1)
    # move up
    points_3d= points[:]
    points_3d[-1]=z
    dexarm.move_to(*points_3d_up, feedrate=ROBOT_SPEED, mode="G1")

# finish and go hom
dexarm.go_home()
