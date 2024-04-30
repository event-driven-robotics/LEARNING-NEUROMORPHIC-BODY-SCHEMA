from pydexarm import Dexarm
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
#help(dexarm)

dexarm.go_home()
#current_position=[0,340,-83]
#dexarm.move_to(*current_position, feedrate=4000, mode="G1")
#current_position=[0,240,-84]
#dexarm.move_to(*current_position, feedrate=4000, mode="G1")
z=-80
ztouch=-82

corner1=[-40,283,z]
corner2=[179,215,z]
corner3=[206,298,z]
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
corner4=corner1 + axis_y
print(corner4)
# init
for i in range(1):
    dexarm.move_to(*corner1, feedrate=4000, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner2, feedrate=4000, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner3, feedrate=4000, mode="G1")
    dexarm.dealy_s(1)
    dexarm.move_to(*corner4, feedrate=4000, mode="G1")
    dexarm.dealy_s(1)

# Convert list of waypoints to numpy array
waypoints = np.array(waypoints)
print(waypoints)
points_3d_up=[]
for points in waypoints[0:len(waypoints):6]:
    points_3d_up= points[:]
    points_3d_up[-1]=ztouch
    dexarm.move_to(*points_3d_up, feedrate=2000, mode="G1")
    print(dexarm.get_current_position())
    # got touch
    points_3d_touch= points[:]
    points_3d_touch[-1]=ztouch
    dexarm.move_to(*points_3d_touch, feedrate=2000, mode="G1")
    dexarm.dealy_s(.1)
    # move up
    points_3d= points[:]
    points_3d[-1]=ztouch
    dexarm.move_to(*points_3d_up, feedrate=2000, mode="G1")


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
