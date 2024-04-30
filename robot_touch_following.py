import sys
sys.path.append('/home/abdelnasser/Git_projects/tactilerobot/')
from DexArm_API.pydexarm import Dexarm

import serial
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.ndimage import convolve1d
from scipy.ndimage import median_filter

import curses  # Import the keyboard module
import select  # Import the select module
import sys
import fcntl
import os
display = 0
if display:
    import cv2
# Configuration
port_skin = "/dev/ttyUSB0"  # Change this to your COM port
baud_rate_skin = 250000
# config robot
port_robot = "/dev/ttyACM0"  # Change this to your second COM port
baud_rate_robot = 115200
# skin param

calibration_time = .5  # seconds
threshold = 20
N = 10
threshold_buffer = [[] for _ in range(252)]
# robot init
z=-70 #no touch height
ztouch=-70 #touch height
#skin corners
corner1=[-45,290,z]
corner2=[177,215,z]
corner3=[206,298,z]

axis_x = np.subtract(corner2,corner1)
axis_y = np.subtract(corner3, corner2)
axis_x_n = axis_x/np.linalg.norm(axis_x)
axis_y_n = axis_y/np.linalg.norm(axis_y)

corner4=corner1 + axis_y
center_position = corner4[:]
# Define axis vectors



robot_speed = 10000;
robot_max_step = 5;

try:
    ser_skin = serial.Serial("/dev/ttyUSB0", baud_rate_skin, timeout=1)
    port_skin = "/dev/ttyUSB0"
except:
    try:
        ser_skin = serial.Serial("/dev/ttyUSB1", baud_rate_skin, timeout=1)
        port_skin = "/dev/ttyUSB1"
    except:
        print("No port detected")

try:
    ser_robot = serial.Serial("/dev/ttyACM1", baud_rate_robot, timeout=1)
    port_skin = "/dev/ttyACM1"
except:
    try:
        ser_robot = serial.Serial("/dev/ttyACM0", baud_rate_robot, timeout=1)
        port_robot = "/dev/ttyACM0"
    except:
        print("No port detected")

## ROBOT FUNCTIONS
dexarm = Dexarm(port=port_robot)

## SKIN FUNCTIONS

def init_skin():
    print("Initializing Skin ...\n")
    #ser.write(b'init\n')  # Send command to initialize skin
    #ser.write(b'data\n')
    while True:
        try:
            response = ser_skin.readline().decode('utf-8').strip().split(',')
        except UnicodeDecodeError:
            response = ser_skin.readline().decode('latin-1').strip().split(',')
        if len(response)==252:
            print("\rInitialization successful\n")
            break
        else:
            print("\rInitialization failed, received len", len(response), "\n")

def init_robot():
    print("Initializing Robot...")
    # Define initial position
    dexarm.go_home()
    new_position=center_position[:]
    dexarm.move_to(*new_position, feedrate=4000, mode="G1", wait=True)

    return new_position[:]

def get_raw_data():
    ser_skin.write(b'data\n')  # Send command to request raw data
    data = ser_skin.readline().decode().strip()
    return data.split(',')  # Split the received data by comma

def get_touch_position(values_2d):
    if np.all(values_2d == 0):
        return None
    # Find the indices of the maximum value in the flattened array
    max_index_flat = np.argmax(values_2d)

    # Convert the flat index to 2D coordinates
    index_touch_2d = np.unravel_index(max_index_flat, values_2d.shape)
    return index_touch_2d

def index2cart(touch_2d_index):
    #from id to mm
    if not touch_2d_index:
        return None
    touch_2d_cart= [touch_2d_index[0] * 8.95, touch_2d_index[1] * 11.3] #8.73
    return touch_2d_cart #reutnr in mm
def cart2robotframe(touch_skin):
    # Construct the transformation matrix T
    T = np.column_stack((-axis_y_n[:2], axis_x_n[:2]))

    # Compute the inverse of the transformation matrix
    T_inv = np.linalg.inv(T)
    touch_roF = np.dot(T, np.array(touch_skin[:2]).T).T + center_position[:2]
    print("touchskin",touch_skin[:2],"touchrof",touch_roF, "T", corner4)
    touch_roF = np.append(touch_roF,z)

    return touch_roF

# Initialize skin
init_skin()

# Display parameters
scale_percent = 5000
img = np.zeros((12, 21))
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

# Variables for storing data over time
data_buffer = []
time_buffer = []
time_step = 1 / 36.0  # Time step in seconds
time = 0 - time_step

# calibration mode
calibration_mode = True
#print("center",corner4, "u1",axis_x_n,"u2",axis_y_n)

try:
    # Initialize curses
    stdscr = curses.initscr()
    curses.cbreak()  # Disable line buffering
    stdscr.keypad(True)  # Enable keypad mode
    #Set stdin to non-blocking mode
    fd = sys.stdin.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

#    new_position = center_position[:]

    while True:
        # Read data until '\r\n'
        #raw_data = ser_skin.readline().decode().strip()
        raw_data = get_raw_data()
        time+=time_step
        # Check if data is not empty
        if raw_data and len(raw_data) == 252:
            try:
                # Split data into individual values and convert to integers
                values = [int(value) for value in raw_data]
            except ValueError:
                # If conversion to integers fails, continue reading from ser_skinial until valid data is received
                print("\rInvalid data received. Waiting for valid data...\n")
                continue

            if time< calibration_time:
                print(f"\rCalibration left: {calibration_time - time:.2f} s\n")
                data_buffer.append(values)
            else:
                if calibration_mode:
                    data_mean = np.mean(data_buffer, axis = 0)
                    #fill the running calibration buffer
                    for i in range(252):
                        for j in range(N):
                            threshold_buffer[i].append(data_mean[i])

                    print("mean_data: ", data_mean,"\n")
                    calibration_mode = False

                    print('\rEnd Calibration\n')
                    old_position = init_robot()
                    new_position = old_position[:]

                abs_diff = -np.subtract(values,data_mean)/5
                abs_diff[abs_diff<0]=0
                below_threshold_indices = np.where(abs_diff < threshold)[0]

                # Update previous mean buffers for values not above the threshold
                for i in below_threshold_indices:
                    threshold_buffer[i].append(values[i])
                    if len(threshold_buffer[i]) > N:
                        threshold_buffer[i].pop(0)
                    data_mean[i]=np.mean(threshold_buffer[i])
                abs_diff = -np.subtract(values,data_mean)/5
                abs_diff[abs_diff<0]=0
                values_2d = np.reshape(abs_diff, (21, 12)).astype(np.uint8).T

                values_2d[values_2d < threshold] = 0  # Apply thresholding
                touch_2d = get_touch_position(values_2d)
                xy = index2cart(touch_2d)

                if xy:
                    xy_roF = cart2robotframe(xy)
                    print("\rTouched position:", xy, "coords", xy_roF,"mm\n")
                    #new_position[0]=center_position[0]-xy[1]+90-100
                    #new_position[1]=center_position[1]+xy[0]-55
                    Dx = old_position[0]-xy_roF[0]
                    Dy = old_position[1]-xy_roF[1]
                    print("dx:", Dx, "dy:", Dy)


                    new_x=old_position[0]-np.sign(Dx)*(min(np.abs(Dx),robot_max_step))
                    new_y=old_position[1]-np.sign(Dy)*(min(np.abs(Dy),robot_max_step))
                    new_position = [new_x, new_y, old_position[2]]  # Keep z coordinate unchanged
                    print("\rCommand:", [-np.sign(Dx)*(min(np.abs(Dx),robot_max_step)),np.sign(Dy)*(min(np.abs(Dy),robot_max_step))], "mm\n")
                    print("\rRobot going to :", new_position, "mm\n")
                    dexarm.move_to(*new_position, feedrate=robot_speed, mode="G1", wait=True)
                    old_position=new_position[:]
                    #current_position=dexarm.get_current_position()
                    #print("\rCurrent pos: ", current_position)
                #values_2d[values_2d >= threshold] = 255  # Apply thresholding
                # Filter the values using Butterworth filter
                #print(derivative_values)
                # Display the image using OpenCV
                if display:
                    resized = cv2.resize(values_2d, dim, interpolation=cv2.INTER_AREA)
                    cv2.imshow('Skin Patch', cv2.applyColorMap(resized, cv2.COLORMAP_VIRIDIS))
                    #Handle key events
                    key = cv2.waitKey(1) & 0xFF
                    if key == 13:  # Enter key for calibration
                        print("Calibration pressed.")
                        time = 0 - time_step  # Reset time for calibration
                        flag = True  # Set flag for recalibration
                        data_buffer=[]
                    elif key == 27:  # Escape key to exit
                        break
                    elif key == ord('+'):  # Up key to increase threshold
                        threshold += 10
                        print("Threshold increased to:", threshold)
                    elif key == ord('-'):  # Down key to decrease threshold
                        threshold -= 10
                        print("Threshold decreased to:", threshold)
                else:
                        # Check if input is available on stdin without blocking
                    try:
                        key = stdscr.getch()
                        if key == curses.KEY_ENTER or key == 10:  # Enter key for calibration
                            print("\rCalibration pressed.")
                            time = 0 - time_step  # Reset time for calibration
                            flag = True  # Set flag for recalibration
                            data_buffer = []
                        elif key == 27:  # Escape key to exit
                            break
                        elif key == ord('+'):  # Plus key to increase threshold
                            threshold += 5
                            print("\rThreshold increased to:", threshold)
                        elif key == ord('-'):  # Minus key to decrease threshold
                            threshold -= 5
                            print("\rThreshold decreased to:", threshold)
                    except curses.error:
                        pass

        else:
            data_buffer=[]
            time = 0 - time_step
            print("\rReceived invalid data length:", len(raw_data),"\n")



except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    # Close curses
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()
    # Close the serial port
    ser_skin.close()
    #cv2.destroyAllWindows()
