import serial #serial can be imported using pip/pip3/conda install pyserial
import numpy as np

from scipy.ndimage import convolve1d
from scipy.ndimage import median_filter


import curses  # Import the keyboard module
import select  # Import the select module
import sys
import fcntl
import os

import csv
from datetime import datetime, timedelta  # Import datetime module for getting the current time

#csv_filename ="~/your_git_repo_name/tactilerobot/sm_events.csv"
#csv_filename = os.path.join(os.getcwd(), "./tactilerobot/tactile_events.csv")
csv_filename = os.path.join(os.getcwd(), "tactile_events_skin.csv")

# Serial configuration (can change if Linux or Mac)
# the port can change for each reconnexion.
port1 = "/dev/ttyUSB0"  # Change this to your COM ports
port2 = "/dev/ttyUSB1"  # Change this to your COM ports

baud_rate = 250000
display = 1
if display:
    import cv2

calibration_time = .5  # seconds
threshold = 100 # threshold for detecting tactile events

# Define the size of the buffer
N = 5
# Initialize a buffer to store values below the threshold
threshold_buffer = [[] for _ in range(252)]

try:
    ser = serial.Serial(port1, baud_rate, timeout=1)
    port=port1
except:
    try:
        ser = serial.Serial(port2, baud_rate, timeout=1)
        port=port2
    except:
        print("No port detected")

def init_skin():
    print("Initializing Skin ...\n")
    #ser.write(b'init\n')  # Send command to initialize skin
    #ser.write(b'data\n')
    while True:
        try:
            response = ser.readline().decode('utf-8').strip().split(',')
        except UnicodeDecodeError:
            response = ser.readline().decode('latin-1').strip().split(',')
        if len(response)==252:
            print("\rInitialization successful\n")
            break
        else:
            print("\rInitialization failed, received len", len(response), "\n")

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
def get_tactile(values,buffer):
    # this function the mean
    values_mean = [[] for _ in range(252)]
    # Compute the mean value
    for i in range(252):
        values_mean[i]=np.mean(buffer[i])
    #compute touch events
    abs_diff = np.subtract(values_mean,values) #get the activation -> a press decreases the touch value
    #abs_diff[abs_diff<0]=0

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
    isTouch = np.any(values1d > threshold) #flag if touch
    # Create a 21 by 12 grid

    return isTouch, values1d, values_2d, values_mean, buffer

def get_touch_position(values_2d):
    if np.all(values_2d == 0):
        return None
    # Find the indices of the maximum value in the flattened array
    max_index_flat = np.argmax(values_2d)

    # Convert the flat index to 2D coordinates
    index_touch_2d = np.unravel_index(max_index_flat, values_2d.shape)
    return index_touch_2d
def pressure2touch(values1d):
    values_2d=np.reshape(values1d, (21, 12)).astype(np.uint8).T
    vmax_pre=np.max(values_2d)
    max_index_flat = np.argmax(values_2d)
    max_index_2d = np.unravel_index(max_index_flat, (12,21)) #or 12,21?
    #print('flat',id_block(max_index_flat))
    # touch = np.zeros(252,)
    # touch[max_index_flat]=1
    # o_pre = torch.tensor(touch).float() #get observation

    #block_id_2d=[np.maximum((max_index_2d[0]-1),0)//3,np.maximum((max_index_2d[1]-1),0)//3]
    block_id_2d=[max_index_2d[0]//2,max_index_2d[1]//2]
    #block_id_flat=block_id_2d[0]*7+block_id_2d[1]
    block_id_flat=block_id_2d[0]*10+block_id_2d[1]
    print('\rblobk',block_id_2d,'id',max_index_2d,'idlin',block_id_flat)
    #print('block_touch', block_touch_pre, 'touche ', touch_pre)
    #touch = np.zeros(28,)
    touch = np.zeros(60,)
    touch[block_id_flat]=1
    return block_id_flat,block_id_2d,touch

def index2cart(touch_2d_index):
    #from id to mm
    if not touch_2d_index:
        return None
    touch_2d_cart= (touch_2d_index[0] * 0.089, touch_2d_index[1] * 0.0873)
    return touch_2d_cart #reutnr in mm

# Initialize skin
init_skin()

# windows parameters
scale_percent = 5000
img = np.zeros((12, 21))
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

# Variables for storing data over time
data_buffer = []
time_buffer = []
#time_step = 1 / 36.0  # Time step in seconds
time_step = 1 / 12.56  # Time step in seconds
time = 0 - time_step

# claibration
flag = True

try:
    # Initialize curses
    stdscr = curses.initscr()
    curses.cbreak()  # Disable line buffering
    stdscr.keypad(True)  # Enable keypad mode
    #Set stdin to non-blocking mode
    fd = sys.stdin.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    # Open the CSV file in write mode and create a CSV writer object
    with open(csv_filename, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)

        # Write the header row
        csv_writer.writerow(['Computer Time (ms)', 'touch_row','touch_col','type'])
        while True:
            # Read data until '\r\n'
            #raw_data = ser.readline().decode().strip()
            raw_data = get_raw_data()
            time+=time_step
            # Check if data is not empty
            if raw_data and len(raw_data) == 252:
                try:
                    # Split data into individual values and convert to integers
                    values = [int(value) for value in raw_data]
                except ValueError:
                    # If conversion to integers fails, continue reading from serial until valid data is received
                    print("\rInvalid data received. Waiting for valid data...\n")
                    continue

                if time< calibration_time:
                    print(f"\rCalibration left: {calibration_time - time:.2f} s\n")
                    data_buffer.append(values)
                else:
                    if flag: #compute activation mean to filter the data
                        data_mean = np.mean(data_buffer, axis = 0)
                        for i in range(252):
                            for j in range(N):
                                threshold_buffer[i].append(data_mean[i])

                        print("mean_data: ", data_mean,"\n")
                        flag=False
                        # start the timer
                        starting_time_ms = int(datetime.now().timestamp() * 1e3)
                        print('\rEnd Calibration\n')


                    # abs_diff = -np.subtract(values,data_mean)/5
                    # abs_diff[abs_diff<0]=0
                    # below_threshold_indices = np.where(abs_diff < threshold)[0]

                    # # Update previous mean buffers for values not above the threshold
                    # for i in below_threshold_indices:
                    #     threshold_buffer[i].append(values[i])
                    #     if len(threshold_buffer[i]) > N:
                    #         threshold_buffer[i].pop(0)
                    #     data_mean[i]=np.mean(threshold_buffer[i])
                    # abs_diff = -np.subtract(values,data_mean)/5
                    # abs_diff[abs_diff<0]=0
                    # values_2d = np.reshape(abs_diff, (21, 12)).astype(np.uint8).T

                    # values_2d[values_2d < threshold] = 0  # Apply thresholding
                    #above_threshold_indices = np.where(values_2d < 0)

                    isTouch, values1d, values_2d, values_mean, threshold_buffer = get_tactile(values,threshold_buffer)
                    block_id_flat,block_id_2d, target_sensation = pressure2touch(values1d)
                    if np.max(values_2d)>threshold:
                        print("\rReceived touch at", block_id_2d)

            #         if xy:
            #             print("\rTouched position:", xy, "cm\n")
            #             current_time_ms = int(datetime.now().timestamp() * 1e3)
            #             csv_writer.writerow([current_time_ms-starting_time_ms, touch_2d[0], touch_2d[1], 2])
            #         #values_2d[values_2d >= threshold] = 255  # Apply thresholding

            #         # Display the image using OpenCV
                    if display:
                        resized = cv2.resize(values_2d, dim, interpolation=cv2.INTER_AREA)
                        cv2.imshow('Skin Patch', cv2.applyColorMap(resized, cv2.COLORMAP_VIRIDIS))
                        cv2.waitKey(1)

            #         try:
            #             key = stdscr.getch()
            #             if key == curses.KEY_ENTER or key == 10:  # Enter key for calibration
            #                 print("\rCalibration pressed.")
            #                 time = 0 - time_step  # Reset time for calibration
            #                 flag = True  # Set flag for recalibration
            #                 data_buffer = []
            #             elif key == 27:  # Escape key to exit
            #                 break
            #             elif key == ord('+'):  # Plus key to increase threshold
            #                 threshold += 5
            #                 print("\rThreshold increased to:", threshold)
            #             elif key == ord('-'):  # Minus key to decrease threshold
            #                 threshold -= 5
            #                 print("\rThreshold decreased to:", threshold)
            #         except curses.error:
            #             pass

            # else:
            #     data_buffer=[]
            #     time=0 - time_step
            #     print("Received invalid data length:", len(raw_data),"\n")


except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    # Close curses
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()
    # Close the serial port
    ser.close()
    #cv2.destroyAllWindows()
