import serial #serial can be imported using pip/pip3/conda install pyserial
import numpy as np

import os
import csv

from datetime import datetime, timedelta  # Import datetime module for getting the current time

#csv_filename ="~/your_git_repo_name/tactilerobot/sm_events.csv"
# Append the formatted time to the file name
csv_filename = os.path.join(os.getcwd(), "tactile_events_skin.csv")

# Serial configuration (can change if Linux or Mac)
# the port can change for each reconnexion.
port1 = "/dev/ttyUSB0"  # Change this to your COM ports
port2 = "/dev/ttyUSB1"  # Change this to your COM ports

baud_rate = 250000

calibration_time = .5  # seconds
threshold = 20 # threshold for detecting tactile events

# Define the size of the buffer
N = 10
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
    data = ser.readline().decode().strip()
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
    touch_2d_cart= (touch_2d_index[0] * 0.089, touch_2d_index[1] * 0.0873)
    return touch_2d_cart #reutnr in mm

# Initialize skin
init_skin()

# Variables for storing data over time
data_buffer = []
time_buffer = []
#time_step = 1 / 36.0  # Time step in seconds
time_step = 1 / 12.56  # Time step in seconds
time = 0 - time_step

# claibration
flag = True

try:

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

                        print('\rEnd Calibration\n')
                        starting_time_ms = int(datetime.now().timestamp() * 1e3)
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
                    above_threshold_indices = np.where(values_2d < 0)

                    touch_2d = get_touch_position(values_2d)
                    xy = index2cart(touch_2d)
                    if xy:
                        print("\rTouched position:", xy, "cm\n")
                        current_time_ms = int(datetime.now().timestamp() * 1e3)
                        csv_writer.writerow([current_time_ms-starting_time_ms, touch_2d[0], touch_2d[1], 2])
                    #values_2d[values_2d >= threshold] = 255  # Apply thresholding

            else:
                data_buffer=[]
                time=0 - time_step
                print("Received invalid data length:", len(raw_data),"\n")


except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    ser.close()
