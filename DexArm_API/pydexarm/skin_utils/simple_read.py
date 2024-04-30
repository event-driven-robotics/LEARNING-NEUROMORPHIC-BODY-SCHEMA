import serial
import numpy as np
import cv2
from scipy.signal import butter, filtfilt
from scipy.ndimage import convolve1d
# Configuration
port = "/dev/ttyUSB0"  # Change this to your COM port
baud_rate = 250000

# Create Serial object
ser = serial.Serial(port, baud_rate, timeout=1)

# Butterworth filter parameters
cut_frequency = 6.0  # Cut-off frequency in Hz
order = 10  # Filter order
fs=36.0
window_size = 3  # Set your desired window size
# Create an initial empty image
scale_percent = 5000
img = np.zeros((12, 21))
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
cv2.imshow('Skin Patch', resized)

# Create a Butterworth filter
b, a = butter(order, cut_frequency, fs=fs, btype='low', analog=False)
# Variables for storing data over time
data_buffer = []
time_buffer = []
time_step = 1 / 36.0  # Time step in seconds
time=0 - time_step

print('Calibration 3s')
flag=True
try:
    while True:
        # Read data until '\r\n'
        raw_data = ser.readline().decode().strip()
        time+=time_step
        # Check if data is not empty
        if raw_data:
            # Split data into individual values
            values = [int(value) for value in raw_data.split(',')]
            # Store data and time
            if len(values) == 252:
                if time<5:
                    data_buffer.append(values)
                else:
                    if flag:
                        data_mean = np.mean(data_buffer, axis = 0)
                        print("mean_data: ", data_mean)
                        flag=False
                        print('End Calibration')


                    data_buffer.append(values)
                    data_buffer.pop(0)
                    #print(np.subtract(data_buffer, data_mean))
                    data_time = (np.subtract(data_buffer, data_mean))# + 500 ) / 600 * 255

                    filtered_values = filtfilt(b, a, data_time, axis=0,  padlen=128)
                    #filtered_values = np.convolve(filtered_values, np.ones(100)/100, mode='valid')
                    moving_average_values = convolve1d(filtered_values, np.ones(window_size) / window_size, axis=0)
                    # Compute the rate of changes using time derivative
                    derivative_values = (np.abs(np.diff(moving_average_values, axis=0)))/60 * 255
                    # Check if the length of values is 252 (12 * 21)
                    # Reshape values into a 2D array (21 rows, 12 columns)
                    #print(data_time[-1])
                    #print((np.abs(np.diff(moving_average_values, axis=0)))[-1])
                    values_2d = np.reshape(derivative_values[-1], (21, 12)).astype(np.uint8).T

                    # Filter the values using Butterworth filter
                    #print(derivative_values)
                    # Display the image using OpenCV
                    resized = cv2.resize(values_2d, dim, interpolation=cv2.INTER_AREA)
                    cv2.imshow('Skin Patch',cv2.applyColorMap(resized, cv2.COLORMAP_VIRIDIS))

                    # Wait for a short duration (adjust as needed)
                    cv2.waitKey(10)

        else:
            print("Received invalid data length:", len(values))

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    # Close the serial port
    ser.close()
    cv2.destroyAllWindows()
