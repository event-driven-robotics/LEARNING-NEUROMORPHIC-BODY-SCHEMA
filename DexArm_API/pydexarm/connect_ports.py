import serial
import threading
from pydexarm import Dexarm
import numpy as np
# Configuration
# for history of port connections run this command in console >>>dmesg | grep tty
port_skin = "/dev/ttyUSB0"  # Change this to your COM port
port_robot = "/dev/ttyACM0"  # Change this to your second COM port
baud_rate_robot = 115200  # Change to the baud rate of port1
baud_rate_skin = 250000  # Change to the baud rate of port2
ser_skin = serial.Serial(port_skin, baud_rate_skin, timeout=1)
ser_robot = serial.Serial(port_robot, baud_rate_robot, timeout=1)

try:
    dexarm = Dexarm(port="/dev/ttyACM0")
    port_robot = "/dev/ttyACM0"
except:
    try:
        dexarm = Dexarm(port="/dev/ttyACM1")
        port_robot = "/dev/ttyACM1"
    except:
        print('Robot is probably not connected or powered')
try:
    ser_skin = serial.Serial("/dev/ttyUSB0", baud_rate_skin, timeout=1)
    port=port1
except:
    try:
        ser_skin = serial.Serial("/dev/ttyUSB1", baud_rate_skin, timeout=1)
        port=port2
    except:
        print("No port detected")

# Buffer for storing data
data_buffer_robot = []
data_buffer_skin = []


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
def get_raw_data():
    #ser_skin.write(b'data\n')  # Send command to request raw data
    data = ser_skin.readline().decode().strip()
    return data.split(',')  # Split the received data by comma

def get_mean_skin_activation():
    #ser_skin.write(b'data\n')  # Send command to request raw data
    print("Get mean activation")
    calibration_buffer=[]
    for i in range(20):
        calibration_buffer.append(ser_skin.readline().decode().strip().split(','))
    return np.mean(calibration_buffer)  # Split the received data by comma

def read_serial_skin():#mean_skin_act):
    #ser_skin.write(b'data\n')  # Send command to request raw data
    while True:
        raw_data = get_raw_data()
        #print("Device 1:", raw_data)
        if raw_data and len(raw_data) == 252:
                    try:
                        # Split data into individual values and convert to integers
                        values = [int(value) for value in raw_data]
                        data_buffer_skin.append(values)#-mean_skin_act)
                    except ValueError:
                        # If conversion to integers fails, continue reading from serial until valid data is received
                        print("Invalid data received. Waiting for valid data...")
                        values=None
                        continue
        else:
            values=None
            print("Received invalid data length:", len(raw_data))

# Function to read data from serial port
def read_serial_robot():

    while True:
        #raw_data = ser_robot.readline().decode().strip()
        raw_data = dexarm.get_current_position()
        #print("Device 2:", raw_data)
        # Add data to buffer
        data_buffer_robot.append(raw_data)


# Function to display synchronized data
def display_synchronized_data(data_buffer1, data_buffer2):
    while True:
        if data_buffer1 and data_buffer2:
        #if data_buffer2:
            # Display synchronized data
            if len(data_buffer1)<11:
                pass
            else:
                for i in range(len(data_buffer1)-11):
                    data_buffer1.pop(0)
                idd = np.argmax(np.abs(data_buffer1[-1]-np.mean(data_buffer1)))
                print("Device 1:", idd,len(data_buffer1))

            if len(data_buffer2)<11:
                pass
            else:
                for i in range(len(data_buffer2)-11):
                    data_buffer2.pop(0)
                #print("Device 2:", data_buffer2[-1],len(data_buffer2))






# Initialize skin
init_skin()
#mean_skin_act = get_mean_skin_activation()
# Create threads for reading from each port
thread_skin = threading.Thread(target=read_serial_skin)#, args=mean_skin_act)
thread_robot = threading.Thread(target=read_serial_robot)
thread_display = threading.Thread(target=display_synchronized_data, args=(data_buffer_skin, data_buffer_robot))
# Start both threads
thread_skin.start()
thread_robot.start()
thread_display.start()

# Wait for both threads to finish
thread_skin.join()
thread_robot.join()
thread_display.join()
# Now you have synchronized data in data_buffer1 and data_buffer2
# You can process them further as needed
