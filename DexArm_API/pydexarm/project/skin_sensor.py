# skin_sensor.py
import serial
import numpy as np

class SkinSensor:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None

    def init(self):
        # Initialize serial connection to the skin sensor
        try:
            self.ser.write(b'init\n')  # Send command to request raw data
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print("Skin sensor initialized successfully.")
        except Exception as e:
            print(f"Error initializing skin sensor: {e}")

    def get_raw_data(self):
        # Read raw data from the skin sensor
        if self.ser:
            try:
                self.ser.write(b'data\n')  # Send command to request raw data
                data = self.ser.readline().decode().strip()
                return data.split(',')  # Split the received data by comma
            except Exception as e:
                print(f"Error reading raw data from skin sensor: {e}")
        else:
            print("Skin sensor not initialized. Call init() first.")
            return []

    def close(self):
        # Close serial connection to the skin sensor
        if self.ser:
            self.ser.close()

    # Other methods as needed
