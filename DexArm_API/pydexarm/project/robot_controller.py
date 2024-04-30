# robot_controller.py
from pydexarm import Dexarm

class RobotController:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.dexarm = None

    def init(self):
        # Initialize Dexarm robot
        try:
            self.dexarm = Dexarm(port=self.port)
            print("Robot initialized successfully.")
        except Exception as e:
            print(f"Error initializing robot: {e}")

    def move_to(self, position):
        # Move robot to specified position
        if self.dexarm:
            try:
                self.dexarm.move_to(*position, feedrate=4000, mode="G1", wait=True)
                print(f"Robot moved to position: {position}")
            except Exception as e:
                print(f"Error moving robot: {e}")
        else:
            print("Robot not initialized. Call init() first.")

    def go_home(self):
        # Move robot to home position
        if self.dexarm:
            try:
                self.dexarm.go_home()
                print("Robot moved to home position.")
            except Exception as e:
                print(f"Error moving robot to home position: {e}")
        else:
            print("Robot not initialized. Call init() first.")

    def close(self):
        # Close serial connection
        if self.dexarm:
            self.dexarm.close()

    # Other methods as needed
