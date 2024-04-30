# main.py
from skin_sensor import SkinSensor
from robot_controller import RobotController

def main():
    skin_sensor = SkinSensor(port="/dev/ttyUSB1", baud_rate=250000)
    robot_controller = RobotController(port="/dev/ttyACM1", baud_rate=115200)

    # Initialize components
    skin_sensor.init()
    robot_controller.init()

    try:
        while True:
            # Your main program logic here
            pass
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        # Cleanup code here
        skin_sensor.close()
        robot_controller.close()

if __name__ == "__main__":
    main()
