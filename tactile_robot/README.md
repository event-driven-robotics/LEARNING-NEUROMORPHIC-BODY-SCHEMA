Files to read inputs from tactile skin and send command to Dexarm robot arm.

-- robot* are principally robot code
-- skin* are principally skin code

### For the robot
Support for Dexarm robotics: https://github.com/Rotrics-Dev/DexArm_API
#### PORT NAMES
PORT NAMES on Mac:
to find the usb names: >> ls /dev/tty*

Robot port name:  dev/tty.usbmodem*

SKIN port name: dev/tty.usbserial*


PORT NAMES on Linux: 
to find the usb names: >> dmesg | grep tty

Robot port name: dev/ttyACM0 or dev/ttyACM1

SKIN port name: dev/ttyUSB0 or dev/ttyUSB1

!!! NOTE !!!! port name might switch at each deconnection that's why you can enter two different names to check every time

### Security principles

- Don't play with the z_touch height the skin is fragile and the robot strong a value to low can damage the skin foam. -85mm is set-up for best contact
- The PCB and the solderings are near the robot workspace. Make sure the movement of the robot do not make contact with the unit.
- Do not let the robot in contact with the foam on the same place for a long time -> turn it off and move it up with by hand.

###
Python3 code:
*robot* code:
- robot_example #simple code for using the go_home and move_to commands
- robot_manual_calibration.py #allow for calibrating the corners of the skin
- robot_random_touch.py #generating random linear trajectory in contact with the skin
- robot_touch_following.py #does the buzzer experience: e.g. reaching the touched position.

*skin* code:
skin_main.py #script for reading the data from the skin and printing/storing touched event. Include visualization and keystroke detection
skin_main_lite.py #same thing than skin_main without loading opencv, or keystroke detection.




