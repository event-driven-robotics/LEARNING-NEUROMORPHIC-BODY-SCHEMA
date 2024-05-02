Files to read inputs from tactile skin and send command to Dexarm robot arm.

-- robot* are principally robot code
-- skin* are principally skin code

###
Support for Dexarm robotics: https://github.com/Rotrics-Dev/DexArm_API

### Security principles

- Don't play with the z_touch height the skin is fragile and the robot strong a value to low can damage the skin foam. -85mm is set-up for best contact
- The PCB and the solderings are near the robot workspace. Make sure the movement of the robot do not make contact with the unit.
- Do not let the robot in contact with the foam on the same place for a long time -> turn it off and move it up with by hand.

###
Python3 code:
*robot* code:
- robot_example
- robot_manual_calibration.py
- robot_random_touch.py
- robot_touch_following.py

*skin* code:
skin_main.py




