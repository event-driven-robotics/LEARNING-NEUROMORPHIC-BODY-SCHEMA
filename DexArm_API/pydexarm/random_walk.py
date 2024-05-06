from pydexarm import Dexarm
import time
import random

'''windows'''
#dexarm = Dexarm(port="ttyACM0")
'''mac & linux'''
dexarm = Dexarm(port="/dev/ttyACM0")
#help(dexarm)

dexarm.go_home()

# Define initial position
current_position = [100, 300, -100]

dexarm.move_to(*current_position, feedrate=20000, mode="G0")
# Define the maximum step size for each axis
max_step_size = 5

# Loop for random walk
for _ in range(20):
    # Generate random displacements for each axis
    dx = random.randint(-max_step_size, max_step_size)
    dy = random.randint(-max_step_size, max_step_size)

    # Update current position with the random displacements
    current_position[0] += dx
    current_position[1] += dy

    # Move the DexArm to the updated position
    dexarm.move_to(*current_position, feedrate=2000, mode="G1")

    # Pause for a short time before the next move
    time.sleep(0.0000001)
dexarm.go_home()

'''DexArm sliding rail Demo'''
'''
dexarm.conveyor_belt_forward(2000)
time.sleep(20)
dexarm.conveyor_belt_backward(2000)
time.sleep(10)
dexarm.conveyor_belt_stop()
'''

'''DexArm sliding rail Demo'''
'''
dexarm.go_home()
dexarm.sliding_rail_init()
dexarm.move_to(None,None,None,0)
dexarm.move_to(None,None,None,100)
dexarm.move_to(None,None,None,50)
dexarm.move_to(None,None,None,200)
'''
dexarm.close()
