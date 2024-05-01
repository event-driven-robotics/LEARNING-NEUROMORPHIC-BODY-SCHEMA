from pydexarm import Dexarm
import time

'''windows'''
#dexarm = Dexarm(port="ttyACM0")
'''mac & linux'''
dexarm = Dexarm(port="/dev/ttyACM0")
#help(dexarm)

dexarm.go_home()


dexarm.move_to(100, 300, -80,feedrate=5000,mode="G1")


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
