import os


DEV_PATH = '/dev'


device_names = os.listdir(DEV_PATH)

print('Connect the robotic arm and press a key')
input()

device_names_new = os.listdir(DEV_PATH)
new_devices = sorted(list(set(device_names_new) - set(device_names)))

arm_port = new_devices[-1]

with open('arm_port.txt', 'w') as file:
    file.write(os.path.join(DEV_PATH, arm_port))

device_names = device_names_new

print('Connect the tactile skin and press a key')
input()

device_names_new = os.listdir(DEV_PATH)
new_devices = sorted(list(set(device_names_new) - set(device_names)))

skin_port = new_devices[-1]

with open('skin_port.txt', 'w') as file:
    file.write(os.path.join(DEV_PATH, skin_port))