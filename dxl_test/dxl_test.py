from dynamixel import dynamixel
import time

dx = dynamixel(device_name='/dev/ttyUSB0', protocol_version=2.0)

for i in 4,5,6,7:
    dx.enable_torque(i, True)
    dx.enable_led(i, True)
    
time.sleep(3.0)


# dx.set_pos(4,1530)
# dx.set_pos(4,2070)

dx.set_pos(5,2431)
dx.set_pos(6,2709)
time.sleep(2.0)

dx.set_pos(5,1924)
dx.set_pos(6,2185)
time.sleep(2.0)

dx.set_pos(5,2900)
dx.set_pos(6,3184)
time.sleep(2.0)

dx.set_pos(5,2628)
dx.set_pos(6,2497)
dx.set_pos(7,2080)
time.sleep(2.0)

dx.set_pos(5,2161)
dx.set_pos(6,2928)
dx.set_pos(7,2500)
time.sleep(2.0)


for i in 4,5,6,7:
    dx.enable_torque(i, False)
    dx.enable_led(i, False)