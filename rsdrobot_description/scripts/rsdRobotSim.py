import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

# Environment setup
SAMPLING_RATE = 1e-3  # 0.001s = 1 ms
physics_client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(SAMPLING_RATE)  # 1000Hz sampling rate
p.setGravity(0, 0, -9.81)

# Setup plane
plane_id = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath(os.path.dirname(__file__))
# Setup objects
StartPos = [0, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0,0,0])

robot_id = p.loadURDF('../urdf/rsdRobot.xacro',StartPos, StartOrientation, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
# robot_id = p.loadURDF('../urdf/rsdRobot.xacro')

while True:
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)

# Exit Simulation
p.disconnect()
print("Simulation end")