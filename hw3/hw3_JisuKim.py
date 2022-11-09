import pybullet as p
import pybullet_data
import time
import os

# Environment setup
SAMPLING_RATE = 1e-2    # 0.01s = 10 ms
physics_client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(SAMPLING_RATE)  # 1000Hz sampling rate
p.setGravity(0, 0, -9.81)
# p.setGravity(0, 0, 0)

# Setup plane
plane_id = p.loadURDF("plane.urdf")

# Setup objects
StartPos = [0, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0,0,0])
manipulator_id = p.loadURDF("./ur_description/urdf/ur5.urdf", StartPos, StartOrientation, useFixedBase=1)

numJoints = p.getNumJoints(manipulator_id) 
print("Number of joints: {}".format(numJoints))

# Reset states controller
p.resetJointState(manipulator_id, 0, targetValue=0)
p.resetJointState(manipulator_id, 1, targetValue=0)
p.setJointMotorControlArray(bodyUniqueId=manipulator_id,
                            jointIndices=range(2),
                            controlMode=p.POSITION_CONTROL,
                            forces=[0. for _ in range(2)])

# Perform simulation step
while True:
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(manipulator_id)

# # Exit Simulation
p.disconnect()
print("Simulation end")