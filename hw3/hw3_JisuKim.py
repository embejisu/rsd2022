import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

# Environment setup
SAMPLING_RATE = 1e-2    # 0.01s = 10 ms
physics_client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(SAMPLING_RATE)  # 1000Hz sampling rate
p.setGravity(0, 0, -9.81)
# p.setGravity(0, 0, 0)

# Setup plane
plane_id = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath(os.path.dirname(__file__) + '/ur_description')
# Setup objects
StartPos = [0, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0,0,0])
robot_id = p.loadURDF("urdf/ur5.urdf", StartPos, StartOrientation, useFixedBase=1,flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_INERTIA_FROM_FILE)

dof = p.getNumJoints(robot_id)-1
joints = range(dof)
# print("Number of joints: {}".format(numJoints))

# Reset states controller
p.resetJointState(robot_id, 0, targetValue=0)
p.resetJointState(robot_id, 1, targetValue=0)
p.setJointMotorControlArray(bodyUniqueId=robot_id,
                            jointIndices=range(2),
                            controlMode=p.POSITION_CONTROL,
                            forces=[0. for _ in range(2)])

# Perform simulation step
while True:
    joint_states = p.getJointStates(robot_id, joints)
    pos = [state[0] for state in joint_states]
    vel = [state[1] for state in joint_states]
    
    pos_ee, ori_ee, _, _, _, _ = p.getLinkState(robot_id,5)
    
    M_sim = np.array(p.calculateMassMatrix(robot_id, pos[:5]))
    print(M_sim)
    print()
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)
# # Exit Simulation
p.disconnect()
print("Simulation end")