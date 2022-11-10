import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

mass = np.array([8.393, 2.275])
m1 = 8.393
m2 = 2.275

p_com = np.array([[0.0, 0.0, 0.28],[0.0, 0.0, 0.25]])
lc1 = 0.28
lc2 = 0.25

Is = []
Is.append([[0.22689067591, 0.0,           0.00],
           [0.0,           0.22689067591, 0.00],
           [0.0,           0.0,           0.0151074]])

Is.append([[0.049443313556, 0.0,            0.0],
           [0.0,            0.049443313556, 0.0],
           [0.0,            0.0,            0.004095]])

# 소수점 셋째수까지 출력
np.set_printoptions(formatter={'float_kind':lambda x:"{0:0.3f}".format(x)})

def calc_mass_mat(q):
    s1 = np.sin(q[0])
    s2 = np.sin(q[0])
    c1 = np.cos(q[1])
    c2 = np.cos(q[1])
    s12 = np.sin(q[0]+q[1])
    c12 = np.cos(q[0]+q[1])

    M = np.array([[m1*lc1*lc1 + m2*(lc1*lc1 + 2*lc1*lc2*c2 + lc2*lc2)+Is[0][2][2]+Is[1][2][2], m2*(lc1*lc2*c2+lc2*lc2)+Is[1][2][2]],
                  [m2*(lc1*lc2*c2+lc2*lc2)+Is[1][2][2], m2*lc2*lc2+Is[1][2][2]]])

    return M

def calc_coriolis_mat(q, q_dot):

    return

def calc_gravity_mat(q):

    return

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
robot_id = p.loadURDF("urdf/ur5.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_INERTIA_FROM_FILE)
# robot_id = p.loadURDF("urdf/ur5.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

dof = p.getNumJoints(robot_id)
joints = range(dof-1)
print("Number of joints: {}".format(dof))

# Reset states controller
p.resetJointState(robot_id, 0, targetValue=0)
p.resetJointState(robot_id, 1, targetValue=0)
p.setJointMotorControlArray(bodyUniqueId=robot_id,
                            jointIndices=range(2),
                            controlMode=p.POSITION_CONTROL,
                            forces=[0. for _ in range(2)])

# Perform simulation step
while True:
    joint_states = p.getJointStates(robot_id, range(dof))
    pos = [state[0] for state in joint_states]
    vel = [state[1] for state in joint_states]
    acc_des = [0. for _ in pos]
    vel_des = [0. for _ in vel]

    pos_ee, ori_ee, _, _, _, _ = p.getLinkState(robot_id,2)
    q = np.array([joint_states[1][0], joint_states[2][0]])
    # print(pos)
    
    M_sim = np.array(p.calculateMassMatrix(robot_id, pos))
    M_cal = calc_mass_mat(q)
    # print(M_sim-M_cal)
    
    # C_sim = np.array(p.calculateInverseDynamics(robot_id, pos, vel, acc_des))
    
    # print(C_sim)
    # G_sim = np.array(p.calculateInverseDynamics(robot_id, pos, vel_des, acc_des))
    # print(G_sim)

    #calc_mass_mat
    
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)

# # Exit Simulation
p.disconnect()
print("Simulation end")