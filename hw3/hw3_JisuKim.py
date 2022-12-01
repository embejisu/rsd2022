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
l1 = 0.425

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
    c2 = np.cos(q[1])

    d11 = m1*lc1*lc1 + m2*(l1*l1 + lc2*lc2 + 2.0*l1*lc2*c2) + Is[0][1][1]+Is[1][1][1]
    d12 = d21 = m2*(lc2*lc2 + l1*lc2*c2) + Is[1][1][1]
    d22 = m2*lc2*lc2 + Is[1][1][1]

    M = np.array([[d11,d12],[d21,d22]])

    return M

def calc_coriolis_mat(q, q_dot):
    h = -m2*l1*lc2*np.sin(q[1])
    C = np.array([[h*q_dot[1], h*(q_dot[0]+q_dot[1])],[-h*q_dot[0], 0.0]])
    # print("C")
    # print(C)
    cal_toque = np.array([C[0][0]*q_dot[0] + C[0][1]*q_dot[1], C[1][0]*q_dot[0] + C[1][1]*q_dot[1]])
    return cal_toque

def calc_gravity_mat(q):
    g1 = (m1*lc1 + m2*l1)*np.cos(q[0]) + m2*lc2*np.cos(q[0]+q[1])
    g2 = m2*lc2*np.cos(q[0]+q[1])
    G = -9.81*np.array([g1,g2])
    return G

# Environment setup
SAMPLING_RATE = 1e-3  # 0.001s = 1 ms
physics_client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(SAMPLING_RATE)  # 1000Hz sampling rate
p.setGravity(0, 0, -9.81)

# Setup plane
plane_id = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath(os.path.dirname(__file__) + '/ur_description')
# Setup objects
StartPos = [0, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0,0,0])
robot_id = p.loadURDF("urdf/ur5.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)

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
    q = [state[0] for state in joint_states]
    q_dot = [state[1] for state in joint_states]
    acc_des = [0. for _ in q]
    vel_des = [0. for _ in q_dot]
    
    M_sim = np.array(p.calculateMassMatrix(robot_id, q[:2]))
    M_cal = calc_mass_mat(q)
    # print("M_sim")
    # print(M_sim)
    # print("M_cal")
    # print(M_cal)
    print("Diff_M")
    print(M_sim-M_cal)
    
    G_sim = np.array(p.calculateInverseDynamics(robot_id, q[:2], vel_des[:2], acc_des[:2]))
    G_cal = calc_gravity_mat(q)
    # print("G_sim")
    # print(G_sim)
    # print("G_cal")
    # print(G_cal)
    print("Diff_G")
    print(G_sim-G_cal)

    C_sim = np.array(p.calculateInverseDynamics(robot_id, q[:2], q_dot[:2], acc_des[:2])) - G_cal
    C_cal = calc_coriolis_mat(q,q_dot)
    # print("C_sim")
    # print(C_sim)
    # print("C_cal")
    # print(C_cal)
    print("Diff_C")
    print(C_sim-C_cal)
    
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)

# # Exit Simulation
p.disconnect()
print("Simulation end")