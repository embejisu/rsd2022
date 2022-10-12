import numpy as np

# Kinematics variables
L1 = 0.3330  # base ~ shoulder (m)
L2 = 0.3160  # shoulder ~ elbow (m)
L3 = 0.3840  # elbow ~ wrist (m)
L4 = 0.0880  # wrist offset (m)
L5 = 0.1070  # flange offset (m)
offset = 0.0825 # elbow offset
# Lg = 0.0584  # gripper body length (m)
# Lt = 0.045   # gripper tip length (m)
Lg = 0.0     # gripper body length (m)
Lt = 0.0     # gripper tip length (m)

def dhparam(joints):
    q1, q2, q3, q4, q5, q6, q7 = np.array(joints).T
    return np.array([[0, 0, L1, q1],
                     [-np.pi/2, 0, 0, q2],
                     [np.pi/2, 0, L2, q3],
                     [np.pi/2, offset, 0, q4],
                     [-np.pi/2, -offset, L3, q5],
                     [np.pi/2, 0, 0, q6],
                     [np.pi/2, L4, 0, q7],
                     [0, 0, L5, 0],
                     [0, 0, Lg+Lt, -np.pi/4]])

# Cartesian space limits
v_max = 1.7000  # trans vel (m/s)
a_max = 13.0000 # trans acc (m/s^2)
j_max = 6500.0000   # trans jerk (m/s^3)
v_max_rot = 2.5000  # rot vel (rad/s)
a_max_rot = 25.0000 # rot acc (rad/s^2)
j_max_rot = 12500.0000  # rot jerk (rad/s^3)
v_max_elbow = 2.1750    # max. vel of elbow (rad/s)
a_max_elbow = 10.0000   # max. acc of elbow (rad/s^2)
j_max_elbow = 5000.0000 # max. jerk of elbow (rad/s^3)

# Joint space limits
q_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])    # joint limit (rad)
q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])    # joint limit (rad)
qv_ratio = 0.8   # safety margin
qa_ratio = 0.8
qj_ratio = 0.8
qv_max = np.array([ 2.1750, 2.1750,  2.1750,  2.1720,  2.6100,  2.6100,  2.6100])*qv_ratio    # max. ang. vel (rad/s)
qa_max = np.array([15.0000, 7.5000, 10.0000, 12.5000, 15.0000, 20.0000, 20.0000])*qa_ratio    # max. ang. acc (rad/s^2)
qj_max = np.array([   7500,   3750,    5000,    6250,    7500,   10000,   10000])*qj_ratio    # max. ang. jerk (rad/s^3)
tau_max = np.array([87, 87, 87, 87, 12, 12, 12])    # max torque (Nm)
ror_max = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000])  # max roratum (Nm/s)