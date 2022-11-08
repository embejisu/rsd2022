## Author: Jisu Kim(202133015)
## Date: 6 Oct., 2022
## Class: [RT750] Robot System Design

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omniKinematics import omniKinematics
from pandaKinematics import pandaKinematics

if not rospy.get_node_uri():
    rospy.init_node('teleop_node', anonymous=True, log_level=rospy.WARN)
else:
    rospy.logdebug(rospy.get_caller_id() + '-> ROS already initialized')

m = [0.0,0.0,0.0,0.0,0.0,0.0]
s = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

joint_names_geomagic = ['m1', 'm2', 'm3', 'm4', 'm5', 'm6']
joint_names_panda = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2']

def _js_callback_m(msg):
    global m
    m = list(msg.position)

def _js_callback_s(msg):
    global s
    s = list(msg.position)

def forward_kinematics_master(joint_m): # Master device, omni
    T = omniKinematics.fk(joints=joint_m)
    return T # 4x4 matrix

def forward_kinematics_slave(joint_s): # Slave robot, Panda
    T = pandaKinematics.fk(joints=joint_s)
    return T # 4x4 matrix

def inverse_kinematics_slave(T):
    joint_s = pandaKinematics.ik(T)
    return joint_s

def mapping(joint_m, joint_s):
    Tm = forward_kinematics_master(joint_m)
    pos = Tm[:3, -1]
    pos *= 5.0
    pos += [0.5, 0.8, 0.3]
    Tm[:3, -1] = pos

    s_new = pandaKinematics.ik(Tb_ed=Tm, q0=joint_s[:7], RRMC=True)
    s_new_null = pandaKinematics.null_space_control(joints=joint_s[:7], crit='joint_limit')
    return s_new #+ s_new_null

teleop_pub = rospy.Publisher('/teleop/panda/joint_states',JointState, latch=True, queue_size=1)
p_sub = rospy.Subscriber('/panda/joint_states',JointState,_js_callback_s,queue_size=1,tcp_nodelay=True)
o_sub = rospy.Subscriber('/omni/joint_states',JointState,_js_callback_m,queue_size=1,tcp_nodelay=True)
rate = rospy.Rate(100) # 100Hz

# main loop
while not rospy.is_shutdown():
    if m == [] or s == []:
        pass
    else:
        myS = mapping(m,s)
        myMsg = JointState()
        myMsg.name = joint_names_panda
        myMsg.position = myS
        teleop_pub.publish(myMsg)
        Tm = forward_kinematics_master(m)
        #print ("Omni_fk=", omniKinematics.fk(m))
        #print ("Panda_ik=", pandaKinematics.ik(Tm))
        #print ("Panda_fk=\n", pandaKinematics.fk(joints=s[:7])[0][-1])
        ss = np.array(s)
        #print(ss)
        #print ('\n')
        rate.sleep()
