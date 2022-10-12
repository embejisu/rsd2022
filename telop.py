#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import subscriber_node

# create node
if not rospy.get_node_uri():
    rospy.init_node('teleop_node', anonymous=True, log_level=rospy.WARN)
else:
    rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

m=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
s=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# s=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

joint_names_geomagic = ['m1', 'm2', 'm3', 'm4', 'm5', 'm6']
joint_names_panda = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']

def _js_callback(msg):
    global m, s
    if msg.name == joint_names_geomagic:
        m=list(msg.position)
    elif msg.name == joint_names_panda:
        s=list(msg.position)


def mapping(m):
    s = m
    return s

def set_joint_position(s):     # Set Joint Angles of Panda robot
    msg = JointState()
    msg.name = joint_names_panda
    msg.position = s
    teleop_pub.publish(msg)

def forward_kinematics_master(joint_m): #omni
    return TT #4x4 matrix

def forward_kinematics_slave(joint_s): #panda
    return TT #4x4 matrix

def inverse_kinematics_slave(T): #panda
    return ss #4x4 matrix

def mapping2(joint_m):
    return joint_s

o_sub = rospy.Subscriber("/omni/joint_states", JointState, _js_callback, queue_size=1, tcp_nodelay=True)
p_sub = rospy.Subscriber("/panda/joint_states", JointState, _js_callback, queue_size=1, tcp_nodelay=True)

teleop_pub = rospy.Publisher('/teleop/panda/joint_states', JointState, latch=True, queue_size=1)

while not rospy.is_shutdown():
    s = mapping(m)
    s = s + [0.0, 0.0, 0.0]
    set_joint_position(s)
    rospy.sleep(0.01)