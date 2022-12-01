#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from robotKinematics import robotKinematics

np.set_printoptions(formatter={'float_kind':lambda x:"{0:0.4f}".format(x)})

_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

q=[]

def _callback(msg):
    global q
    q = list(msg.position)
    # print(q)

def set_joint_position(joints):     # Set Joint Angles of Panda robot
    msg = JointState()
    msg.name = _joint_names
    msg.position = joints
    set_joint_pub.publish(msg)

if __name__ == "__main__":
    myRobot = robotKinematics()

    rospy.init_node('js_cmd_node', anonymous=True, log_level=rospy.WARN)
    rospy.Subscriber('/js/joint_states', JointState, _callback, queue_size=1)
    set_joint_pub = rospy.Publisher('/js/cmd/joint_states', JointState, latch=True, queue_size=1)    
    time.sleep(1)
    
    q0 = np.array([0,0,0,0,0,0])
    set_joint_position(q0)
    print('Initial Position')
    time.sleep(1)

    # # FK
    # Tbs, Ts = myRobot.fk(qr)
    # J = myRobot.computeJacobian(Tbs)

    # xf=Tbs[6][0][3]
    # yf=Tbs[6][1][3]
    # zf=Tbs[6][2][3]
    # Rxf=np.arctan2(Tbs[6][3][2],Tbs[6][3][3])
    # Ryf=np.arctan2(Tbs[6][3][1],np.sqrt(Tbs[6][3][2]*Tbs[6][3][2] + Tbs[6][3][3]*Tbs[6][3][3]))
    # Rzf=np.arctan2(Tbs[6][2][1],Tbs[6][1][1])
    # goalPos = np.array([xf,yf,zf,Rxf,Ryf,Rzf])
    # print(goalPos)

    # # IK
    # qq = np.zeros(6)
    # qk = myRobot.ik(qq,goalPos,RRMC=True)
    # print(qk)

    # # IK -> FK
    # Tbs, Ts = myRobot.fk(qk)
    # xf=Tbs[6][0][3]
    # yf=Tbs[6][1][3]
    # zf=Tbs[6][2][3]
    # Rxf=np.arctan2(Tbs[6][3][2],Tbs[6][3][3])
    # Ryf=np.arctan2(Tbs[6][3][1],np.sqrt(Tbs[6][3][2]*Tbs[6][3][2] + Tbs[6][3][3]*Tbs[6][3][3]))
    # Rzf=np.arctan2(Tbs[6][2][1],Tbs[6][1][1])
    
    goalPos = np.array([337.29, 0.0, 94.645])
    goalOri_d = np.array([0,0,0])
    goalOri_r = np.deg2rad(goalOri_d)
    
    myGoal = np.hstack((goalPos, goalOri_r))
    # print(goalPos)


    while not rospy.is_shutdown():
        qk = myRobot.ik(q,myGoal,RRMC=False)
        print(qk)
        set_joint_position(qk)
        time.sleep(0.01)
        