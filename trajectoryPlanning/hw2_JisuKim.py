#!/usr/bin/env python3
import rospy
import numpy as np
import time
from sensor_msgs.msg import JointState
from pandaKinematics import pandaKinematics
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})

myRate = 100

class pandaROS:
    def __init__(self):
        # create node
        if not rospy.get_node_uri():
            rospy.init_node('panda_ros_node', anonymous=True, log_level=rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        # member variables
        self.joints = []
        self._joint_names_panda = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                   'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']

        # ROS publisher
        self._set_joint_pub = rospy.Publisher('/panda/joint_states', JointState, latch=True, queue_size=1)

        # ROS subscriber
        sub_topic = '/joint_states'
        rospy.Subscriber(sub_topic, JointState, self.joint_states_callback, queue_size=1)
        rospy.wait_for_message(topic=sub_topic, topic_type=JointState)

    """
    Get Joint Positions
    """
    def joint_states_callback(self, msg):
        self.joints = list(msg.position)    # exclude both finger joints
    """
    Set Joint Positions
    """
    def set_joint_position(self, s):     # Set Joint Angles of Panda robot
        msg = JointState()
        msg.name = self._joint_names_panda
        msg.position = s
        self._set_joint_pub.publish(msg)

    def linearTrajectory(self, q0, qf, t_end):
        # q = a1*t + a0
        print('Linear Trajectory')
        timeStep = 1.0/myRate
        #print('%.3f' %panda.joints[0])
        linTraj = np.arange(0,t_end,timeStep)
        A = np.array([[0,1],[t_end,1]])
        b = np.array([q0, qf])
        x = np.linalg.inv(A) @ b
        Trj = np.zeros((linTraj.size,6))

        for i in range(6):
            for tt in range(linTraj.size):
                Trj[tt,i] = x[0,i]*linTraj[tt] + x[1,i]
        
        return Trj

    def cubicTrajectory(self, q0, qf, t_end):
        # q = a2*t^2 + a1*t + a0
        print('Cubic Trajectory')

        timeStep = 1.0/myRate
        #print('%.3f' %panda.joints[0])
        linTraj = np.arange(0,t_end,timeStep)
        A = np.array([[0,1],[t_end,1]])
        b = np.array([q0, qf])
        x = np.linalg.inv(A) @ b
        Trj = np.zeros((linTraj.size,6))

        for i in range(6):
            for tt in range(linTraj.size):
                Trj[tt,i] = x[0,i]*linTraj[tt] + x[1,i]
        
        return Trj
        
    def LSPBTrajectory(self, q0, qf, t_end):
        
        print('LSPB Trajectory')


if __name__ == "__main__":
    panda = pandaROS()
    joints = [0.0, -0.702, -0.678, -2.238, -0.365, 0.703, 0.808, 0.0, 0.0]

    q0 = [-0.12, 0.30, 0.18, -2.26, 1.58, 1.64, -1.77, 0.00, 0.00] # 0.5, 0.13, 0.26
    qf = [ 0.41, 0.50, 0.38, -1.68, 2.02, 2.28, -1.60, 0.00, 0.00] # 0.5, 0.49, 0.37
    
    panda.set_joint_position(q0)
    time.sleep(1)

    t = 0.0
    while not rospy.is_shutdown():
        # joints[0] = 1.5*np.sin(2*np.pi*t/10)
        # panda.set_joint_position(joints)
        # t += 0.01
        # time.sleep(0.01)

        linTraj = panda.linearTrajectory(q0, qf, 3)
        for i in range(linTraj.shape[0]):
            panda.set_joint_position(linTraj[i,:])
            time.sleep(0.01)

        linTraj = panda.linearTrajectory(qf, q0, 3)
        for i in range(linTraj.shape[0]):
            panda.set_joint_position(linTraj[i,:])
            time.sleep(0.01)
        
