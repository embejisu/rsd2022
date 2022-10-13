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
        timeArr = np.arange(0,t_end,timeStep)
        A = np.array([[0,1],[t_end,1]])
        b = np.array([q0, qf])
        x = np.linalg.inv(A) @ b
        Trj = np.zeros((timeArr.size,9))

        for i in range(9):
            for tt in range(timeArr.size):
                Trj[tt,i] = x[0,i]*timeArr[tt] + x[1,i]
        return Trj

    def cubicTrajectory(self, q0, qf, v0, vf, t_end):
        # q = a2*t^2 + a1*t + a0
        print('Cubic Trajectory')

        timeStep = 1.0/myRate
        timeArr = np.arange(0,t_end,timeStep)
        zero = [0,0,0,0,0,0,0,0,0]
        A = np.array([[0,0,0,1],[t_end**3,t_end**2, t_end, 1],[0,0,1,0],[3*t_end**2,2*t_end,1,0]])
        b = np.array([q0, qf, v0, vf])
        x = np.linalg.inv(A) @ b

        Trj = np.zeros((timeArr.size,9))

        for i in range(9):
            for tt in range(timeArr.size):
                Trj[tt,i] = x[0,i]*timeArr[tt]**3 + x[1,i]*timeArr[tt]**2 + x[2,i]*timeArr[tt] + x[3,i]
        return Trj
        
    def LSPBTrajectory(self, q0, qf, t_b, t_end):
        print('LSPB Trajectory')
        timeStep = 1.0/myRate
        timeArr = np.arange(0,t_end,timeStep)
        Trj = np.zeros((timeArr.size,9))
        for i in range(timeArr.size):
            for j in range(9):
                v_max = 1.5*(qf[j]-q0[j])/t_end
                if timeArr[i] <= t_b:
                    Trj[i,j] = q0[j] + 0.5*(v_max/t_b)*(timeArr[i]**2)
                elif timeArr[i] <= (t_end-t_b):
                    Trj[i,j] = (qf[j] + q0[j] - v_max*t_end)/2.0 + v_max*timeArr[i]
                else:
                    Trj[i,j] = qf[j] - 0.5*(v_max/t_b)*(timeArr[i]-t_end)**2
        return Trj

    def cubic_with_waypoint(self, q0, qw1, qw2, qf):
        print('2 Waypoints Trajectory')
        v0 = np.zeros(9)
        vf = np.zeros(9)
        Trj = []
        
        traj1 = self.cubicTrajectory(q0, qw1, v0, vf, 3)
        traj2 = self.cubicTrajectory(qw1, qw2, v0, vf, 3)
        traj3 = self.cubicTrajectory(qw2, qf, v0, vf, 3)
        
        Trj.append(traj1)
        Trj.append(traj2)
        Trj.append(traj3)
        
        return Trj
    
if __name__ == "__main__":
    panda = pandaROS()
    q0  = [-0.12,  0.30, 0.18, -2.26,  1.58, 1.64, -1.77, 0.00, 0.00] # 0.5, 0.13, 0.26, 0, 0, 0
    qw1 = [-1.53, -2.90, 0.00, -4.41, -0.06, 0.15, -2.41, 0.00, 0.00] # 0.5, 0.10, 0.50, 0, 0, 0
    qw2 = [-1.42, -2.68, 0.06, -4.28,  0.01, 0.49, -2.41, 0.00, 0.00] # 0.5, 0.21, 0.45, 0, 0, 0
    qf  = [ 0.41, 0.50, 0.38, -1.68, 2.02, 2.28, -1.60, 0.00, 0.00] # 0.5, 0.49, 0.37, 0, 0, 0
    
    panda.set_joint_position(q0)
    time.sleep(1)

    t = 0.0
    while not rospy.is_shutdown():
        # linTraj = panda.linearTrajectory(q0, qf, 3)
        # for i in range(linTraj.shape[0]):
        #     panda.set_joint_position(linTraj[i,:])
        #     time.sleep(0.01)
        # linTraj = panda.linearTrajectory(qf, q0, 3)
        # for i in range(linTraj.shape[0]):
        #     panda.set_joint_position(linTraj[i,:])
        #     time.sleep(0.01)      

        # v0 = [0,0,0,0,0,0,0,0,0]
        # vf = [0,0,0,0,0,0,0,0,0]
        # cubicTraj = panda.cubicTrajectory(q0, qf, v0, vf, 3)
        # for i in range(cubicTraj.shape[0]):
        #     panda.set_joint_position(cubicTraj[i,:])
        #     time.sleep(0.01)
        # cubicTraj = panda.cubicTrajectory(qf, q0, v0, vf, 3)
        # for i in range(cubicTraj.shape[0]):
        #     panda.set_joint_position(cubicTraj[i,:])
        #     time.sleep(0.01)
        
        # lspbTraj = panda.LSPBTrajectory(q0,qf,1,3)
        # for i in range(lspbTraj.shape[0]):
        #     panda.set_joint_position(lspbTraj[i,:])
        #     time.sleep(0.01)

        # lspbTraj = panda.LSPBTrajectory(qf,q0,1,3)
        # for i in range(lspbTraj.shape[0]):
        #     panda.set_joint_position(lspbTraj[i,:])
        #     time.sleep(0.01)
        
        wpTraj = panda.cubic_with_waypoint(q0,qw1,qw2,qf)
        for i in range(wpTraj.shape[0]):
            panda.set_joint_position(wpTraj[i,:])
            time.sleep(0.01)