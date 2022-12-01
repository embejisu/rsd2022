#!/usr/bin/env python

import numpy as np
np.set_printoptions(formatter={'float_kind':lambda x:"{0:0.4f}".format(x)})

class robotKinematics:
    #                       a,        alpha,       d,      theta
    dhParams = np.array([[70.0,     np.pi/2,    130.3,       0.0],
                         [203.5,        0.0,      0.0,   np.pi/2],
                         [174.6,        0.0,      0.0,  -np.pi/2],
                         [35.6,     -np.pi/2,     0.0,       0.0],
                         [0.0,       np.pi/2,     0.0,   np.pi/2],
                         [0.0,           0.0,    75.0,       0.0]])

    @classmethod
    def fk(self, q):
        a = self.dhParams[:,0]
        alpha = self.dhParams[:,1]
        d = self.dhParams[:,2]
        theta = self.dhParams[:,3] + q

        Ts = np.zeros((7,4,4))
        Tbs = np.zeros((7,4,4))

        Ts[0,:,:] = np.identity(4)

        for i in range(6):
            Ts[i+1,:,:] = np.array([[np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],
                                   [np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.cos(theta[i])*np.sin(alpha[i]),  a[i]*np.sin(theta[i])],
                                   [0,                 np.sin(alpha[i]),               np.cos(alpha[i]),                 d[i]],
                                   [0,                        0,                            0,                            1]])

        Tbs[0,:,:] = np.identity(4)
        Tbs[1,:,:] = Ts[1,:,:]
        Tbs[2,:,:] = Tbs[1,:,:].dot(Ts[2,:,:])
        Tbs[3,:,:] = Tbs[2,:,:].dot(Ts[3,:,:])
        Tbs[4,:,:] = Tbs[3,:,:].dot(Ts[4,:,:])
        Tbs[5,:,:] = Tbs[4,:,:].dot(Ts[5,:,:])
        Tbs[6,:,:] = Tbs[5,:,:].dot(Ts[6,:,:])
        
        return Tbs, Ts

    @classmethod
    def computeJacobian(self, Tbs):
        J = np.zeros((6, 6))
        for i in range(6):
            zi = Tbs[i,:3,2]
            J[3:,i] = zi
            J[:3,i] = np.cross(zi,Tbs[6,:3,3]-Tbs[i,:3,3])

        return J

    @classmethod
    def ik(self, q_init, goalPos, RRMC=False):
        qk = q_init

        xf = goalPos[0]
        yf = goalPos[1]
        zf = goalPos[2]
        Rxf = goalPos[3]
        Ryf = goalPos[4]
        Rzf = goalPos[5]
        
        reached = False
        iter = 0
        while not reached:
            Tbs = robotKinematics.fk(q=qk)[0]
            x=Tbs[6][0][3]
            y=Tbs[6][1][3]
            z=Tbs[6][2][3]
            rx=np.arctan2(Tbs[6][3][2],Tbs[6][3][3])
            ry=np.arctan2(Tbs[6][3][1],np.sqrt(Tbs[6][3][2]*Tbs[6][3][2] + Tbs[6][3][3]*Tbs[6][3][3]))
            rz=np.arctan2(Tbs[6][2][1],Tbs[6][1][1])

            dx = np.array([xf-x, yf-y, zf-z, Rxf-rx, Ryf-ry, Rzf-rz])
            
            if np.linalg.norm(dx) < 0.1:
                reached = True
                print('Solved~!')
            else:
                iter+=1

            if iter==1000:
                reached = True
                print('Fail')

            if RRMC:
                reached = True

            J = robotKinematics.computeJacobian(Tbs)
            Jp = np.linalg.pinv(J)

            alpha=0.5

            qk_next = qk + Jp.dot(dx*alpha)
            qk = qk_next

        return qk