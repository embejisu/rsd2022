#!/usr/bin/env python

import numpy as np

class robotKinmatics:
    #                     a,    alpha,   d,   theta
    # dhParams = np.array([[70.0, np.pi/2, 130.3, 0.0],
    #                      [203.5, 0.0, 0.0, np.pi/2],
    #                      [174.6, 0.0, 0.0, -np.pi/2],
    #                      [35.6, -np.pi/2, 0.0, 0.0],
    #                      [0.0, np.pi/2, 0.0, np.pi/2],
    #                      [0.0, 0.0, 75.0, 0.0]])
    dhParams = np.array([[70.0, np.pi/2, 130.3, 0.0],
                         [203.5, 0.0, 0.0, 0],
                         [174.6, 0.0, 0.0, 0],
                         [35.6, -np.pi/2, 0.0, 0.0],
                         [0.0, np.pi/2, 0.0, 0],
                         [0.0, 0.0, 75.0, 0.0]])


    def DH_transform(self, q):
        for i in range(6):
            self.dhParams[i][3] += q[i]
    
    def fk(self, q):
        a = self.dhParams[:,3]
        return a

if __name__ == "__main__":
    myRobot = robotKinmatics()

    joints = np.array([[1, 2, 3, 4, 5, 6]])
    myRobot.DH_transform(joints)

    print(myRobot.fk(joints))