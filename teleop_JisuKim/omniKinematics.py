import numpy as np
import omniVar


class omniKinematics:
    @classmethod
    def DH_transform(cls, alpha, a, d, theta, unit='rad'):  # modified DH convention
        if unit == 'deg':
            alpha = np.deg2rad(alpha)
            theta = np.deg2rad(theta)
        T = np.array([[np.cos(theta), -np.sin(theta), 0, a],
                      [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                       -np.sin(alpha) * d],
                      [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                      [0, 0, 0, 1]])
        return T

    @classmethod
    def fk(cls, joints, L1=omniVar.L1, L2=omniVar.L2, L3=omniVar.L3):
        q1, q2, q3, q4, q5, q6 = np.array(joints).T
        T01 = omniKinematics.DH_transform(0,        0,  L1, -np.pi/2-q1)
        T12 = omniKinematics.DH_transform(-np.pi/2, 0,   0,         -q2)
        T23 = omniKinematics.DH_transform(0,       L2,   0,         -q3)
        T34 = omniKinematics.DH_transform(np.pi/2,  0, -L3,          q4)
        T45 = omniKinematics.DH_transform(-np.pi/2, 0,   0, -np.pi/2-q5)
        T5e = omniKinematics.DH_transform(np.pi/2,  0,   0,          q6)
        T0e = T01.dot(T12).dot(T23).dot(T34).dot(T45).dot(T5e)
        return T0e


if __name__ == "__main__":
    import utils as U
    T = omniKinematics.fk(joints=[0.10, 0.48, -0.19, 0.27, -0.7, 0.74])
    p = T[:3, -1]
    quat = U.R_to_quaternion(T[:3, :3])
    print (p)
    print (quat)