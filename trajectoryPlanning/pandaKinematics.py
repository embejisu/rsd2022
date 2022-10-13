import pandaVar
from scipy.spatial.transform import Rotation
import scipy.misc
import numpy as np
np.set_printoptions(precision=5, suppress=True)
import time


class pandaKinematics:
    def __init__(self):
        self.q0_null = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    @classmethod
    def DH_transform(cls, dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
        Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                         -np.sin(alpha) * d],
                        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                        [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
        return Ts

    @classmethod
    def fk(cls, joints):
        Ts = pandaKinematics.DH_transform(pandaVar.dhparam(joints))     # Tb1, T12, T23, ...
        # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
        Tbs = np.array([np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts)+1)])  # Tb1, Tb2, Tb3, ...
        # Tbs[-1]: from base to end effector
        return Tbs, Ts

    @classmethod
    def fk_matrix_to_vector(cls, T):
        pos = T[:3, -1]
        rot = Rotation.from_matrix(T[:3, :3]).as_rotvec()     # for Python 3
        # rot = Rotation.from_dcm(T[:3, :3]).as_rotvec()  # for Python2
        return np.concatenate((pos, rot))

    @classmethod
    def fk_vector_to_matrix(cls, x):
        T = np.zeros((4,4))
        T[:3, -1] = x[:3]
        T[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
        return T

    @classmethod
    def jacobian(cls, Tbs):
        """
        Tbs: Tb0, Tb1, Tb2, ...
        """
        Tbe = Tbs[-1]
        J = np.zeros((6, 7))
        for i in range(7):
            Zi = Tbs[i, :3, 2]  # vector of actuation axis
            J[3:, i] = Zi  # Jw
            Pin = (Tbe[:3, -1] - Tbs[i, :3, -1])  # pos vector from (i) to (n)
            J[:3, i] = np.cross(Zi, Pin)  # Jv
        return J

    # Using matrix calculation to get J (similar computing speed but maybe faster to get multiple jacobian at once)
    # Please don't delete this function just in case
    # @classmethod
    # def jacobian(cls, joints):
    #     Tbe, Ts = pandaKinematics.fk(joints)
    #     J = np.zeros((6, 7))
    #     Tbi = np.array([np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, 8)])
    #     Zi = Tbi[:, :3, 2]   # vector of actuation axis
    #     J[3:] = Zi.T  # Jw
    #     Pin = (Tbe[:3, -1] - Tbi[:, :3, -1])  # pos vector from (i) to (n)
    #     J[:3] = np.cross(Zi, Pin).T  # Jv
    #     return J

    @classmethod
    def ik(cls, Tb_ed, q0=[], RRMC=False):     # inverse kinematics using Newton-Raphson Method
        assert Tb_ed.shape == (4, 4)
        st = time.time()
        if q0 == []:
            qk = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0])  # initial guess
        else:
            qk = np.array(q0)
        iter = 1
        reached = False
        while not reached:
            Tbs = pandaKinematics.fk(joints=qk)[0]

            # Define Cartesian error
            Tb_ec = Tbs[-1]  # base to current ee
            Tec_ed = np.linalg.inv(Tb_ec).dot(Tb_ed)     #   transform from current ee to desired ee
            pos_err = Tb_ec[:3, :3].dot(Tec_ed[:3, -1])     # pos err in the base frame
            # rot_err = Tb_ec[:3, :3].dot(Rotation.from_dcm(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
            rot_err = Tb_ec[:3, :3].dot(Rotation.from_matrix(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
            err_cart = np.concatenate((pos_err, rot_err))

            # Inverse differential kinematics (Newton-Raphson method)
            J = pandaKinematics.jacobian(Tbs)
            Jp = np.linalg.pinv(J)
            k = 0.5     # step size is scaled down
            qk_next = qk + Jp.dot(err_cart*k)
            qk = qk_next

            # Convergence condition
            if np.linalg.norm(err_cart) < 10e-4:
                reached = True
            else:
                iter += 1
            if RRMC:
                reached = True

        # print ("iter=", iter, "time=", time.time() - st)
        assert ~np.isnan(qk).any()
        return qk

    @classmethod
    def null_space_control(cls, joints, crit='joint_limit'):    # Null-space control input
        Tbs = pandaKinematics.fk(joints=joints)[0]
        J = pandaKinematics.jacobian(Tbs)
        Jp = np.linalg.pinv(J)
        Jn = np.eye(7) - Jp.dot(J)
        k=0.1
        if crit == 'joint_limit':   # distance to joint limits
            qk_null_dot = [k*pandaKinematics.partial_derivative(pandaKinematics.distance_to_joint_limits, i, joints)
                           for i in range(len(joints))]
        elif crit == 'manipulability':
            qk_null_dot = [k * pandaKinematics.partial_derivative(pandaKinematics.manipulability, i, joints)
                           for i in range(len(joints))]
        elif crit == 'obstacle_avoidance':
            qk_null_dot = [k * pandaKinematics.partial_derivative(pandaKinematics.obstacle_avoidance, i, joints)
                           for i in range(len(joints))]
        else:
            raise ValueError
        return Jn.dot(qk_null_dot)

    @classmethod
    def manipulability(cls, q1, q2, q3, q4, q5, q6, q7):
        J = pandaKinematics.jacobian([q1, q2, q3, q4, q5, q6, q7])
        det = np.linalg.det(J.dot(J.T))
        return np.sqrt(det)

    @classmethod
    def distance_to_joint_limits(cls, q1, q2, q3, q4, q5, q6, q7):
        q = [q1, q2, q3, q4, q5, q6, q7]
        dist = [((q - (q_max+q_min)/2)/(q_max - q_min))**2 for q, q_max, q_min in zip(q, pandaVar.q_max, pandaVar.q_min)]
        return -np.sum(dist)/7/2

    @classmethod
    def obstacle_avoidance(cls, q1, q2, q3, q4, q5, q6, q7):
        q = [q1, q2, q3, q4, q5, q6, q7]
        Tbs = pandaKinematics.fk(joints=q)[0]
        p04 = Tbs[3][:3, -1]   # we can define multiple points on the robot
        p_obj = np.array([0.5, -0.5, 0.3])
        return np.linalg.norm(p04 - p_obj)

    @classmethod
    def partial_derivative(cls, func, var=0, point=[]):
        args = point[:]
        def wraps(x):
            args[var] = x
            return func(*args)
        return scipy.misc.derivative(wraps, point[var], dx=1e-6)


if __name__ == "__main__":
    # FK
    joints = [0.95, -0.702, -0.678, -2.238, -0.365, 0.703, 0.808]
    Tbe = pandaKinematics.fk(joints=joints)[0][-1]

    # IK
    import time
    st = time.time()
    print("q_des=", joints)
    print ("q_ik =", pandaKinematics.ik(Tb_ed=Tbe))
    print ("t_comp=", time.time() - st)