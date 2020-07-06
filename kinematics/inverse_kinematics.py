'''In this exercise you need to implement inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from math import atan2
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        # YOUR CODE HERE
        lambda_ = 1
        max_step = 0.1
        joint_angles = np.random.random(len(self.chains[effector_name]))

        target = np.matrix([self.from_trans(transform)]).T
        while True:
            Ts = [identity(len(self.chains[effector_name]))]
            for name in self.chains[effector_name]:
                Ts.append(self.transforms[name])

            Te = np.matrix([self.from_trans(Ts[-1])]).T

            error = target - Te
            error[error > max_step] = max_step
            error[error < -max_step] = -max_step
            T = np.matrix([self.from_trans(j) for j in Ts[0:-1]]).T
            J = Te - T
            dT = Te - T

            J[0, :] = dT[2, :]
            J[1, :] = dT[1, :]
            J[2, :] = dT[0, :]
            J[-1, :] = 1

            d_theta = np.dot(lambda_ ,np.dot(np.linalg.pinv(J),error))
            joint_angles += np.asarray(d_theta.T)[0]
            if np.linalg.norm(d_theta) < 1e-4:
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)
        names = []
        times = []
        keys = []
        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([5.0, 7.0])
            keys.append([[angles[i], [2, 0, 0], [2, 0, 0]],
                         [angles[i], [1, 0, 0], [1, 0, 0]]])
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

    def from_trans(self, T):
        theta_x, theta_y, theta_z = 0, 0, 0
	x, y, z = T[3,0], T[3,1], T[3,2]
        if T[0, 0] == 1:
            theta_x = atan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            theta_y = atan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            theta_z = atan2(T[1, 0], T[0, 0])
        return [x,y,z, theta_x, theta_y, theta_z]


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
