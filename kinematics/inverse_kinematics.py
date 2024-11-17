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
import numpy as np
from scipy.optimize import minimize

from joint_control.keyframes import hello


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_bounds = {
            'LHipRoll': (-0.379472, 0.790477),
            'LHipPitch': (-1.535889, 0.484090),
            'LKneePitch': (-0.092346, 2.112528),
            'LAnklePitch': (-1.189516, 0.922747),
            'LAnkleRoll': (-0.397880, 0.769001),
            'RHipRoll': (-0.790477, 0.379472),
            'RHipPitch': (-1.535889, 0.484090),
            'RKneePitch': (-0.103083, 2.120198),
            'RAnklePitch': (-1.186448, 0.932056),
            'RAnkleRoll': (-0.768992, 0.397935),
        }

        joint_angles = []
        # YOUR CODE HERE

        chain = self.chains[effector_name]

        bounds = [joint_bounds[joint] if joint in joint_bounds else (0.0, 0.0) for joint in chain]

        # Initialize all joints with a very small non-zero angle
        initial_angles = {j: 1e-6 for j in self.transforms.keys()}

        def error_function(angles):
            for i, joint in enumerate(chain):
                initial_angles[joint] = angles[i]

            self.forward_kinematics(initial_angles)
            current_transform = self.transforms[chain[-1]]

            # Calculate the error between current and target transform
            error = np.linalg.norm(transform[:3, 3] - current_transform[:3, 3])  # Position error
            return error


        result = minimize(
            fun=error_function,
            x0=[0] * len(chain),
            bounds= bounds,  # Joint angle limits (example bounds)
            method='L-BFGS-B'  # Optimization method
        )

        # Extract optimized angles
        if result.success:
            joint_angles = result.x.tolist()
        else:
            print("Optimization failed:", result.message)

        return joint_angles


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        print(joint_angles)
        chain = self.chains[effector_name]
        names = []
        times = []
        keys = []
        for i, (joint, angle) in enumerate(zip(chain, joint_angles)):
            names.append(joint)
            keys.append(angle)
        times.append(0.0)

        self.keyframes = (names, times,keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    agent.keyframes = hello()

    # Test inverse kinematics with a transformation matrix
    T = np.identity(4)  # Start with an identity matrix
    T[0, 3] = 0.0  # X translation (column index 3)
    T[1, 3] = 0.05  # Y translation (column index 3)
    T[2, 3] = -0.26  # Z translation (column index 3)

    # Call the set_transforms method for the left leg (LLeg)
    #agent.set_transforms('LLeg', T)

    # Run the agent
    agent.run()
