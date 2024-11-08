'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np
from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll',' LElbowYaw', 'LElbowRoll', 'LWristYaw2','LHand2'],
                       'LLeg': [ 'LHipYawPitch1','LHipRoll', 'LHipPitch','LKneePitch','LAnklePitch', 'RAnkleRoll' ],
                       'RLeg': ['RHipYawPitch1', 'RHipRoll', 'RHipPitch', 'RKneePitch','RAnklePitch','LAnkleRoll' ],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw2', 'RHand2']
                       }



    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        """
        Calculate the local transformation of one joint.

        :param joint_name: Name of the joint
        :param joint_angle: Joint angle in radians
        :return: 4x4 local transformation matrix
        """
        # Mapping of joint axes and translations
        joint_mapping = {
            # Head
            'HeadYaw': [2, [0.00, 0.00, 126.50]],
            'HeadPitch': [0, [0.00, 0.00, 0.00]],

            # Left Arm
            'LShoulderPitch': [0, [0.00, 98.00, 100.00]],
            'LShoulderRoll': [1, [0.00, 0.00, 0.00]],
            'LElbowYaw': [2, [105.00, 15.00, 0.00]],
            'LElbowRoll': [1, [0.00, 0.00, 0.00]],
            'LWristYaw': [2, [55.95, 0.00, 0.00]],

            # Right Arm
            'RShoulderPitch': [0, [0.00, -98.00, 100.00]],
            'RShoulderRoll': [1, [0.00, 0.00, 0.00]],
            'RElbowYaw': [2, [105.00, -15.00, 0.00]],
            'RElbowRoll': [1, [0.00, 0.00, 0.00]],
            'RWristYaw': [2, [55.95, 0.00, 0.00]],

            # Left Leg
            'LHipYawPitch': [2, [0.00, 50.00, -85.00]],
            'LHipRoll': [1, [0.00, 0.00, 0.00]],
            'LHipPitch': [0, [0.00, 0.00, 0.00]],
            'LKneePitch': [0, [0.00, 0.00, -100.00]],
            'LAnklePitch': [0, [0.00, 0.00, -102.90]],
            'LAnkleRoll': [1, [0.00, 0.00, 0.00]],

            # Right Leg
            'RHipYawPitch': [2, [0.00, -50.00, -85.00]],
            'RHipRoll': [1, [0.00, 0.00, 0.00]],
            'RHipPitch': [0, [0.00, 0.00, 0.00]],
            'RKneePitch': [0, [0.00, 0.00, -100.00]],
            'RAnklePitch': [0, [0.00, 0.00, -102.90]],
            'RAnkleRoll': [1, [0.00, 0.00, 0.00]],
        }

        # Get rotation axis and translation for the joint
        axis, translation = joint_mapping[joint_name]

        # Compute the rotation matrix
        if axis == 0:  # Pitch (y-axis)
            rotation = np.array([
                [np.cos(joint_angle), 0, np.sin(joint_angle)],
                [0, 1, 0],
                [-np.sin(joint_angle), 0, np.cos(joint_angle)]
            ])
        elif axis == 1:  # Roll (x-axis)
            rotation = np.array([
                [1, 0, 0],
                [0, np.cos(joint_angle), -np.sin(joint_angle)],
                [0, np.sin(joint_angle), np.cos(joint_angle)]
            ])
        elif axis == 2:  # Yaw (z-axis)
            rotation = np.array([
                [np.cos(joint_angle), -np.sin(joint_angle), 0],
                [np.sin(joint_angle), np.cos(joint_angle), 0],
                [0, 0, 1]
            ])
        else:
            rotation = np.eye(3)  # Identity matrix if no rotation axis is defined

        # Combine rotation and translation into a 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation  # Top-left 3x3 is the rotation matrix
        T[:3, 3] = np.array(translation) / 1000.0  # Top-right 3x1 is the translation vector (converted to meters)
        print(T)

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    #agent.run()
    # Initialize the ForwardKinematicsAgent
    #agent = ForwardKinematicsAgent()

    # Test Cases
    test_cases = [
        ('LShoulderPitch', np.radians(45)),  # 45 degrees for Left Shoulder Pitch
        ('LShoulderYaw', np.radians(30)),  # 30 degrees for Left Shoulder Yaw
        ('HeadYaw', np.radians(60)),  # 60 degrees for Head Yaw
        ('LElbowRoll', np.radians(-90)),  # -90 degrees for Left Elbow Roll
    ]

    # Run Tests
    for joint_name, joint_angle in test_cases:
        T = agent.local_trans(joint_name, joint_angle)
        print(f"Local Transformation for {joint_name} at angle {np.degrees(joint_angle):.2f} degrees:")
        print(T)
        print("-" * 50)

