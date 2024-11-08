'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from joint_control.keyframes import leftBellyToStand
from keyframes import hello
import keras
import numpy as np


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = keras.models.load_model('clfv1.keras')
    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):

        # used by the model
        joint_names = [
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch'
        ]
        imu_names  = ['AngleX', 'AngleY']

        posture_labels = [
            'Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee',
            'Left', 'Right', 'Sit', 'Stand', 'StandInit'
        ]


        joint_angles = []
        for joint_name in joint_names:
            angle = perception.joint.get(joint_name, 0.0)
            joint_angles.append(angle)
        for i in range(2):
            angle = perception.imu[i]
            joint_angles.append(angle)



        joint_angles_array = np.array(joint_angles).reshape(1, -1) # rehape for model
        prediction = self.posture_classifier.predict(joint_angles_array) # let model predict
        predicted_class_index = np.argmax(prediction, axis=1)[0] # get max out of the distribution
        print(predicted_class_index)
        posture = posture_labels[predicted_class_index]


        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    print(agent.posture)
