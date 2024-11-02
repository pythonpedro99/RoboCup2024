'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''
from joint_control.keyframes import wipe_forehead, rightBackToStand
from pid import PIDAgent
from keyframes import hello
import time


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.current_time = 0  # Initialize current time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezier_interpolation(self, t, key_points):
        return (
            (1 - t) ** 3 * key_points[0] +
            3 * (1 - t) ** 2 * t * key_points[1] +
            3 * (1 - t) * t ** 2 * key_points[2] +
            t ** 3 * key_points[3]
        )

    def angle_interpolation(self, keyframes, perception):
        joints, times, keys = keyframes  # Unpack keyframes
        target_joints = {} # initialize for later use

        for joint_index, joint_name in enumerate(joints):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]
            if not joint_times:
                continue


            for i in range(len(joint_times) - 1):
                if joint_times[i] <= perception.time <= joint_times[i + 1]:
                    t = (perception.time - joint_times[i]) / (joint_times[i + 1] - joint_times[i]) # select interpolation window
                    P0 = joint_keys[i][0]
                    handle1 = joint_keys[i][1]
                    handle2 = joint_keys[i][2]
                    P1 = P0 + handle1[2]
                    P2 = joint_keys[i + 1][0] + handle2[2]
                    P3 = joint_keys[i + 1][0]

                    # Perform Bezier interpolation
                    angle = self.bezier_interpolation(t, [P0, P1, P2, P3])
                    target_joints[joint_name] = angle
                    break
            else:
                # If current time is beyond last keyframe, hold the last position
                target_joints[joint_name] = joint_keys[-1][0]
        return target_joints



if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
