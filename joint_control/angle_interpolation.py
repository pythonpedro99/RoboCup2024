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
from jedi.inference.recursion import total_function_execution_limit

from joint_control.keyframes import wipe_forehead, rightBackToStand, leftBellyToStand, leftBackToStand
from pid import PIDAgent
from keyframes import hello



def bezier_interpolation(t, p0, p1, p2, p3):
    angle = (
            (1 - t) ** 3 * p0 +
            3 * (1 - t) ** 2 * t * p1 +
            3 * (1 - t) * t ** 2 * p2 +
            t ** 3 * p3
    )
    return angle


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        joints, times, keys = keyframes
        target_joints = {}

        if not keys or not times:
            # If keys or times are empty, return an empty target_joints
            return target_joints


        if len(times) == 1:# Handle keyframes with a single entry
            for joint_index, joint_name in enumerate(joints):
                target_joints[joint_name] = keys[joint_index]
            return target_joints


        # End of the keyframe: compute total duration
        total_duration = max(max(joint_times) for joint_times in times if joint_times)

        # Scale the perception time
        scaled_perception_time = perception.time % total_duration

        for joint_index, joint_name in enumerate(joints):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]

            # Interpolate only if there are multiple keyframes
            for i in range(len(joint_times) - 1):
                t_start = joint_times[i]
                t_end = joint_times[i + 1]

                if t_start <= scaled_perception_time <= t_end:
                    # Bezier control points
                    p0 = joint_keys[i][0]
                    p3 = joint_keys[i + 1][0]

                    if len(joint_keys[i]) > 1:
                        _, _, handle1_angle = joint_keys[i][1]
                        _, _, handle2_angle = joint_keys[i + 1][2]
                        p1 = p0 + handle1_angle
                        p2 = p3 + handle2_angle
                    else:
                        # Linear interpolation fallback
                        p1 = p0 + (p3 - p0)
                        p2 = p3 + (p3 - p0)

                    # Normalize time for the interpolation
                    e_time = scaled_perception_time - t_start
                    total_time = t_end - t_start
                    normalized_time = e_time / total_time

                    # Bezier interpolation
                    target_joints[joint_name] = bezier_interpolation(normalized_time, p0, p1, p2, p3)
                    break

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
