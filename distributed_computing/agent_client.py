'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from jsonrpclib import Server
import threading
import os
import sys
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from joint_control.keyframes import hello, leftBackToStand


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        def task():
            self.proxy.execute_keyframes(keyframes)

        thread = threading.Thread(target=task)
        thread.daemon = True
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        def task():
            self.proxy.set_transform(effector_name, transform)

        thread = threading.Thread(target=task)
        thread.daemon = True
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.server = Server("http://localhost:1999")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        try:
            response = self.server.get_angle(joint_name)
            print("Response:", response)
        except Exception as e:
            print(f"Error in get_angle: {e}")

    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        try:
            response = self.server.set_angle(joint_name, angle)
            print("Response:", response)
        except Exception as e:
            print(f"Error in set_angle: {e}")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        try:
            response = self.server.get_posture()
            print("Response:", response)
        except Exception as e:
            print(f"Error in get_posture: {e}")

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        try:
            response = self.server.execute_keyframes(keyframes)
            print("Response:", response)
        except Exception as e:
            print(f"Error in execute_keyframes: {e}")

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        try:
            response = self.server.get_transform(name)
            print("Response:", response)
        except Exception as e:
            print(f"Error in get_transform: {e}")

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        try:
            response = self.server.set_transform(effector_name, transform.tolist())
            print("Response:", response)
        except Exception as e:
            print(f"Error in set_transform: {e}")


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    T = np.identity(4)  # Start with an identity matrix
    T[0, 3] = 0.0  # X translation (column index 3)
    T[1, 3] = 0.05  # Y translation (column index 3)
    T[2, 3] = -0.26  # Z translation (column index 3)
    agent.get_angle("HeadYaw")
    agent.set_angle("HeadYaw", 1.5)
    agent.get_angle("HeadYaw")
    agent.get_posture()
    agent.execute_keyframes(hello())
    agent.execute_keyframes( ([], [], []))
    agent.get_transform('HeadYaw')
    agent.set_transform('LLeg',T)
    agent.get_transform('HeadYaw')
    agent.execute_keyframes(leftBackToStand())




