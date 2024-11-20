'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import time
import numpy as np
from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
#from jsonrpclib import Server
import threading
#from joint_control import keyframes
#from joint_control.keyframes import hello

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super().__init__()  # Initialize the AngleInterpolationAgent
        self.server = SimpleJSONRPCServer(("localhost", 1999))
        print("Starting JSON-RPC server on localhost:1999")

        # Register RPC functions
        self.server.register_function(self.get_angle, "get_angle")
        self.server.register_function(self.set_angle, "set_angle")
        self.server.register_function(self.get_posture, "get_posture")
        self.server.register_function(self.execute_keyframes, "execute_keyframes")
        self.server.register_function(self.get_transform, "get_transform")
        self.server.register_function(self.set_transform, "set_transform")


        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()


    def run_server(self):
        self.server.serve_forever()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        try:
            angle = self.perception.joint.get(joint_name)
            return {"status": "success", "angle": angle}
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        try:
            self.target_joints[joint_name] = angle
            return {"status": "success", "angle": angle}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        try:
            posture = self.posture
            return {"status": "success", "posture": posture}
        except Exception as e:
            return {"status": "error", "message": str(e)}


    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
                e.g. return until keyframes are executed'''
        try:
            self.keyframes = keyframes
            joints, times, keys = keyframes
            total_duration = max(max(joint_times) for joint_times in times if joint_times)
            time.sleep(total_duration + 0.5)
            return {"status": "success"}
        except Exception as e:
            return {"status": "error", "message": str(e)}


    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        try:
            transform = self.transforms[name]
            transform = transform.tolist()
            return {"status": "success", "transform": transform}
        except Exception as e:
            return {"status": "error", "message": str(e)}


    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        try:
            if isinstance(transform, list):
                transform = np.array(transform)

            if transform.shape != (4, 4):
                raise ValueError("Transform must be a 4x4 matrix.")
            self.set_transforms(effector_name, transform)
            time.sleep(0.5)
            return {"status": "success"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()


