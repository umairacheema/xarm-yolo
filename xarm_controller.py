"""
xarm_controller.py
-------------

Description:
    This module provides the interface with xARM 1s over USB

Author:
    Umair Cheema <cheemzgpt@gmail.com>

Version:
    1.0.0

License:
    Apache License 2.0 (https://www.apache.org/licenses/LICENSE-2.0)

Date Created:
    2024-12-01

Last Modified:
    2024-12-13

Python Version:
    3.8+

Usage:
    Can be imported for controlling xARM 1S robotic arm 
    Example:
        from xarm_controller import XARMController

Dependencies:
    xarm

"""


import os
import xarm
import numpy as np

class XARMController:

    def __init__(self):
        os.environ['LD_LIBRARY_PATH'] = os.getcwd() 
        self.arm = xarm.Controller('USB')
        self.end_effector = 0.
        self.state = np.array([0.0,0.0,0.0,0.0,0.0])
        self.get_initial_state()
    
    
    def reset(self):
        self.set_joint_state([0.,-8.,0.,0.,-50.])
        self.move_end_effector(40)
    
    def get_initial_state(self):
        self.state = np.array([self.arm.getPosition(6, degrees=True),
                      self.arm.getPosition(5, degrees=True),
                      self.arm.getPosition(4, degrees=True),
                      self.arm.getPosition(3, degrees=True),
                      self.arm.getPosition(2, degrees=True)
            ])
        self.end_effector = self.arm.getPosition(1, degrees=True)
    
    def update_joints_state(self):
        self.state = np.array([self.arm.getPosition(6, degrees=True),
                      self.arm.getPosition(5, degrees=True),
                      self.arm.getPosition(4, degrees=True),
                      self.arm.getPosition(3, degrees=True),
                      self.arm.getPosition(2, degrees=True)
        ])
    
    def update_end_effector_state(self):
        self.end_effector = self.arm.getPosition(1, degrees=True)


    def get_joints_state(self, radians = False):
        self.update_joints_state()
        state = self.state
        if(radians):
            conversion = np.pi/180
            state = conversion * state
        return state
    
    def set_joint_state(self, position_vector, duration_vector = None, radians=False, wait=True):
        if isinstance(position_vector, list):
            position_vector = np.array(position_vector)
        servos = [6,5,4,3,2]
        q = position_vector.astype(float)
        conversion = 180/np.pi
        if (radians):
            q *= conversion
        
        for i,angle in enumerate(q):
            if (duration_vector):
                self.arm.setPosition(servos[i],q[i], duration_vector[i], wait=wait)
            else:
                self.arm.setPosition(servos[i],q[i],1000, wait=wait)

    def move_joint(self, joint_number, angle, duration=1000, radians=False, wait=True):
        servos = [6,5,4,3,2]
        angle_val = float(angle)
        if(radians):
            conversion = 180/np.pi
            angle_val *= conversion
        self.arm.setPosition(servos[joint_number], angle_val,duration,wait=wait)

    def get_end_effector_state(self, textual = False):
        self.update_end_effector_state()
        state = self.end_effector
        if(textual):
            if -125 <= state < -90:
                state = 'fully_open'
            elif -90 <= state < -45:
                state = 'partially_open'
            elif -45 <= state < 45:
                state = 'partially_close'
            elif 45 <= state < 90:
                state = 'fully_closed' 
        return state

    def move_end_effector(self, angle):
        self.arm.setPosition(1,float(angle), wait=True)
    


