"""
xarm_model_test.py
-------------

Description:
    This module is used to test xarm model through robotics toolbox

Author:
    Umair Cheema <cheemzgpt@gmail.com>

Version:
    1.0.0

License:
    Apache License 2.0 (https://www.apache.org/licenses/LICENSE-2.0)

Date Created:
    2024-12-01

Last Modified:
    2024-12-10

Python Version:
    3.8+

Usage:
    Can be run from the command line /Terminal /Shell
    Example:
        python xarm_model_test.py

Dependencies:
    roboticstoolbox

"""
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot
from xarm_model import XARM


def degree_to_radians(angles):
    _angles = np.asarray(angles)
    _angles = np.deg2rad(_angles)
    return _angles

# Make and instance of the Swift simulator and open it
env = PyPlot()
env.launch(realtime=True, browser="safari")

# Make a xarm model and set its joint angles to the ready joint configuration
xarm = XARM()
xarm.q = xarm.qr

data = np.asarray([90,37.5,80.25,-5,-50])
xarm.q = degree_to_radians(data)
# Set a desired and effector pose an an offset from the current end-effector pose
forward_kinematics = xarm.fkine(xarm.q)
inverse_kinematics = xarm.ikine_LM(forward_kinematics)
print(inverse_kinematics.q)
xarm.q = inverse_kinematics.q
env = PyPlot()
env.launch()
env.add(xarm)
#xarm.q = [0.0,0.0,0.0,0.0,0.0]
# Set a desired and effector pose an an offset from the current end-effector pose
env.hold()