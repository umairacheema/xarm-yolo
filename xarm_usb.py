"""
xarm_usb.py
-------------

Description:
    This module demonstrates communication with robotic arm using USB

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
        python xarm_usb.py

Dependencies:
    xarm

"""


import xarm
import os

#os.environ['LD_LIBRARY_PATH'] = os.getcwd() 
arm = xarm.Controller('USB', debug=True)

servo1 = xarm.Servo(1)
servo2 = xarm.Servo(2)
servo3 = xarm.Servo(3)

# Gets the position of servo 1 in units
position = arm.getPosition(1)
print('Servo 1 position:', position)

# Gets the position of servo 2 as defined above
position = arm.getPosition(servo2)
print('Servo 2 position:', position)

# Gets the position of servo 3 in degrees
position = arm.getPosition(3, True)
print('Servo 3 position (degrees):', position)

# Gets the position of servo 2 as defined above
# It is not necessary to set the degreees parameter
# because the Servo object performes that conversion
position = arm.getPosition([servo1, servo2, servo3])
print('Servo 1 position (degrees):', servo1.angle)
print('Servo 2 position (degrees):', servo2.angle)
print('Servo 3 position (degrees):', servo3.angle)

arm.setPosition(1, -90.0, wait=True) 
