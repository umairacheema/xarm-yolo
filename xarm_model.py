"""
xarm_model.py
-------------

Description:
    This is the definition of the robotic arm to be used in robotics toolbox

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
    Can be imported for simulating xARM 1S robotic arm without end effector
    Example:
        from xarm_model import XARM

Dependencies:

"""

import numpy as np
from roboticstoolbox.robot.ET import ET
from roboticstoolbox.robot.Robot import Robot
from roboticstoolbox.robot.Link import Link


class XARM(Robot):
    """
    Create model of XARM 1S manipulator

    xarm = XARM() creates a robot object representing the XARM 1S
     robot arm. This robot is represented using the elementary
    transform sequence (ETS).

    :references:
        - Kinematic Derivatives using the Elementary Transform
          Sequence, J. Haviland and P. Corke

    """

    def __init__(self):

        mm = 1e-3
        tool_offset = (110) * mm


        l0 = Link(ET.tz(0.03) * ET.Rz(qlim=[-1.57,1.57]), name="link0", parent=None)

        l1 = Link( ET.tz(0.02) * ET.Rx(qlim=[-1.57,1.57]), name="link1", parent=l0)

        l2 = Link(ET.tz(0.1) * ET.Rx(qlim=[-1.57,1.57]), name="link2", parent=l1)

        l3 = Link(ET.tz(0.1) * ET.Rx(qlim=[-1.57,1.57]), name="link3", parent=l2)

        l4 = Link( ET.tz(0.055) * ET.Rz(qlim=[-1.57,1.57]),
            name="link4",
            parent=l3,
        )

        ee = Link(ET.tz(tool_offset) 
                  , name="ee", parent=l4)

        elinks = [l0, l1, l2, l3, l4]

        super(XARM, self).__init__(elinks, name="XARM 1S", manufacturer="Hiwonder")

        self.qr = np.array([0, -0.3, 0, -2.2, 0])
        self.qz = np.zeros(5)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = XARM()
    print(robot)