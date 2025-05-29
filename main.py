# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:02:56 2025

@author: elija
"""


from meca500 import Meca500


def main():
    arm = Meca500()

    try:
        arm.connect()
        arm.activate_and_home()

        arm.move_joints(0, 0, 0, 0, 0, 0)
        # arm.move_joints(-45, -20, -20, 0, 25, 0)
        # arm.move_joints(-45, -20, -20, 0, -25, 0)
        # arm.move_joints(-45, -20, -20, 0, 60, 0)
        # arm.move_joints(-45, -20, -20, 0, -25, 0)
        # arm.move_joints(0, 0, 0, 0, 0, 0)
        # arm.move_pose(200, 0, 150, 180, 0, 0)
        # arm.move_joints(0, 0, 0, 0, 0, 0)

        path = [
            (250, 0, 150, 90, 0, 0),
            (240, 20, 150, 90, 0, 0),
            (230, 40, 150, 90, 0, 0)
        ]
        
        arm.follow_path(path, linear=True)

        arm.move_joints(0,-60,60,0,30,0)

    finally:
        arm.disconnect()

if __name__ == "__main__":
    main()
