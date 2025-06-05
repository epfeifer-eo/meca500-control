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
        arm.set_joint_vel(50)
        arm.set_cart_vel(50)
        # arm.nod("yes")
        # arm.nod("no")
        # arm.move_pose(175, -54, 150, 0, 90, 0)
        arm.move_joints(0, 0, 0, 0, 0, 0)
        # arm.set_joint_vel(5)
        # arm.move_pose(175, -54, 126.5, 0, 90, 0)
        # arm.tap()
        # arm.set_joint_vel(10)
        # arm.clean()
        arm.grid(origin=(175,-50),z_height=126.5, run_cleaning=False)
        
        # arm.move_pose(190, 0, 308, 0, 90, 0)  #Equivalent to joints = 0
        
        # arm.tap(distance_mm=50, pause_sec=1, cart_vel=20)
        
        arm.move_joints(0, 0, 0, 0, 0, 0)

        # path = [
        #     (200, 0, 130, -180, 0, 180),
        #     (200, 0, 120, -180, 0, 180),
        #     (200, 0, 110, -180, 0, 180)
        # ]
        
        # arm.follow_path(path, linear=True)

        # arm.move_joints(0,-60,60,0,30,0)

    finally:
        arm.disconnect()

if __name__ == "__main__":
    main()
