# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:12 2025

@author: elija
"""

import time
import threading
from mecademicpy.robot import Robot
from typing import Tuple, Optional


class Meca500:
    def __init__(self, ip_address="192.168.0.100", stepper=None):
        self.ip = ip_address
        self.robot = Robot()
        self.connected = False
        self.stepper = stepper

    def connect(self):
        if not self.connected:
            print(f"[Meca500] Connecting to {self.ip}...")
            self.robot.Connect(self.ip)
            self.connected = True
            print("[Meca500] Connected.")

    def activate_and_home(self):
        if not self.connected:
            raise RuntimeError("Robot not connected. Call connect() first.")

        print("[Meca500] Activating robot...")
        self.robot.ActivateRobot()

        print("[Meca500] Homing robot...")
        self.robot.Home()
        self.robot.WaitIdle(timeout=30)
        
    def set_joint_vel(self, deg_per_sec):
        print(f"[Meca500] Setting joint velocity: {deg_per_sec} deg/s")
        self.robot.SetJointVel(deg_per_sec)
    
    def set_cart_vel(self, mm_per_sec):
        print(f"[Meca500] Setting Cartesian velocity: {mm_per_sec} mm/s")
        self.robot.SetCartLinVel(mm_per_sec)

    def is_pose_safe(self, x, y, z, alpha=0, beta=0, gamma=0):
        """Check if a pose is probably within reach."""
        #TODO: update this
        in_bounds = (
            50 <= x <= 260 and
            -150 <= y <= 150 and
            100 <= z <= 300
        )
        return in_bounds


    def move_pose(self, x, y, z, alpha, beta, gamma):
        if not self.connected:
            raise RuntimeError("Robot not connected. Call connect() first.")
    
        print(f"[Meca500] Moving to pose: ({x}, {y}, {z}, {alpha}, {beta}, {gamma})")
        try:
            self.robot.MovePose(x, y, z, alpha, beta, gamma)
            self.robot.WaitIdle(timeout=10)
        except Exception as e:
            print(f"[Meca500] ERROR: Failed to move to pose — {e}")
            self.connected = False
    
    def move_joints(self, j1, j2, j3, j4, j5, j6):
        if not self.connected:
            raise RuntimeError("Robot not connected. Call connect() first.")
    
        print(f"[Meca500] Moving joints to: {j1}, {j2}, {j3}, {j4}, {j5}, {j6}")
        try:
            self.robot.MoveJoints(j1, j2, j3, j4, j5, j6)
            self.robot.WaitIdle(timeout=10)
        except Exception as e:
            print(f"[Meca500] ERROR: Failed to move joints — {e}")
            self.connected = False

    def follow_path(self, poses, linear=False):
        if not self.connected:
            raise RuntimeError("Robot not connected. Call connect() first.")
        
        for pose in poses:
            x, y, z, alpha, beta, gamma = pose
    
            if not self.is_pose_safe(x, y, z, alpha, beta, gamma):
                print(f"[Meca500] Skipping unsafe pose: {pose}")
                continue    
            print(f"[Meca500] Moving to pose: {pose}")
            try:
                if linear:
                    cmd = f"MoveLin({x:.3f},{y:.3f},{z:.3f},{alpha:.3f},{beta:.3f},{gamma:.3f})"
                    self.robot.SendCustomCommand(cmd)
                else:
                    self.robot.MovePose(x, y, z, alpha, beta, gamma)
                self.robot.WaitIdle(timeout=10)
            except Exception as e:
                print(f"[Meca500] ERROR: Failed to move to pose {pose} — {e}")
                self.connected = False
                break

    def tap(self, distance_mm=8, pause_sec=0.5, cart_vel=5, ramp_time=1.5, target_speed=2400):
        """Tap while stepper ramps up during descent and ramps down during ascent."""
        try:
            print(f"[Meca500] Setting Cartesian velocity to {cart_vel} mm/s")
            self.robot.SetCartLinVel(cart_vel)
    
            if self.stepper:
                print("[Meca500] Ramping up while descending")
                self.stepper.forward(speed=100)
                self.stepper.ramp_to_speed(target_speed, ramp_time, async_mode=True)
    
            print(f"[Meca500] Moving down {distance_mm} mm")
            self.robot.MoveLinRelWrf(0, 0, -distance_mm, 0, 0, 0)
            self.robot.WaitIdle()
    
            print(f"[Meca500] Pausing for {pause_sec} seconds")
            time.sleep(pause_sec)
    
            if self.stepper:
                print("[Meca500] Ramping down while ascending")
                self.stepper.ramp_to_speed(0, ramp_time, async_mode=True)
    
            print(f"[Meca500] Moving back up {distance_mm} mm")
            self.robot.MoveLinRelWrf(0, 0, distance_mm, 0, 0, 0)
            self.robot.WaitIdle()
    
            if self.stepper:
                self.stepper.stop()
    
        except Exception as e:
            print(f"[Meca500] ERROR: Tap failed — {e}")
            if self.stepper:
                self.stepper.stop()



    # def tap(self, distance_mm=8, pause_sec=0.5, cart_vel=1, ramp_time=0.25, target_speed=800):
    #     """Moves the end effector down and back up, spinning the stepper motor with ramp up/down."""
    #     try:
    #         print(f"[Meca500] Setting Cartesian velocity to {cart_vel} mm/s")
    #         self.robot.SetCartLinVel(cart_vel)
    
    #         print(f"[Meca500] Moving down {distance_mm} mm")
    #         self.robot.MoveLinRelWrf(0, 0, -distance_mm, 0, 0, 0)
    #         self.robot.WaitIdle()
    
    #         if self.stepper:
    #             print("[Meca500] Ramping up stepper motor")
    #             self.stepper.forward()
                
    
    #         print(f"[Meca500] Pausing for {pause_sec} seconds")
    #         time.sleep(pause_sec)
    
    #         if self.stepper:
    #             print("[Meca500] Ramping down and stopping stepper motor")
    #             self.stepper.stop()
    
    #         print(f"[Meca500] Moving back up {distance_mm} mm")
    #         self.robot.MoveLinRelWrf(0, 0, distance_mm, 0, 0, 0)
    #         self.robot.WaitIdle()
    
    #     except Exception as e:
    #         print(f"[Meca500] ERROR: Tap-down failed — {e}")
    #         if self.stepper:
    #             self.stepper.stop()

    
    def nod(self, mode="yes"):
        """
        Perform a cheeky nod gesture. 
        mode="yes": nod up and down
        mode="no": shake left and right
        """
        print(f"[Meca500] That is gonna be a {mode}...")
    
        # Move to base pose
        self.move_joints(0, -20, 0, 0, 0, 0)
    
        if mode == "yes":
            self.set_joint_vel(20)
            self.move_joints(-45, -20, -20, 0, 25, 0)
            self.set_joint_vel(50)
            self.move_joints(-45, -20, -20, 0, -15, 0)
            self.move_joints(-45, -20, -20, 0, 50, 0)
            self.move_joints(-45, -20, -20, 0, -15, 0)
    
        elif mode == "no":
            self.set_joint_vel(20)
            self.move_joints(-45, -20, -20, 0, 35, 0)
            self.set_joint_vel(30)
            self.move_joints(-35, -20, -20, 0, 35, 0)
            self.move_joints(-55, -20, -20, 0, 35, 0)
            self.move_joints(-45, -20, -20, 0, 35, 0)
    
        else:
            print(f"[Meca500] Unknown nod mode: {mode}. Use 'yes' or 'no'.")
            return
    
        # Return to base
        self.set_joint_vel(20)
        self.move_joints(0, -20, 0, 0, 0, 0)
        self.set_joint_vel(10)

    def clean(self, pause_sec=1):
        """Move to cleaning station"""
        print("[Meca500] Moving to cleaning station...")
        
        self.set_joint_vel(20)
        self.move_joints(45, -20, -20, 0, 35, 0)
        #TODO: Trigger air blast
        #TODO: add gpio on/off for stepper
        time.sleep(pause_sec)
        # self.robot.MoveLinRelWrf(0, 10, 0, 0, 0, 0)  # small wipe right
        # self.robot.WaitIdle()
        # self.robot.MoveLinRelWrf(0, -10, 0, 0, 0, 0)  # small wipe left
        # self.robot.WaitIdle()

    def grid(
        self,
        origin=(100, -50),
        x_spacing=9,
        y_spacing=9,
        z_height=308,
        rows=12,
        cols=8,
        angles=(0, 90, 0),
        run_cleaning=True
    ):
        """
        Run a grid routine: move to each point, tap, and optionally clean.

    
        Args:
            origin (tuple): (x, y) starting coordinate.
            x_spacing (int): Horizontal spacing between columns (mm).
            y_spacing (int): Vertical spacing between rows (mm).
            z_height (int): Constant Z height (mm).
            rows (int): Number of rows.
            cols (int): Number of columns.
            angles (tuple): (alpha, beta, gamma) orientation.
            run_cleaning (bool): Whether to call self.clean() after each tap.
        """
        x0, y0 = origin
        alpha, beta, gamma = angles
    
        for row in range(rows):
            y = y0 + row * y_spacing
            x_range = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
    
            for col in x_range:
                x = x0 + col * x_spacing
                print(f"[Meca500] Moving to grid point ({row}, {col}) at ({x}, {y}, {z_height})")
    
                if not self.is_pose_safe(x, y, z_height, alpha, beta, gamma):
                    print("[Meca500] Skipping unsafe pose.")
                    continue
    
                self.move_pose(x, y, z_height, alpha, beta, gamma)
                self.tap()
                if run_cleaning:
                    self.clean()

    # def grid_from_references(
    #     self,
    #     A1: Tuple[float, float],
    #     A12: Tuple[float, float],
    #     H12: Tuple[float, float],
    #     z_height: float = 308,
    #     rows: int = 8,
    #     cols: int = 12,
    #     angles: Tuple[float, float, float] = (0, 90, 0),
    #     run_cleaning: bool = True,
    #     skip_columns: Optional[str] = None,  # 'even', 'odd', or (Default)=None
    #     safe_pose: Tuple[float, float, float, float, float, float] = (190, 0, 308, 0, 90, 0)
    # ):
    #     x0, y0 = A1                 # A1 (tuple): (x, y) coordinates of top-left well (row 0, col 0)
    #     x1, y1 = A12                # A12 (tuple): (x, y) coordinates of top-right well (row 0, col 11)
    #     x2, y2 = H12                # H12 (tuple): (x, y) coordinates of bottom-right well (row 7, col 11)
    #     alpha, beta, gamma = angles
    
    #     row_dx = (x1 - x0) / (cols - 1)
    #     row_dy = (y1 - y0) / (cols - 1)
    #     col_dx = (x2 - x1) / (rows - 1)
    #     col_dy = (y2 - y1) / (rows - 1)
    
    #     for row in range(rows):
    #         col_range = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
    
    #         for col in col_range:
    #             if skip_columns == 'odd' and col % 2 == 0:
    #                 continue
    #             elif skip_columns == 'even' and col % 2 == 1:
    #                 continue
    
    #             x = x0 + col * row_dx + row * col_dx
    #             y = y0 + col * row_dy + row * col_dy
    
    #             print(f"[Meca500] Moving to grid point ({row}, {col}) at ({x:.2f}, {y:.2f}, {z_height})")
    
    #             if not self.is_pose_safe(x, y, z_height, alpha, beta, gamma):
    #                 print("[Meca500] Skipping unsafe pose.")
    #                 continue
    
    #             self.move_pose(x, y, z_height, alpha, beta, gamma)
    #             self.tap()
    #             if run_cleaning:
    #                 self.clean()
    #     print(f"[Meca500] Returning to safe pose: {safe_pose}")
    #     self.move_pose(*safe_pose)
    #     self.move_joints(0, 0, 0, 0, 0, 0)

    def grid_from_references(
        self,
        A1: Tuple[float, float],
        A12: Tuple[float, float],
        H12: Tuple[float, float],
        z_height: float = 308,
        rows: int = 8,
        cols: int = 12,
        angles: Tuple[float, float, float] = (0, 90, 0),
        run_cleaning: bool = True,
        skip_columns: Optional[str] = None,  # 'Even', 'Odd', or None
        should_stop: Optional[threading.Event] = None,  # Optional stop flag
        return_safe_pose: Tuple[float, float, float, float, float, float] = (190, 0, 308, 0, 90, 0)
    ):
        x0, y0 = A1
        x1, y1 = A12
        x2, y2 = H12
        alpha, beta, gamma = angles
    
        row_dx = (x1 - x0) / (cols - 1)
        row_dy = (y1 - y0) / (cols - 1)
        col_dx = (x2 - x1) / (rows - 1)
        col_dy = (y2 - y1) / (rows - 1)
    
        for row in range(rows):
            if should_stop and should_stop.is_set():
                print("[Meca500] Stop signal received. Exiting grid routine.")
                break
    
            col_range = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
    
            for col in col_range:
                if should_stop and should_stop.is_set():
                    print("[Meca500] Stop signal received. Exiting grid routine.")
                    break
    
                if skip_columns == 'Odd' and col % 2 == 0:
                    continue
                elif skip_columns == 'Even' and col % 2 == 1:
                    continue
    
                x = x0 + col * row_dx + row * col_dx
                y = y0 + col * row_dy + row * col_dy
    
                print(f"[Meca500] Moving to grid point ({row}, {col}) at ({x:.2f}, {y:.2f}, {z_height})")
    
                if not self.is_pose_safe(x, y, z_height, alpha, beta, gamma):
                    print("[Meca500] Skipping unsafe pose.")
                    continue
    
                try:
                    self.move_pose(x, y, z_height, alpha, beta, gamma)
                    self.tap()
                    if run_cleaning:
                        self.clean()
                except Exception as e:
                    print(f"[Meca500] ERROR: Failed at grid point ({row}, {col}) — {e}")
                    break
    
        """If stop was triggered, move to safe pose"""
        if should_stop and should_stop.is_set():
            try:
                print(f"[Meca500] Moving to safe pose {return_safe_pose}")
                self.move_pose(*return_safe_pose)
            except Exception as e:
                print(f"[Meca500] ERROR: Could not return to safe pose — {e}")
        try:
            print(f"[Meca500] Returning to safe pose: {return_safe_pose}")
            self.move_pose(*return_safe_pose)
            self.move_joints(0, 0, 0, 0, 0, 0)
        except Exception as e:
            print(f"[Meca500] ERROR: Could not return to safe pose — {e}")

    def abort_and_recover(self, safe_pose=(190, 0, 308, 0, 90, 0)):
        print("[Meca500] Aborting and recovering...")
    
        try:
            self.robot.SendCustomCommand("Abort")
        except Exception as e:
            print(f"[Meca500] Abort failed — {e}")
    
        try:
            self.robot.ResetError()
        except Exception as e:
            print(f"[Meca500] ResetError failed — {e}")
    
        if not self.connected:
            try:
                self.connect()
            except Exception as e:
                print(f"[Meca500] Reconnect failed — {e}")
                return
    
        try:
            self.robot.ActivateRobot()
            self.move_pose(*safe_pose)
        except Exception as e:
            print(f"[Meca500] Recovery move failed — {e}")


    def disconnect(self):
        if self.connected:
            print("[Meca500] Disconnecting...")
            self.robot.Disconnect()
            self.connected = False
            print("[Meca500] Disconnected.")