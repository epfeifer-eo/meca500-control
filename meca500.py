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
        self.tap_params = {}

    def connect(self):
        if not self.connected:
            print(f"[Meca500] Connecting to {self.ip}...")
            self.robot.Connect(self.ip)
            self.connected = True
            print("[Meca500] Connected.")
    
    """It is critical that you call this after connecting to the arm"""
    def activate_and_home(self):
        if not self.connected:
            raise RuntimeError("Robot not connected. Call connect() first.")

        print("[Meca500] Activating robot...")
        self.robot.ActivateRobot()

        print("[Meca500] Homing robot...")
        self.robot.Home()
        self.robot.WaitIdle(timeout=30)
    
    """This and set_cart_vel can be used initially to set a base or called upon in other methods"""   
    def set_joint_vel(self, deg_per_sec):
        print(f"[Meca500] Setting joint velocity: {deg_per_sec} deg/s")
        self.robot.SetJointVel(deg_per_sec)
    
    def set_cart_vel(self, mm_per_sec):
        print(f"[Meca500] Setting Cartesian velocity: {mm_per_sec} mm/s")
        self.robot.SetCartLinVel(mm_per_sec)
    
    
    """
    This is a user defined safety check, I highly reccomend you log into the mecademic portal to move the arm around and find these values.
    Just put the robots ip into a web browser with the arm connected to the laptop over ethernet. 
    Default ip is 192.168.0.100 and make sure your computer has a static ip on the same network as the arm.
    Note that this does not check for singularities, it is simply a boundary so that you dont slam the tool into something.
    """
    def is_pose_safe(self, x, y, z, alpha=0, beta=0, gamma=0):
        """Check if a pose is probably within reach."""
        #TODO: update this
        in_bounds = (
            50 <= x <= 260 and
            -150 <= y <= 150 and
            100 <= z <= 300
        )
        return in_bounds

    """
    Move_pose vs Move_joints are different see: https://resources.mecademic.com/en/doc/latest/Meca500/MC-PM-MECA500/motion_commands/index.html
    """
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

    def tap(self,
             distance_mm=8, pause_sec=0.5, cart_vel=5, ramp_time=1.5,
             target_speed=26000, circle_radius=1):
        """Tap while stepper ramps up/down concurrently with Z motion."""
        try:
            #import math
            print(f"[Meca500] Setting Cartesian velocity to {cart_vel} mm/s")
            self.robot.SetCartLinVel(cart_vel)
    
            # DESCEND + ramp-up at the same time
            print(f"[Meca500] Moving down {distance_mm} mm and ramping up stepper")
    
            if self.stepper:
                self.stepper.reverse(speed=50)
                threading.Thread(
                    target=lambda: self.stepper.ramp_to_speed(target_speed, ramp_time, async_mode=False),
                    daemon=True
                ).start()
    
            self.robot.MoveLinRelWrf(0, 0, -distance_mm, 0, 0, 0)
            self.robot.WaitIdle()
    
            time.sleep(pause_sec / 2)
            
            # Spiral from edge inward
            print(f"[Meca500] Performing inward spiral (radius={circle_radius}, turns=1.5)")
            
            from math import pi, cos, sin
            
            spiral_points = []
            turns = 1.5  # full rotations
            total_points = 40  # more = smoother
            
            for i in range(total_points):
                t = i / (total_points - 1)
                radius = circle_radius * (1 - t)
                angle = turns * 2 * pi * t
                dx = radius * cos(angle)
                dy = radius * sin(angle)
                spiral_points.append((dx, dy))
            
            # Move from center to spiral start point
            first_dx, first_dy = spiral_points[0]
            cmd = f"MoveLinRelWrf({first_dx:.3f},{first_dy:.3f},0,0,0,0)"
            self.robot.SendCustomCommand(cmd)
            
            # Perform spiral
            for dx, dy in spiral_points:
                cmd = f"MoveLinRelWrf({dx - first_dx:.3f},{dy - first_dy:.3f},0,0,0,0)"
                self.robot.SendCustomCommand(cmd)
                first_dx, first_dy = dx, dy
            
            self.robot.WaitIdle()

            
            
            # Circle motion 
            # print(f"[Meca500] Drawing circle pattern (radius={circle_radius}, points={circle_points})")
            # path = [
            #     (circle_radius * math.cos(2 * math.pi * i / circle_points),
            #      circle_radius * math.sin(2 * math.pi * i / circle_points))
            #     for i in range(circle_points)
            # ]
    
            # for dx, dy in path:
            #     cmd = f"MoveLinRelWrf({dx:.3f},{dy:.3f},0,0,0,0)"
            #     self.robot.SendCustomCommand(cmd)
            #     # self.robot.MoveLinRelWrf(dx, dy, 0, 0, 0, 0)
            #     # self.robot.WaitIdle()
            # for dx, dy in reversed(path):
            #     cmd = f"MoveLinRelWrf({-dx:.3f},{-dy:.3f},0,0,0,0)"
            #     self.robot.SendCustomCommand(cmd)
            #     # self.robot.MoveLinRelWrf(-dx, -dy, 0, 0, 0, 0)
            #     # self.robot.WaitIdle()
            # self.robot.WaitIdle()
            
            time.sleep(pause_sec / 2)
    
            # ASCEND + ramp-down at the same time
            print(f"[Meca500] Moving up {distance_mm} mm and ramping down stepper")
    
            if self.stepper:
                threading.Thread(
                    target=lambda: self.stepper.ramp_to_speed(0, 2 * ramp_time, async_mode=False),
                    daemon=True
                ).start()
    
            self.robot.MoveLinRelWrf(0, 0, distance_mm, 0, 0, 0)
            self.robot.WaitIdle()
    
            if self.stepper:
                self.stepper.stop()
    
        except Exception as e:
            print(f"[Meca500] ERROR: Tap failed — {e}")
            if self.stepper:
                self.stepper.stop()

    #This was me learning how to move the arm and so that I could have the arm tell Mike yes or no when he came to my door
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
            print(f"[Meca500] There is no {mode}. Use 'yes' or 'no'.")
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


    #DO NOT use this grid, it will have compounding errors from localizing off just a single point
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

    
    #This is the correct grid function, it extrapolates vectors from the 3 given points A1, A12, and H12 with A1 being close right well from arm's perspective
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
                    if self.tap_params:
                        self.tap(**self.tap_params)
                    else:
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


    #I had a thougt to include a soft-estop but never actually tested this, proceed with caution
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

    #Self explanatory 
    def disconnect(self):
        if self.connected:
            print("[Meca500] Disconnecting...")
            self.robot.Disconnect()
            self.connected = False
            print("[Meca500] Disconnected.")
            
    def collect(
        self,
        pose=(103.5, -62, 245, 90, 59.06, -90),
        surface_offset_mm=10.0,
        drill_depth_mm=8.0,
        speed=1200,
        cart_vel=2.0,
        pause_sec=0.5
    ):
        """Move to soil collection pose, descend to surface, drill down, return."""
        print("[Meca500] Starting collection sequence...")
        x, y, z, alpha, beta, gamma = pose
    
        # Move to approach pose
        self.set_cart_vel(5)
        self.move_pose(x, y, z, alpha, beta, gamma)
    
        # Descend to surface at fixed speed
        self.set_cart_vel(5)
        self.robot.MoveLinRelWrf(0, 0, -surface_offset_mm, 0, 0, 0)
        self.robot.WaitIdle()
    
        # Start auger forward
        if self.stepper:
            self.stepper.forward(speed)
    
        # Drill at user-defined speed
        self.set_cart_vel(cart_vel)
        self.robot.MoveLinRelWrf(0, 0, -drill_depth_mm, 0, 0, 0)
        self.robot.WaitIdle()
    
        # Pause at bottom
        print(f"[Meca500] Dwell at bottom for {pause_sec} sec")
        time.sleep(pause_sec)
    
        # Stop auger
        if self.stepper:
            self.stepper.stop()
    
        # Return upward at fixed speed
        self.set_cart_vel(5)
        self.robot.MoveLinRelWrf(0, 0, surface_offset_mm + drill_depth_mm, 0, 0, 0)
        self.robot.WaitIdle()
    
        print("[Meca500] Collection complete.")
    
    def deposit(
        self,
        pose,
        deposit_depth_mm=5.0,
        speed=800,
        cart_vel=2.0,
        pause_sec=1.5
    ):
        """Move to well, lower tool, reverse stepper, raise up."""
        print("[Meca500] Starting deposit sequence...")
        x, y, z, alpha, beta, gamma = pose
    
        self.set_cart_vel(cart_vel)
    
        # Move to well pose
        self.move_pose(x, y, z, alpha, beta, gamma)
    
        # Lower to just inside well
        self.robot.MoveLinRelWrf(0, 0, -deposit_depth_mm, 0, 0, 0)
        self.robot.WaitIdle()
    
        # Start reverse
        if self.stepper:
            self.stepper.reverse(speed)
        
        print(f"[Meca500] Dwell at bottom for {pause_sec} sec")
        time.sleep(pause_sec)
        
        # Stop stepper before lifting
        if self.stepper:
            self.stepper.stop()
        
        # Raise up
        self.robot.MoveLinRelWrf(0, 0, deposit_depth_mm, 0, 0, 0)
        self.robot.WaitIdle()
    
        print("[Meca500] Deposit complete.")

    def dump(
        self,
        pose=(103.5, -62, 245, 90, 59.06, -90),
        reverse_speed=800,
        duration=2.0,
        return_safe_pose=(190, 0, 308, 0, 90, 0)
    ):
        """Move to collection pose and reverse stepper to empty tool."""
        print("[Meca500] Dumping contents...")
        x, y, z, alpha, beta, gamma = pose
    
        self.set_cart_vel(5)
        self.move_pose(x, y, z, alpha, beta, gamma)
    
        if self.stepper:
            self.stepper.reverse(reverse_speed)
            time.sleep(duration)
            self.stepper.stop()
    
        print("[Meca500] Returning to safe pose...")
        self.move_pose(*return_safe_pose)
        self.move_joints(0, 0, 0, 0, 0, 0)
    
        print("[Meca500] Dump complete. Nice")



    def auger(
        self,
        A1: Tuple[float, float],
        A12: Tuple[float, float],
        H12: Tuple[float, float],
        z_height: float = 308,
        rows: int = 8,
        cols: int = 12,
        angles: Tuple[float, float, float] = (0, 90, 0),
        skip_columns: Optional[str] = None,
        return_safe_pose: Tuple[float, float, float, float, float, float] = (190, 0, 308, 0, 90, 0),
        collection_pose: Tuple[float, float, float, float, float, float] = (103.5, -62, 245, 90, 59.06, -90),
        collect_kwargs: Optional[dict] = None,
        deposit_kwargs: Optional[dict] = None
    ):
        """Perform soil collection + 3-well deposit routine using reference points."""
        print("[Meca500] Starting auger routine...")
    
        if collect_kwargs is None:
            collect_kwargs = {}
        if deposit_kwargs is None:
            deposit_kwargs = {}
    
        if not hasattr(self, "_auger_row"):
            self._auger_row = 0
            self._auger_col = 0
    
        x0, y0 = A1
        x1, y1 = A12
        x2, y2 = H12
        alpha, beta, gamma = angles
    
        row_dx = (x1 - x0) / (cols - 1)
        row_dy = (y1 - y0) / (cols - 1)
        col_dx = (x2 - x1) / (rows - 1)
        col_dy = (y2 - y1) / (rows - 1)
    
        # Move to safe pose first
        print("[Meca500] Moving to safe pose before collection")
        self.move_pose(*return_safe_pose)
    
        # Collect material
        self.collect(pose=collection_pose, **collect_kwargs)
    
        deposits_done = 0
    
        while deposits_done < 3 and self._auger_row < rows:
            col_range = range(cols) if self._auger_row % 2 == 0 else range(cols - 1, -1, -1)
    
            for c in col_range[self._auger_col:]:
                # Check skip logic
                if skip_columns == 'Even' and c % 2 == 1:
                    continue
                if skip_columns == 'Odd' and c % 2 == 0:
                    continue
    
                x = x0 + c * row_dx + self._auger_row * col_dx
                y = y0 + c * row_dy + self._auger_row * col_dy
    
                pose = (x, y, z_height, alpha, beta, gamma)
                print(f"[Meca500] Depositing at ({self._auger_row}, {c}) — {pose}")
                self.deposit(pose=pose, **deposit_kwargs)
                deposits_done += 1
    
                self._auger_col = c + 1
                if deposits_done >= 3:
                    break
    
            if self._auger_col >= cols:
                self._auger_row += 1
                self._auger_col = 0
    
        # Return to safe pose
        print("[Meca500] Returning to safe pose.")
        self.move_pose(*return_safe_pose)
        self.move_joints(0, 0, 0, 0, 0, 0)

    def reset_auger_progress(self):
        self._auger_row = 0
        self._auger_col = 0
        print("[Meca500] Auger progress reset.")