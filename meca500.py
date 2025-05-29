# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:12 2025

@author: elija
"""


from mecademicpy.robot import Robot

class Meca500:
    def __init__(self, ip_address="192.168.0.100"):
        self.ip = ip_address
        self.robot = Robot()
        self.connected = False

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
        self.robot.WaitIdle(timeout=10)
        
    def is_pose_safe(self, x, y, z, alpha=0, beta=0, gamma=0):
        """Conservatively check if a pose is probably within reach."""
        in_bounds = (
            150 <= x <= 275 and
            -75 <= y <= 75 and
            100 <= z <= 250
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






    def disconnect(self):
        if self.connected:
            print("[Meca500] Disconnecting robot...")
            self.robot.Disconnect()
            self.connected = False
            print("[Meca500] Disconnected.")
