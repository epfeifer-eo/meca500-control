# -*- coding: utf-8 -*-
"""
Created on Wed Jun 18 10:59:29 2025

@author: elija
"""
import sys
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit
)
from PyQt5.QtCore import QThread, pyqtSignal

from meca500 import Meca500
from stepper import Stepper


class RobotWorker(QThread):
    finished = pyqtSignal()
    message = pyqtSignal(str)

    def __init__(self, arm, stepper):
        super().__init__()
        self.arm = arm
        self.stepper = stepper
        self._stop_event = threading.Event()

    def run(self):
        try:
            self.arm.connect()
            self.arm.activate_and_home()
            self.message.emit("Running grid routine...")

            self.arm.grid_from_references(
                A1=(140.63, -49.80),
                A12=(139.34, 49.34),
                H12=(202.46, 49.77),
                z_height=218,
                run_cleaning=False,
                skip_columns='even',
                should_stop=self._stop_event
            )

        except Exception as e:
            self.message.emit(f"ERROR: {e}")

        finally:
            self.arm.disconnect()
            self.finished.emit()

    def stop(self):
        self._stop_event.set()


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Meca500 Controller")
        self.arm = Meca500()
        self.stepper = Stepper()
        self.worker = None

        # UI Elements
        self.layout = QVBoxLayout()

        self.start_btn = QPushButton("Start Grid Routine")
        self.start_btn.clicked.connect(self.start_routine)
        self.layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_routine)
        self.stop_btn.setEnabled(False)
        self.layout.addWidget(self.stop_btn)

        self.reset_btn = QPushButton("Reset Error")
        self.reset_btn.clicked.connect(self.reset_error)
        self.layout.addWidget(self.reset_btn)

        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.layout.addWidget(self.status_display)

        self.setLayout(self.layout)

    def log(self, message):
        self.status_display.append(message)

    def start_routine(self):
        self.log("Starting grid routine...")
        self.worker = RobotWorker(self.arm, self.stepper)
        self.worker.message.connect(self.log)
        self.worker.finished.connect(self.routine_done)
        self.worker.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_routine(self):
        self.log("Stop button pressed.")
        if self.worker:
            self.worker.stop()
            self.log("Stop signal sent to robot routine. Waiting for thread to finish...")
            self.stop_btn.setEnabled(False)

    def routine_done(self):
        self.log("Routine finished.")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        if self.worker and self.worker._stop_event.is_set():
            self.log("Routine was stopped. Attempting recovery...")
            try:
                self.arm.connect()
                self.arm.robot.ResetError()
                self.arm.robot.ActivateRobot()
                self.arm.move_pose(190, 0, 308, 0, 90, 0)
                self.log("Recovery complete.")
            except Exception as e:
                self.log(f"Recovery failed: {e}")

    def reset_error(self):
        self.log("Resetting robot error...")
        try:
            self.arm.connect()
            self.arm.robot.ResetError()
            self.arm.robot.ActivateRobot()
            self.log("Robot error reset successfully.")
        except Exception as e:
            self.log(f"Failed to reset error: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = GUI()
    gui.show()
    sys.exit(app.exec_())


