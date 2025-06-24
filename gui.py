import sys
import threading
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit, QFormLayout,
    QLineEdit, QCheckBox, QComboBox, QLabel, QInputDialog, QGroupBox, QHBoxLayout
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

        self.A1 = (140.63, -49.80)
        self.A12 = (139.34, 49.34)
        self.H12 = (202.46, 49.77)
        self.z_height = 218
        self.rows = 8
        self.cols = 12
        self.angles = (0, 90, 0)
        self.run_cleaning = False
        self.skip_columns = None
        self.tap_params = {}  # New

    def run(self):
        try:
            self.arm.connect()
            self.arm.activate_and_home()
            self.message.emit("Running grid routine...")

            self.arm.tap_params = self.tap_params  # Pass GUI tap settings into arm

            self.arm.grid_from_references(
                A1=self.A1,
                A12=self.A12,
                H12=self.H12,
                z_height=self.z_height,
                rows=self.rows,
                cols=self.cols,
                angles=self.angles,
                run_cleaning=self.run_cleaning,
                skip_columns=self.skip_columns,
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
        self.stepper = Stepper()
        self.arm = Meca500(stepper=self.stepper)
        self.worker = None

        self.layout = QVBoxLayout()

        # --- Grid Configuration Group ---
        grid_group = QGroupBox("Grid Configuration")
        grid_layout = QFormLayout()

        self.input_a1_x = QLineEdit("140.63")
        self.input_a1_y = QLineEdit("-49.80")
        grid_layout.addRow("A1 X:", self.input_a1_x)
        grid_layout.addRow("A1 Y:", self.input_a1_y)

        self.input_a12_x = QLineEdit("139.34")
        self.input_a12_y = QLineEdit("49.34")
        grid_layout.addRow("A12 X:", self.input_a12_x)
        grid_layout.addRow("A12 Y:", self.input_a12_y)

        self.input_h12_x = QLineEdit("202.46")
        self.input_h12_y = QLineEdit("49.77")
        grid_layout.addRow("H12 X:", self.input_h12_x)
        grid_layout.addRow("H12 Y:", self.input_h12_y)

        self.input_z_height = QLineEdit("218")
        grid_layout.addRow("Z Height:", self.input_z_height)

        self.cleaning_checkbox = QCheckBox("Run Cleaning")
        grid_layout.addRow(self.cleaning_checkbox)

        self.skip_dropdown = QComboBox()
        self.skip_dropdown.addItems(["None", "Even", "Odd"])
        grid_layout.addRow(QLabel("Skip Columns:"), self.skip_dropdown)

        grid_group.setLayout(grid_layout)
        self.layout.addWidget(grid_group)

        # --- Tap Parameters Group ---
        tap_group = QGroupBox("Tap Parameters")
        tap_layout = QFormLayout()

        self.tap_distance = QLineEdit("8")
        tap_layout.addRow("Distance (mm):", self.tap_distance)

        self.tap_pause = QLineEdit("0.5")
        tap_layout.addRow("Pause (s):", self.tap_pause)

        self.tap_cart_vel = QLineEdit("1")
        tap_layout.addRow("Velocity (mm/s):", self.tap_cart_vel)

        self.tap_ramp_time = QLineEdit("2")
        tap_layout.addRow("Ramp Time (s):", self.tap_ramp_time)

        self.tap_speed = QLineEdit("1800")
        tap_layout.addRow("Target Speed:", self.tap_speed)

        tap_group.setLayout(tap_layout)
        self.layout.addWidget(tap_group)

        # --- Preset Controls Row ---
        preset_row = QHBoxLayout()
        self.presets_dropdown = QComboBox()
        self.load_presets()
        preset_row.addWidget(self.presets_dropdown)

        self.load_btn = QPushButton("Load Preset")
        self.load_btn.clicked.connect(self.load_selected_preset)
        preset_row.addWidget(self.load_btn)

        self.save_btn = QPushButton("Save Preset")
        self.save_btn.clicked.connect(self.save_preset)
        preset_row.addWidget(self.save_btn)

        self.layout.addLayout(preset_row)

        # --- Command Buttons ---
        button_row = QHBoxLayout()

        self.start_btn = QPushButton("Start Grid Routine")
        self.start_btn.clicked.connect(self.start_routine)
        button_row.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_routine)
        self.stop_btn.setEnabled(False)
        button_row.addWidget(self.stop_btn)

        self.reset_btn = QPushButton("Reset Error")
        self.reset_btn.clicked.connect(self.reset_error)
        button_row.addWidget(self.reset_btn)

        self.layout.addLayout(button_row)

        # --- Status Display ---
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.layout.addWidget(self.status_display)

        self.setLayout(self.layout)

    def log(self, message):
        self.status_display.append(message)

    def load_presets(self):
        self.presets_file = "presets.json"
        if not os.path.exists(self.presets_file):
            with open(self.presets_file, "w") as f:
                json.dump({}, f)

        with open(self.presets_file, "r") as f:
            self.presets = json.load(f)

        self.presets_dropdown.clear()
        self.presets_dropdown.addItems(self.presets.keys())

    def save_preset(self):
        name, ok = QInputDialog.getText(self, "Save Preset", "Preset name:")
        if not ok or not name.strip():
            return
        name = name.strip()

        preset = {
            "A1": [float(self.input_a1_x.text()), float(self.input_a1_y.text())],
            "A12": [float(self.input_a12_x.text()), float(self.input_a12_y.text())],
            "H12": [float(self.input_h12_x.text()), float(self.input_h12_y.text())],
            "z_height": float(self.input_z_height.text()),
            "run_cleaning": self.cleaning_checkbox.isChecked(),
            "skip_columns": self.skip_dropdown.currentText(),
            "tap_params": {
                "distance_mm": float(self.tap_distance.text()),
                "pause_sec": float(self.tap_pause.text()),
                "cart_vel": float(self.tap_cart_vel.text()),
                "ramp_time": float(self.tap_ramp_time.text()),
                "target_speed": int(self.tap_speed.text())
            }
        }
        if preset["skip_columns"] == "None":
            preset["skip_columns"] = None

        self.presets[name] = preset
        with open(self.presets_file, "w") as f:
            json.dump(self.presets, f, indent=2)

        self.load_presets()
        self.log(f"Preset '{name}' saved.")

    def load_selected_preset(self):
        name = self.presets_dropdown.currentText()
        if not name or name not in self.presets:
            self.log("No preset selected.")
            return

        preset = self.presets[name]
        self.input_a1_x.setText(str(preset["A1"][0]))
        self.input_a1_y.setText(str(preset["A1"][1]))
        self.input_a12_x.setText(str(preset["A12"][0]))
        self.input_a12_y.setText(str(preset["A12"][1]))
        self.input_h12_x.setText(str(preset["H12"][0]))
        self.input_h12_y.setText(str(preset["H12"][1]))
        self.input_z_height.setText(str(preset["z_height"]))
        self.cleaning_checkbox.setChecked(preset["run_cleaning"])

        idx = self.skip_dropdown.findText(preset["skip_columns"] or "None")
        self.skip_dropdown.setCurrentIndex(idx)

        tap = preset.get("tap_params", {})
        self.tap_distance.setText(str(tap.get("distance_mm", 8)))
        self.tap_pause.setText(str(tap.get("pause_sec", 0.5)))
        self.tap_cart_vel.setText(str(tap.get("cart_vel", 1)))
        self.tap_ramp_time.setText(str(tap.get("ramp_time", 2)))
        self.tap_speed.setText(str(tap.get("target_speed", 1800)))

        self.log(f"Preset '{name}' loaded.")

    def start_routine(self):
        self.log("Starting grid routine...")
        try:
            A1 = (float(self.input_a1_x.text()), float(self.input_a1_y.text()))
            A12 = (float(self.input_a12_x.text()), float(self.input_a12_y.text()))
            H12 = (float(self.input_h12_x.text()), float(self.input_h12_y.text()))
            z_height = float(self.input_z_height.text())
            run_cleaning = self.cleaning_checkbox.isChecked()
            skip_val = self.skip_dropdown.currentText()
            skip_columns = None if skip_val == "None" else skip_val
        except ValueError as e:
            self.log(f"Invalid input: {e}")
            return

        self.worker = RobotWorker(self.arm, self.stepper)
        self.worker.A1 = A1
        self.worker.A12 = A12
        self.worker.H12 = H12
        self.worker.z_height = z_height
        self.worker.run_cleaning = run_cleaning
        self.worker.skip_columns = skip_columns
        self.worker.tap_params = {
            "distance_mm": float(self.tap_distance.text()),
            "pause_sec": float(self.tap_pause.text()),
            "cart_vel": float(self.tap_cart_vel.text()),
            "ramp_time": float(self.tap_ramp_time.text()),
            "target_speed": int(self.tap_speed.text())
        }

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



# # -*- coding: utf-8 -*-
# """
# Created on Wed Jun 18 10:59:29 2025

# @author: elija
# """
# import sys
# import threading
# import json
# import os

# from PyQt5.QtWidgets import (
#     QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit, QFormLayout,
#     QLineEdit, QCheckBox, QComboBox, QLabel, QInputDialog
# )
# from PyQt5.QtCore import QThread, pyqtSignal

# from meca500 import Meca500
# from stepper import Stepper

# class RobotWorker(QThread):
#     finished = pyqtSignal()
#     message = pyqtSignal(str)

#     def __init__(self, arm, stepper):
#         super().__init__()
#         self.arm = arm
#         self.stepper = stepper
#         self._stop_event = threading.Event()

        
#         self.A1 = (140.63, -49.80)
#         self.A12 = (139.34, 49.34)
#         self.H12 = (202.46, 49.77)
#         self.z_height = 218
#         self.rows = 8
#         self.cols = 12
#         self.angles = (0, 90, 0)
#         self.run_cleaning = False
#         self.skip_columns = None

#     def run(self):
#         try:
#             self.arm.connect()
#             self.arm.activate_and_home()
#             self.message.emit("Running grid routine...")

#             self.arm.grid_from_references(
#                 A1=self.A1,
#                 A12=self.A12,
#                 H12=self.H12,
#                 z_height=self.z_height,
#                 rows=self.rows,
#                 cols=self.cols,
#                 angles=self.angles,
#                 run_cleaning=self.run_cleaning,
#                 skip_columns=self.skip_columns,
#                 should_stop=self._stop_event
#             )

#         except Exception as e:
#             self.message.emit(f"ERROR: {e}")

#         finally:
#             self.arm.disconnect()
#             self.finished.emit()

#     def stop(self):
#         self._stop_event.set()



# class GUI(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Meca500 Controller")
#         self.stepper = Stepper()
#         self.arm = Meca500(stepper=self.stepper)
#         self.worker = None

#         # UI Elements
#         self.layout = QVBoxLayout()

#         # Load/Save Presets
#         self.presets_dropdown = QComboBox()
#         self.load_presets()
#         self.layout.addWidget(self.presets_dropdown)
        
#         self.save_btn = QPushButton("Save Preset")
#         self.save_btn.clicked.connect(self.save_preset)
#         self.layout.addWidget(self.save_btn)
        
#         self.load_btn = QPushButton("Load Preset")
#         self.load_btn.clicked.connect(self.load_selected_preset)
#         self.layout.addWidget(self.load_btn)


        

#         # Buttons
#         self.start_btn = QPushButton("Start Grid Routine")
#         self.start_btn.clicked.connect(self.start_routine)
#         self.layout.addWidget(self.start_btn)

#         self.stop_btn = QPushButton("Stop")
#         self.stop_btn.clicked.connect(self.stop_routine)
#         self.stop_btn.setEnabled(False)
#         self.layout.addWidget(self.stop_btn)

#         self.reset_btn = QPushButton("Reset Error")
#         self.reset_btn.clicked.connect(self.reset_error)
#         self.layout.addWidget(self.reset_btn)

#         # CMD output
#         self.status_display = QTextEdit()
#         self.status_display.setReadOnly(True)
#         self.layout.addWidget(self.status_display)

#         # Grid Parameter Inputs
#         form_layout = QFormLayout()
        
#         self.input_a1_x = QLineEdit("140.63")
#         self.input_a1_y = QLineEdit("-49.80")
#         form_layout.addRow("A1 X:", self.input_a1_x)
#         form_layout.addRow("A1 Y:", self.input_a1_y)
        
#         self.input_a12_x = QLineEdit("139.34")
#         self.input_a12_y = QLineEdit("49.34")
#         form_layout.addRow("A12 X:", self.input_a12_x)
#         form_layout.addRow("A12 Y:", self.input_a12_y)
        
#         self.input_h12_x = QLineEdit("202.46")
#         self.input_h12_y = QLineEdit("49.77")
#         form_layout.addRow("H12 X:", self.input_h12_x)
#         form_layout.addRow("H12 Y:", self.input_h12_y)
        
#         self.input_z_height = QLineEdit("218")
#         form_layout.addRow("Z Height:", self.input_z_height)
        
#         self.cleaning_checkbox = QCheckBox("Run Cleaning")
#         self.cleaning_checkbox.setChecked(False)
#         form_layout.addRow(self.cleaning_checkbox)
        
#         self.skip_dropdown = QComboBox()
#         self.skip_dropdown.addItems(["None", "Even", "Odd"])
#         form_layout.addRow(QLabel("Skip Columns:"), self.skip_dropdown)
        
#         self.layout.addLayout(form_layout)

#         self.setLayout(self.layout)

#     def log(self, message):
#         self.status_display.append(message)

#     def load_presets(self):
#         self.presets_file = "presets.json"
#         if not os.path.exists(self.presets_file):
#             with open(self.presets_file, "w") as f:
#                 json.dump({}, f)
    
#         with open(self.presets_file, "r") as f:
#             self.presets = json.load(f)
    
#         self.presets_dropdown.clear()
#         self.presets_dropdown.addItems(self.presets.keys())
    
#     def save_preset(self):
#         name, ok = QInputDialog.getText(self, "Save Preset", "Preset name:")
#         if not ok or not name.strip():
#             return
#         name = name.strip()
    
#         # Gather values from GUI
#         preset = {
#             "A1": [float(self.input_a1_x.text()), float(self.input_a1_y.text())],
#             "A12": [float(self.input_a12_x.text()), float(self.input_a12_y.text())],
#             "H12": [float(self.input_h12_x.text()), float(self.input_h12_y.text())],
#             "z_height": float(self.input_z_height.text()),
#             "run_cleaning": self.cleaning_checkbox.isChecked(),
#             "skip_columns": self.skip_dropdown.currentText()
#         }
#         if preset["skip_columns"] == "None":
#             preset["skip_columns"] = None
    
#         self.presets[name] = preset
#         with open(self.presets_file, "w") as f:
#             json.dump(self.presets, f, indent=2)
    
#         self.load_presets()
#         self.log(f"Preset '{name}' saved.")
    
#     def load_selected_preset(self):
#         name = self.presets_dropdown.currentText()
#         if not name or name not in self.presets:
#             self.log("No preset selected.")
#             return
    
#         preset = self.presets[name]
#         self.input_a1_x.setText(str(preset["A1"][0]))
#         self.input_a1_y.setText(str(preset["A1"][1]))
#         self.input_a12_x.setText(str(preset["A12"][0]))
#         self.input_a12_y.setText(str(preset["A12"][1]))
#         self.input_h12_x.setText(str(preset["H12"][0]))
#         self.input_h12_y.setText(str(preset["H12"][1]))
#         self.input_z_height.setText(str(preset["z_height"]))
#         self.cleaning_checkbox.setChecked(preset["run_cleaning"])
    
#         idx = self.skip_dropdown.findText(preset["skip_columns"] or "None")
#         self.skip_dropdown.setCurrentIndex(idx)
    
#         self.log(f"Preset '{name}' loaded.")


#     def start_routine(self):
#         self.log("Starting grid routine...")
    
#         try:
#             # Parse GUI input fields
#             A1 = (float(self.input_a1_x.text()), float(self.input_a1_y.text()))
#             A12 = (float(self.input_a12_x.text()), float(self.input_a12_y.text()))
#             H12 = (float(self.input_h12_x.text()), float(self.input_h12_y.text()))
#             z_height = float(self.input_z_height.text())
#             run_cleaning = self.cleaning_checkbox.isChecked()
    
#             skip_val = self.skip_dropdown.currentText()
#             skip_columns = None if skip_val == "None" else skip_val
    
#         except ValueError as e:
#             self.log(f"Invalid input: {e}")
#             return
    
#         # Set up and launch worker thread
#         self.worker = RobotWorker(self.arm, self.stepper)
#         self.worker.A1 = A1
#         self.worker.A12 = A12
#         self.worker.H12 = H12
#         self.worker.z_height = z_height
#         self.worker.run_cleaning = run_cleaning
#         self.worker.skip_columns = skip_columns
    
#         self.worker.message.connect(self.log)
#         self.worker.finished.connect(self.routine_done)
#         self.worker.start()
    
#         self.start_btn.setEnabled(False)
#         self.stop_btn.setEnabled(True)

#     def stop_routine(self):
#         self.log("Stop button pressed.")
#         if self.worker:
#             self.worker.stop()
#             self.log("Stop signal sent to robot routine. Waiting for thread to finish...")
#             self.stop_btn.setEnabled(False)

#     def routine_done(self):
#         self.log("Routine finished.")
#         self.start_btn.setEnabled(True)
#         self.stop_btn.setEnabled(False)

#         if self.worker and self.worker._stop_event.is_set():
#             self.log("Routine was stopped. Attempting recovery...")
#             try:
#                 self.arm.connect()
#                 self.arm.robot.ResetError()
#                 self.arm.robot.ActivateRobot()
#                 self.arm.move_pose(190, 0, 308, 0, 90, 0)
#                 self.log("Recovery complete.")
#             except Exception as e:
#                 self.log(f"Recovery failed: {e}")

#     def reset_error(self):
#         self.log("Resetting robot error...")
#         try:
#             self.arm.connect()
#             self.arm.robot.ResetError()
#             self.arm.robot.ActivateRobot()
#             self.log("Robot error reset successfully.")
#         except Exception as e:
#             self.log(f"Failed to reset error: {e}")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     gui = GUI()
#     gui.show()
#     sys.exit(app.exec_())



