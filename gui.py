"""
This is the file you are looking for. If on my pc and in the correct directory run: 
>poetry shell
>python gui.py

If on rpi5 navigate to the root directory:
>source venv/bin/activate
>python gui.py 

"""

import sys
import threading
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit, QFormLayout,
    QLineEdit, QCheckBox, QComboBox, QLabel, QInputDialog, QGroupBox, QHBoxLayout, QScrollArea
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

        # Routine parameters
        self.A1 = (140.63, -49.80)
        self.A12 = (139.34, 49.34)
        self.H12 = (202.46, 49.77)
        self.z_height = 250
        self.rows = 8
        self.cols = 12
        self.angles = (0, 90, 0)
        self.run_cleaning = False
        self.skip_columns = None
        self.tap_params = {}
        self.mode = "Grid Routine"

        # Auger-specific parameters
        self.collect_kwargs = {}
        self.deposit_kwargs = {}

    def run(self):
        try:
            self.arm.connect()
            self.arm.activate_and_home()

            if self.mode == "Auger Routine":
                self.message.emit("Running auger routine...")
                self.arm.auger(
                    A1=self.A1,
                    A12=self.A12,
                    H12=self.H12,
                    z_height=self.z_height,
                    rows=self.rows,
                    cols=self.cols,
                    angles=self.angles,
                    skip_columns=self.skip_columns,
                    collect_kwargs=self.collect_kwargs,
                    deposit_kwargs=self.deposit_kwargs
                )
            else:
                self.message.emit("Running grid routine...")
                self.arm.tap_params = self.tap_params
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
        self.mode_selector = QComboBox()
        self.mode_selector.addItems(["Grid Routine", "Auger Routine"])
        self.mode_selector.currentTextChanged.connect(self.update_start_button_label)
        grid_layout.addRow(QLabel("Mode:"), self.mode_selector)


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

        # --- Collect Parameters Group ---
        collect_group = QGroupBox("Collect Parameters")
        collect_layout = QFormLayout()
        
        self.collect_surface_offset = QLineEdit("10")
        collect_layout.addRow("Surface Offset (mm):", self.collect_surface_offset)
        
        self.collect_drill_depth = QLineEdit("8")
        collect_layout.addRow("Drill Depth (mm):", self.collect_drill_depth)
        
        self.collect_speed = QLineEdit("1200")
        collect_layout.addRow("Speed (steps/sec):", self.collect_speed)
        
        self.collect_cart_vel = QLineEdit("2.0")
        collect_layout.addRow("Velocity (mm/s):", self.collect_cart_vel)
        
        self.collect_pause = QLineEdit("0.5")
        collect_layout.addRow("Pause (s):", self.collect_pause)
        
        collect_group.setLayout(collect_layout)
        self.layout.addWidget(collect_group)
        
        # --- Deposit Parameters Group ---
        deposit_group = QGroupBox("Deposit Parameters")
        deposit_layout = QFormLayout()
        
        self.deposit_depth = QLineEdit("5")
        deposit_layout.addRow("Deposit Depth (mm):", self.deposit_depth)
        
        self.deposit_speed = QLineEdit("800")
        deposit_layout.addRow("Speed (steps/sec):", self.deposit_speed)
        
        self.deposit_cart_vel = QLineEdit("2.0")
        deposit_layout.addRow("Velocity (mm/s):", self.deposit_cart_vel)
        
        self.deposit_pause = QLineEdit("1.5")
        deposit_layout.addRow("Pause (s):", self.deposit_pause)
        
        deposit_group.setLayout(deposit_layout)
        self.layout.addWidget(deposit_group)

        # --- Dump Parameters Group ---
        dump_group = QGroupBox("Dump Parameters")
        dump_layout = QFormLayout()
        
        self.dump_speed = QLineEdit("800")
        dump_layout.addRow("Reverse Speed (steps/sec):", self.dump_speed)
        
        self.dump_duration = QLineEdit("2.0")
        dump_layout.addRow("Duration (s):", self.dump_duration)
        
        dump_group.setLayout(dump_layout)
        self.layout.addWidget(dump_group)

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
        self.update_start_button_label(self.mode_selector.currentText())
        self.start_btn.clicked.connect(self.start_routine)
        button_row.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_routine)
        self.stop_btn.setEnabled(False)
        button_row.addWidget(self.stop_btn)
        
        self.reset_plate_btn = QPushButton("New Plate")
        self.reset_plate_btn.clicked.connect(self.reset_plate)
        button_row.addWidget(self.reset_plate_btn)

        self.dump_btn = QPushButton("Dump")
        self.dump_btn.clicked.connect(self.dump_material)
        button_row.addWidget(self.dump_btn)

        self.reset_btn = QPushButton("Reset Error")
        self.reset_btn.clicked.connect(self.reset_error)
        button_row.addWidget(self.reset_btn)

        self.layout.addLayout(button_row)

        # --- Status Display ---
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.layout.addWidget(self.status_display)

        # --- Wrap layout in a scrollable container ---
        scroll_area = QScrollArea()
        container = QWidget()
        container.setLayout(self.layout)
        scroll_area.setWidget(container)
        scroll_area.setWidgetResizable(True)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(scroll_area)
        self.setLayout(main_layout)
        
    def update_start_button_label(self, mode):
        if "Auger" in mode:
            self.start_btn.setText("Start Auger Routine")
        else:
            self.start_btn.setText("Start Grid Routine")

    def reset_plate(self):
        self.log("Resetting plate position...")
        self.arm.reset_auger_progress()
        self.log("Auger plate index reset.")

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
            },
            "collect_params": {
                "surface_offset_mm": float(self.collect_surface_offset.text()),
                "drill_depth_mm": float(self.collect_drill_depth.text()),
                "speed": int(self.collect_speed.text()),
                "cart_vel": float(self.collect_cart_vel.text()),
                "pause_sec": float(self.collect_pause.text())
            },
            "deposit_params": {
                "deposit_depth_mm": float(self.deposit_depth.text()),
                "speed": int(self.deposit_speed.text()),
                "cart_vel": float(self.deposit_cart_vel.text()),
                "pause_sec": float(self.deposit_pause.text())
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
    
        collect = preset.get("collect_params", {})
        self.collect_surface_offset.setText(str(collect.get("surface_offset_mm", 10)))
        self.collect_drill_depth.setText(str(collect.get("drill_depth_mm", 8)))
        self.collect_speed.setText(str(collect.get("speed", 1200)))
        self.collect_cart_vel.setText(str(collect.get("cart_vel", 2.0)))
        self.collect_pause.setText(str(collect.get("pause_sec", 0.5)))
    
        deposit = preset.get("deposit_params", {})
        self.deposit_depth.setText(str(deposit.get("deposit_depth_mm", 5)))
        self.deposit_speed.setText(str(deposit.get("speed", 800)))
        self.deposit_cart_vel.setText(str(deposit.get("cart_vel", 2.0)))
        self.deposit_pause.setText(str(deposit.get("pause_sec", 1.5)))
    
        self.log(f"Preset '{name}' loaded.")

    def start_routine(self):
        self.log("Starting routine...")
        try:
            A1 = (float(self.input_a1_x.text()), float(self.input_a1_y.text()))
            A12 = (float(self.input_a12_x.text()), float(self.input_a12_y.text()))
            H12 = (float(self.input_h12_x.text()), float(self.input_h12_y.text()))
            z_height = float(self.input_z_height.text())
            run_cleaning = self.cleaning_checkbox.isChecked()
            skip_val = self.skip_dropdown.currentText()
            skip_columns = None if skip_val == "None" else skip_val
            mode = self.mode_selector.currentText()
        except ValueError as e:
            self.log(f"Invalid input: {e}")
            return
    
        self.worker = RobotWorker(self.arm, self.stepper)
        self.worker.mode = mode
        self.worker.A1 = A1
        self.worker.A12 = A12
        self.worker.H12 = H12
        self.worker.z_height = z_height
        self.worker.run_cleaning = run_cleaning
        self.worker.skip_columns = skip_columns
    
        self.worker.rows = 8
        self.worker.cols = 12
    
        self.worker.tap_params = {
            "distance_mm": float(self.tap_distance.text()),
            "pause_sec": float(self.tap_pause.text()),
            "cart_vel": float(self.tap_cart_vel.text()),
            "ramp_time": float(self.tap_ramp_time.text()),
            "target_speed": int(self.tap_speed.text())
        }
    
        self.worker.collect_kwargs = {
            "surface_offset_mm": float(self.collect_surface_offset.text()),
            "drill_depth_mm": float(self.collect_drill_depth.text()),
            "speed": int(self.collect_speed.text()),
            "cart_vel": float(self.collect_cart_vel.text()),
            "pause_sec": float(self.collect_pause.text())
        }
    
        self.worker.deposit_kwargs = {
            "deposit_depth_mm": float(self.deposit_depth.text()),
            "speed": int(self.deposit_speed.text()),
            "cart_vel": float(self.deposit_cart_vel.text()),
            "pause_sec": float(self.deposit_pause.text())
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

    def dump_material(self):
        self.log("Performing dump...")
        try:
            reverse_speed = int(self.dump_speed.text())
            duration = float(self.dump_duration.text())
    
            self.arm.connect()
            self.arm.activate_and_home()
            self.arm.dump(reverse_speed=reverse_speed, duration=duration)
            self.log("Dump complete.")
        except Exception as e:
            self.log(f"Dump failed: {e}")



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

