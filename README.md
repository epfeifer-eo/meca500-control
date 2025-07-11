# Meca500 + Stepper Controller

This project integrates a Mecademic Meca500 robotic arm with a TB6600-controlled NEMA 17 stepper motor to automate soil sampling in a 96-well plate format. A PyQt5 GUI provides configuration and control for grid-based tapping, soil collection, deposition, and dumping routines.

------------------------------------------------------------------

## SYSTEM REQUIREMENTS

- Raspberry Pi 5 (or other Linux machine)
- Python 3.9+
- `mecademicpy` library for Meca500 communication
- GPIO access (`gpiozero`) for stepper control
- PyQt5 for GUI interface

------------------------------------------------------------------

## PROJECT STRUCTURE

    config.py         - IP configuration
    gui.py            - PyQt5 GUI for user interaction
    meca500.py        - Meca500 robot logic and motion sequences
    stepper.py        - Stepper motor control logic via GPIO
    presets.json      - (Optional) Saved GUI parameter presets

------------------------------------------------------------------

## SETUP INSTRUCTIONS

### On Raspberry Pi 5:

    cd /path/to/project
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    python gui.py

### On Development PC (for testing):

    poetry shell
    python gui.py

Note: Mock GPIO behavior is used on non-Linux systems for testing.

------------------------------------------------------------------

## CORE COMPONENTS

### meca500.py (Meca500 Class)

- Establishes TCP/IP connection to robot
- Supports:
  - Cartesian and joint motion
  - Tap, grid, auger, collect, deposit, dump routines
  - Safe motion bounds and pose checking
  - Grid routine sequentially homogenizes a 96 well plate
  - Auger routine delivers one collection → three deposit cycles to a 96 well plate

### stepper.py (Stepper Class)

- Controls a stepper motor via 4 GPIO pins
- Features:
  - Forward/reverse rotation
  - Speed ramping with optional async mode
  - Immediate stop and cleanup

### gui.py (GUI Class)

- PyQt5 app with user-configurable grid and motion parameters
- Features:
  - Select Grid or Auger routine
  - Set positions (A1, A12, H12), Z-height, cleaning behavior, skip rules
  - Configure tap, collect, deposit, and dump parameters
  - Save/load named presets
  - View real-time logs and start/stop execution

------------------------------------------------------------------

## ROUTINES OVERVIEW

Routine Name           | Description
-----------------------|-----------------------------------------------------------
tap()                  | Homogenization routine. Lower, spiral motion, return — used in Grid routine
grid_from_references() | 8x12 grid movement and homogenization based on 3 reference wells
auger()                | Collect from central point, deposit into 3 wells, repeat with memory
collect()              | Drill down into soil with stepper running forward
deposit()              | Drop sample into well and reverse stepper to unload
dump()                 | Reverse auger for N seconds at the collection pose

------------------------------------------------------------------

## SAFETY & RECOVERY

- All movement checks bounds using `is_pose_safe()`
- Emergency stop supported via threading flag `should_stop` NOT TESTED
- `abort_and_recover()` halts all motion and returns to a safe pose NOT TESTED
- GUI `Stop` button triggers cleanup and optional recovery logic ONLY FUNTIONAL IN GRID ROUTINE

------------------------------------------------------------------

## KNOWN SAFE POSES

- Safe pose:        (190, 0, 308, 0, 90, 0)
- Collection pose:  (103.5, -62, 245, 90, 59.06, -90)

------------------------------------------------------------------

## TESTING NOTES

- Stepper speeds are in steps/second
- Stepper ramping provides smooth start/stop
- TB6600 microstepping is set via DIP switches (16 microsteps 1.5A)
- Always verify motion via mecademic web portal before running full routines

------------------------------------------------------------------

## RECOVERY INSTRUCTIONS

If a fault occurs or the routine is stopped:

1. Press the "Reset Error" button in the GUI
2. Robot should re-home and return to the safe pose
3. Ensure the physical environment is clear before restarting
4. When in doubt - hard reset the robot arm

------------------------------------------------------------------

## ADDITIONAL NOTES

- All logs are displayed in the GUI’s status panel and in cmd line
- Presets are stored in `presets.json` in the working directory
- On shutdown, the GUI disconnects from the robot and stops the stepper

------------------------------------------------------------------