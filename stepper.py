# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:24 2025

@author: elija
"""
import time
import sys

if sys.platform == 'linux':
    from gpiozero import DigitalOutputDevice
else:
    class DigitalOutputDevice:
        def __init__(self, pin): self.pin = pin
        def on(self): print(f"[Mock] GPIO {self.pin} ON")
        def off(self): print(f"[Mock] GPIO {self.pin} OFF")
        def close(self): print(f"[Mock] GPIO {self.pin} CLOSED")


# Setup pins
PULSE_PIN = 17
DIR_PIN = 27

pulse = DigitalOutputDevice(PULSE_PIN)
direction = DigitalOutputDevice(DIR_PIN)

# Configurable params
direction.on()  # FORWARD (use .off() for reverse)
steps_per_sec = 500  # Try 500–800
duration_sec = 2  # Run for 2 seconds

# Calculate delay per step
delay = 1.0 / steps_per_sec

print(f"[Stepper] Running FORWARD at {steps_per_sec} steps/sec for {duration_sec} seconds")

try:
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        pulse.on()
        time.sleep(delay / 2)
        pulse.off()
        time.sleep(delay / 2)

    print("[Stepper] Done. Motor stopped.")

finally:
    pulse.off()
    direction.off()
    pulse.close()
    direction.close()


