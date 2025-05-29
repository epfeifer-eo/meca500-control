# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:24 2025

@author: elija
"""

import time
import platform

IS_PI = "raspberrypi" in platform.uname().nodename.lower()

if IS_PI:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
else:
    class MockGPIO:
        BCM = None
        OUT = None
        def setmode(self, *_): pass
        def setup(self, *_): pass
        def output(self, *_): pass
        def cleanup(self): pass
    GPIO = MockGPIO()

from config import STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_ENABLE_PIN

class StepperDriver:
    def __init__(self):
        GPIO.setup(STEPPER_STEP_PIN, GPIO.OUT)
        GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT)
        GPIO.setup(STEPPER_ENABLE_PIN, GPIO.OUT)
        self.enabled = False
        self.disable()

    def enable(self):
        GPIO.output(STEPPER_ENABLE_PIN, GPIO.LOW)  # LOW = enable
        self.enabled = True

    def disable(self):
        GPIO.output(STEPPER_ENABLE_PIN, GPIO.HIGH)  # HIGH = disabled
        self.enabled = False

    def step(self, direction=True, steps=200, delay=0.001):
        GPIO.output(STEPPER_DIR_PIN, GPIO.HIGH if direction else GPIO.LOW)
        for _ in range(steps):
            GPIO.output(STEPPER_STEP_PIN, GPIO.HIGH)
            time.sleep(delay / 2)
            GPIO.output(STEPPER_STEP_PIN, GPIO.LOW)
            time.sleep(delay / 2)

    def cleanup(self):
        GPIO.cleanup()
