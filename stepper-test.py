# -*- coding: utf-8 -*-
"""
Created on Thu May 29 13:17:00 2025

@author: elija
"""

import RPi.GPIO as GPIO
import time

# Define pin numbers (BCM numbering)
DIR = 27   # GPIO27, pin 13
STEP = 17  # GPIO17, pin 11
DELAY = 0.001  # 1 ms pulse (speed)

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Set direction: HIGH = one direction, LOW = the other
GPIO.output(DIR, GPIO.HIGH)

# Move 200 steps (1 rev at full step)
for _ in range(200):
    GPIO.output(STEP, GPIO.HIGH)
    time.sleep(DELAY)
    GPIO.output(STEP, GPIO.LOW)
    time.sleep(DELAY)

# Cleanup GPIO
GPIO.cleanup()
