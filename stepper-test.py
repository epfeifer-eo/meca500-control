# -*- coding: utf-8 -*-
"""
Created on Thu May 29 13:17:00 2025

@author: elija
"""

from gpiozero import OutputDevice
from time import sleep

step = OutputDevice(17)
direction = OutputDevice(27)

direction.on()  # or .off() for other direction

for _ in range(200):
    step.on()
    sleep(0.001)
    step.off()
    sleep(0.001)

