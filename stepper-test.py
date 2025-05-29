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

for delay in [0.01, 0.005, 0.002, 0.001]:
    print(f"Running at delay: {delay}s")
    for _ in range(50):
        step.on()
        sleep(delay)
        step.off()
        sleep(delay)
