# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:24 2025

@author: elija
"""

import sys

if sys.platform == 'linux':
    from gpiozero import DigitalOutputDevice
else:
    # Dummy fallback classes for non-RPi environments
    class DigitalOutputDevice:
        def __init__(self, pin):
            self.pin = pin
        def on(self):
            print(f"[Mock] GPIO {self.pin} ON")
        def off(self):
            print(f"[Mock] GPIO {self.pin} OFF")

import time
import threading
# from gpiozero import DigitalOutputDevice

class Stepper:
    def __init__(self, pul_pin=17, dir_pin=27, step_delay=0.001):
        self.pul = DigitalOutputDevice(pul_pin)
        self.dir = DigitalOutputDevice(dir_pin)
        self.running = False
        self.step_delay = step_delay  # Seconds between steps
        self._lock = threading.Lock()
        self._thread = None

    def _step_loop(self):
        while self.running:
            self.pul.on()
            time.sleep(self.step_delay / 2)
            self.pul.off()
            time.sleep(self.step_delay / 2)

    def _start(self):
        with self._lock:
            if not self.running:
                self.running = True
                self._thread = threading.Thread(target=self._step_loop, daemon=True)
                self._thread.start()

    def forward(self):
        print("[Stepper] Moving forward")
        self.dir.on()
        self._start()

    def reverse(self):
        print("[Stepper] Moving reverse")
        self.dir.off()
        self._start()

    def stop(self):
        print("[Stepper] Stopping")
        with self._lock:
            self.running = False
        if self._thread:
            self._thread.join()
            self._thread = None

    def set_speed(self, steps_per_sec):
        """
        Sets stepper speed. Higher is faster.
        """
        if steps_per_sec <= 0:
            self.step_delay = None
            print("[Stepper] Speed set to 0 (stopped)")
            return
        self.step_delay = 1.0 / steps_per_sec
        print(f"[Stepper] Speed set to {steps_per_sec} steps/sec")


    def set_acceleration(self, ramp_time=1.0, target_speed=500):
        """
        Gradually increases speed over ramp_time (in seconds).
        """
        print(f"[Stepper] Ramping up to {target_speed} steps/sec over {ramp_time}s")
    
        if target_speed <= 0:
            self.stop()
            return
        
        self.forward()
        
        steps = 20
        for i in range(1, steps + 1):
            speed = (target_speed / steps) * i
            self.set_speed(speed)
            time.sleep(ramp_time / steps)


    def cleanup(self):
        print("[Stepper] Cleaning up GPIO")
        self.stop()
        self.pul.close()
        self.dir.close()

