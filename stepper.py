# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:24 2025

@author: elija
"""
# stepper.py

import sys
import time
import threading

if sys.platform == 'linux':
    from gpiozero import DigitalOutputDevice
else:
    class DigitalOutputDevice:
        def __init__(self, pin): self.pin = pin
        def on(self): print(f"[Mock] GPIO {self.pin} ON")
        def off(self): print(f"[Mock] GPIO {self.pin} OFF")
        def close(self): print(f"[Mock] GPIO {self.pin} CLOSED")


class Stepper:
    def __init__(self, pul_pin=17, dir_pin=27, default_speed=600):
        self.pul = DigitalOutputDevice(pul_pin)
        self.dir = DigitalOutputDevice(dir_pin)
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._speed = default_speed  # in steps/sec

    def _step_loop(self):
        while self._running:
            with self._lock:
                delay = 1.0 / self._speed
            self.pul.on()
            time.sleep(delay / 2)
            self.pul.off()
            time.sleep(delay / 2)
            
    def _start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()
        
    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join()
            self.pul.off()
            self._thread = None
            
    def set_speed(self, speed):
        """Set the speed immediately (does not start motor)."""
        with self._lock:
            self._speed = max(1, speed)
    
    def forward(self, speed=None):
        """Set direction and start spinning (optionally with initial speed)."""
        self.dir.on()
        if speed:
            self.set_speed(speed)
        self._start()
    
    def reverse(self, speed=None):
        self.dir.off()
        if speed:
            self.set_speed(speed)
        self._start()
    
    def ramp_to_speed(self, target_speed, ramp_time=1.0, steps=20, async_mode=False):
        """Ramp the speed to a target value. Assumes motor is already running."""
        def ramp():
            with self._lock:
                current_speed = self._speed
            delta = target_speed - current_speed
            for i in range(1, steps + 1):
                speed = current_speed + (delta * i / steps)
                self.set_speed(speed)
                time.sleep(ramp_time / steps)
    
        if async_mode:
            thread = threading.Thread(target=ramp, daemon=True)
            thread.start()
        else:
            ramp()

    def cleanup(self):
        self.stop()
        self.pul.close()
        self.dir.close()



