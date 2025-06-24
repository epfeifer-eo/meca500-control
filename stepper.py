# -*- coding: utf-8 -*-
"""
Created on Thu May 22 14:03:24 2025

@author: elija
"""
import sys
import time
import threading

if sys.platform == 'linux':
    from gpiozero import DigitalOutputDevice
else:
    class DigitalOutputDevice:
        def __init__(self, pin):
            self.pin = pin
        def on(self):
            print(f"[Mock] GPIO {self.pin} ON")
        def off(self):
            print(f"[Mock] GPIO {self.pin} OFF")
        def close(self):
            print(f"[Mock] GPIO {self.pin} CLOSED")


class Stepper:
    def __init__(self, pul_pin=17, dir_pin=27):
        self.pul = DigitalOutputDevice(pul_pin)
        self.dir = DigitalOutputDevice(dir_pin)
        self._running = False
        self._speed = 800  # steps/sec default
        self._lock = threading.Lock()
        self._thread = None

    def _pulse_loop(self):
        print("[Stepper] Step loop started")
        while self._running:
            with self._lock:
                delay = 1.0 / self._speed if self._speed > 0 else 0.01
            self.pul.on()
            time.sleep(delay / 2)
            self.pul.off()
            time.sleep(delay / 2)
        print("[Stepper] Step loop exited")

    def _start_loop(self):
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._pulse_loop, daemon=True)
        self._thread.start()

    def forward(self):
        print("[Stepper] Direction: FORWARD")
        self.dir.on()
        self._start_loop()

    def reverse(self):
        print("[Stepper] Direction: REVERSE")
        self.dir.off()
        self._start_loop()

    def stop(self):
        print("[Stepper] STOP")
        self._running = False
        if self._thread:
            self._thread.join()
        self._thread = None

    def set_speed(self, steps_per_sec):
        with self._lock:
            self._speed = max(1, steps_per_sec)
            print(f"[Stepper] Speed set to {self._speed} steps/sec")

    def ramp_to_speed(self, target_speed, ramp_time=1.0):
        print(f"[Stepper] Ramping to {target_speed} steps/sec over {ramp_time}s")
        steps = 20
        interval = ramp_time / steps
        for i in range(1, steps + 1):
            speed = int((target_speed / steps) * i)
            self.set_speed(speed)
            time.sleep(interval)

    def cleanup(self):
        self.stop()
        self.pul.close()
        self.dir.close()
        print("[Stepper] Cleanup complete")

