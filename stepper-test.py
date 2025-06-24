# -*- coding: utf-8 -*-
"""
Created on Thu May 29 13:17:00 2025

@author: elija
"""
# stepper-test.py
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


# Pin assignments
PUL_PIN = 17
DIR_PIN = 27

# Parameters
DIRECTION = "forward"  # "forward" or "reverse"
STEPS_PER_SEC = 600
RUN_DURATION_SEC = 3

# Setup GPIO pins
pulse = DigitalOutputDevice(PUL_PIN)
direction = DigitalOutputDevice(DIR_PIN)

# Set direction
if DIRECTION == "forward":
    direction.on()
else:
    direction.off()

# Calculate pulse delay
delay = 1.0 / STEPS_PER_SEC
print(f"[Test] Direction: {DIRECTION}, Speed: {STEPS_PER_SEC} steps/sec, Duration: {RUN_DURATION_SEC}s")

# Run motor
try:
    start_time = time.time()
    while time.time() - start_time < RUN_DURATION_SEC:
        pulse.on()
        time.sleep(delay / 2)
        pulse.off()
        time.sleep(delay / 2)

    print("[Test] Stepper run complete.")

finally:
    # Cleanup
    pulse.off()
    direction.off()
    pulse.close()
    direction.close()
    print("[Test] GPIO cleaned up.")



# from stepper import Stepper
# import time

# stepper = Stepper()

# # Forward with ramp-up
# stepper.forward()
# #stepper.ramp_to_speed(800, ramp_time=1.0)
# time.sleep(2)

# # Stop 
# stepper.stop()
# time.sleep(1)

# # Reverse with slow ramp-up
# stepper.reverse()
# #stepper.ramp_to_speed(800, ramp_time=1.0)
# time.sleep(2)

# # Stop
# stepper.stop()

# stepper.cleanup()



# def main():
#     motor = Stepper(pul_pin=17, dir_pin=27)

#     try:
#         motor.set_speed(500)  # 500 steps/sec
#         print("[Test] Moving forward for 2 seconds")
#         motor.forward()
#         time.sleep(2)

#         motor.stop()
#         time.sleep(1)

#         print("[Test] Moving in reverse for 2 seconds")
#         motor.reverse()
#         time.sleep(2)

#         motor.stop()

#     except KeyboardInterrupt:
#         print("\n[Test] Interrupted by user")

#     finally:
#         motor.cleanup()
#         print("[Test] Done.")

# if __name__ == "__main__":
#     main()



# from gpiozero import OutputDevice
# from time import sleep

# step = OutputDevice(17)
# direction = OutputDevice(27)

# direction.on()  # or .off() for other direction

# for delay in [0.01, 0.005, 0.002, 0.001]:
#     print(f"Running at delay: {delay}s")
#     for _ in range(50):
#         step.on()
#         sleep(delay)
#         step.off()
#         sleep(delay)
