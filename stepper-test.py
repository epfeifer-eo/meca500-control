# -*- coding: utf-8 -*-
"""
Created on Thu May 29 13:17:00 2025

@author: elija
"""
import time
from stepper import Stepper

stepper = Stepper()

try:
    print("[Test] Running forward for 2 seconds at 600 steps/sec")
    stepper.forward(speed=600)
    time.sleep(2)

    print("[Test] Reversing for 2 seconds at 800 steps/sec")
    stepper.reverse(speed=800)
    time.sleep(2)

    print("[Test] Stopping")
    stepper.stop()

finally:
    print("[Test] Cleaning up GPIO")
    stepper.cleanup()



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
