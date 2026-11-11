"""
example_controller.py - starting point for a tendon controller. Edit freely.

Running it PULLS ALL FOUR TENDONS at the forces below until you press Ctrl+C.

Tendon IDs:  1 = tendon A   2 = tendon B   3 = tendon C   4 = central tendon
"""
import time

from multi_torque import MOTORS, PORT, TorqueBus

with TorqueBus(PORT, MOTORS, command_timeout=0.5) as bus:
    bus.start()                    # all four limp
    bus.run_in_background(100)     # keeps each tendon at its target

    try:
        while True:
            # --- your control logic: decide a force for each tendon ---
            # bus.set_force_n(1, 10.0)   # tendon A
            bus.set_force_n(2, 0.0)    # tendon B
            # bus.set_force_n(3, 6.0)    # tendon C

            status = bus.status()      # feedback, e.g. status[1]["force_n"], status[1]["position"]
            bus.raise_if_error()       # stops your loop if the motors' loop died
            time.sleep(0.02)           # ~50 Hz
    except KeyboardInterrupt:
        print("stopping")
# leaving the with-block stops all motors: pulling ones hold where they are
