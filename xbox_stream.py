#!/usr/bin/env python3
"""
xbox_stream.py - stream the left stick and right trigger of an Xbox One controller.

Each sample is a four-value array, every value from 0 to 1:

    [axis_1, axis_2, axis_3, right_trigger]

    axis_1..axis_3  How far the left stick points along each of three directions
                    spaced 120 degrees apart (default: up, lower-left, lower-right).
    right_trigger   How far the right trigger is pressed.

Run it:      python xbox_stream.py              (prints arrays until Ctrl+C)
Import it:   from xbox_stream import XboxStream (call .read() in your own loop)

Requires pygame-ce (pip install pygame-ce) or pygame 2.x.
"""

import os

# Read the controller even when no window has focus (we never open one), and
# hide pygame's import banner so stdout carries only data.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import argparse
import math
import sys
import time

import pygame
from pygame._sdl2 import controller as sdl_controller

# SDL's game-controller API gives every supported pad the same layout, so the
# Xbox One left stick and right trigger are found the same way on every OS.
LEFT_X = pygame.CONTROLLER_AXIS_LEFTX
LEFT_Y = pygame.CONTROLLER_AXIS_LEFTY
RIGHT_TRIGGER = pygame.CONTROLLER_AXIS_TRIGGERRIGHT
AXIS_MAX = 32767.0  # sticks report -32768..32767, triggers 0..32767

# Direction of each of the three axes, in degrees counterclockwise from
# "stick pushed right" (unit-circle convention): up, lower-left, lower-right.
DEFAULT_ANGLES = (90.0, 210.0, 330.0)


def _clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadzone(value, deadzone):
    """Zero anything below the dead zone and rescale the rest back to 0-1."""
    if value <= deadzone:
        return 0.0
    return _clamp((value - deadzone) / (1.0 - deadzone), 0.0, 1.0)


def stick_to_triaxial(x, y, angles_deg=DEFAULT_ANGLES, deadzone=0.10):
    """
    Convert a stick position into one 0-1 value per axis direction.

    x, y       Stick position, each -1..1, with +y meaning "pushed up".
    angles_deg Direction of each axis in degrees (counterclockwise from right).
    deadzone   Fraction of full deflection treated as "centered".

    Each value is the part of the stick's push that lies along that axis
    (the projection onto the axis), with negative results set to 0. A full
    push straight along an axis gives 1 on that axis and 0 on the others.
    """
    magnitude = math.hypot(x, y)
    strength = apply_deadzone(min(magnitude, 1.0), deadzone)
    if strength == 0.0:
        return [0.0] * len(angles_deg)

    ux, uy = x / magnitude, y / magnitude  # direction the stick points
    values = []
    for angle in angles_deg:
        a = math.radians(angle)
        alignment = ux * math.cos(a) + uy * math.sin(a)  # cos of angle between them
        values.append(max(0.0, strength * alignment))
    return values


class XboxStream:
    """Reads the controller on demand. Reconnects automatically if it drops."""

    def __init__(self, angles_deg=DEFAULT_ANGLES, stick_deadzone=0.10,
                 trigger_deadzone=0.02):
        self.angles_deg = tuple(angles_deg)
        self.stick_deadzone = stick_deadzone
        self.trigger_deadzone = trigger_deadzone
        self._pad = None

        pygame.display.init()   # starts SDL's event system (no window is opened)
        sdl_controller.init()

    @property
    def name(self):
        """Name SDL reports for the connected controller, or None."""
        if self._pad is None:
            return None
        name = getattr(self._pad, "name", "controller")
        return name() if callable(name) else name

    def _is_attached(self):
        try:
            return self._pad is not None and self._pad.attached()
        except Exception:
            return False

    def _connect(self):
        for index in range(sdl_controller.get_count()):
            if sdl_controller.is_controller(index):
                self._pad = sdl_controller.Controller(index)
                return True
        return False

    def read(self):
        """
        Return [axis_1, axis_2, axis_3, right_trigger] (each 0-1),
        or None while no controller is connected.
        """
        pygame.event.get()  # let SDL refresh controller state; discard the events

        if not self._is_attached():
            self._pad = None
            if not self._connect():
                return None

        x = _clamp(self._pad.get_axis(LEFT_X) / AXIS_MAX, -1.0, 1.0)
        y = _clamp(-self._pad.get_axis(LEFT_Y) / AXIS_MAX, -1.0, 1.0)  # SDL: up is negative
        trigger = _clamp(self._pad.get_axis(RIGHT_TRIGGER) / AXIS_MAX, 0.0, 1.0)

        stick = stick_to_triaxial(x, y, self.angles_deg, self.stick_deadzone)
        return stick + [apply_deadzone(trigger, self.trigger_deadzone)]

    def close(self):
        if self._pad is not None:
            try:
                self._pad.quit()
            except Exception:
                pass
            self._pad = None
        sdl_controller.quit()
        pygame.quit()


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Stream the Xbox One left stick (as three 120-degree axes) and "
                    "right trigger as [axis_1, axis_2, axis_3, right_trigger].")
    parser.add_argument("--rate", type=float, default=50.0,
                        help="samples per second (default: 50)")
    parser.add_argument("--deadzone", type=float, default=0.10,
                        help="left-stick dead zone, 0-1 (default: 0.10)")
    parser.add_argument("--trigger-deadzone", type=float, default=0.02,
                        help="right-trigger dead zone, 0-1 (default: 0.02)")
    parser.add_argument("--angles", type=float, nargs=3, default=list(DEFAULT_ANGLES),
                        metavar=("A1", "A2", "A3"),
                        help="direction of each axis in degrees, counterclockwise "
                             "from right (default: 90 210 330)")
    parser.add_argument("--decimals", type=int, default=3,
                        help="decimal places to print (default: 3)")
    args = parser.parse_args(argv)

    if args.rate <= 0:
        parser.error("--rate must be greater than 0")
    for flag in ("deadzone", "trigger_deadzone"):
        if not 0.0 <= getattr(args, flag) < 1.0:
            parser.error(f"--{flag.replace('_', '-')} must be at least 0 and less than 1")
    return args


def main(argv=None):
    args = _parse_args(argv)
    stream = XboxStream(args.angles, args.deadzone, args.trigger_deadzone)
    period = 1.0 / args.rate
    fmt = f"{{:.{args.decimals}f}}"

    # Status messages go to stderr so stdout holds only the data arrays.
    print("Waiting for an Xbox controller... (Ctrl+C to quit)", file=sys.stderr)
    waiting = True
    next_tick = time.perf_counter()
    try:
        while True:
            values = stream.read()
            if values is None:
                if not waiting:
                    print("Controller disconnected. Waiting...", file=sys.stderr)
                    waiting = True
            else:
                if waiting:
                    print(f"Connected: {stream.name}", file=sys.stderr)
                    waiting = False
                print("[" + ", ".join(fmt.format(v) for v in values) + "]", flush=True)

            next_tick += period
            delay = next_tick - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.perf_counter()  # fell behind; don't try to catch up
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()


if __name__ == "__main__":
    main()
