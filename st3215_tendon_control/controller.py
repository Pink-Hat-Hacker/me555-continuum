"""
controller.py - three-tendon (A, B, C) controller for a SpiRobs-style continuum arm.

    python controller.py --dry-run                      keyboard control, logs instead of moving motors
    python controller.py                                keyboard control on the real servos
    python controller.py --forces 10 10 -5              hold A=10 N, B=10 N, C released, until Ctrl+C
    python controller.py --dry-run --forces 10 10 -5    same, logged only
    python -m unittest test_controller                  quick tests (no hardware)

Every input method ends up in Controller.set_forces([fa, fb, fc]):
    positive = pull that many newtons, zero or negative = release (limp, or RELEASE_FORCE_N).
Targets are clamped to [FORCE_MIN, FORCE_MAX] and the forces actually sent move toward the
targets by at most MAX_DELTA_PER_TICK per tick, so the arm never jerks.
"""
import argparse
import time

# ---------------------------------------------------------------------------
# Configuration - edit these
# ---------------------------------------------------------------------------
PORT = "COM3"
BAUDRATE = 1000000
SERVO_IDS = [1, 2, 3]          # tendon A, B, C (must exist in multi_torque.MOTORS)

FORCE_MIN = -5.0               # N; anything <= 0 is sent as a release
FORCE_MAX = 15.0               # N; hard ceiling on any tendon
RELEASE_FORCE_N = 0.0          # N sent for a released tendon (0 = limp; a small pull pays out better)
KEY_STEP_N = 0.5               # N added/removed per key press
MAX_DELTA_PER_TICK = 0.25      # N per tick; with TICK_S = 0.02 that is 12.5 N/s
TICK_S = 0.02                  # ~50 Hz; also keeps the bus watchdog fed
COMMAND_TIMEOUT_S = 0.5        # bus releases everything if no command arrives for this long

# key -> (tendon index, direction); index None = all three tendons. One place to remap keys.
KEYMAP = {
    "KEY_UP":    (0, +1), "KEY_DOWN": (0, -1),   # tendon A
    "KEY_RIGHT": (1, +1), "KEY_LEFT":  (1, -1),   # tendon B
    "w":         (2, +1), "s":         (2, -1),   # tendon C
    "d":      (None, +1), "a":      (None, -1),   # all tendons together
}
RELEASE_KEYS = (" ", "\x1b")   # space, Esc
QUIT_KEY = "q"
NAMES = ["A", "B", "C"]


# ---------------------------------------------------------------------------
# Core
# ---------------------------------------------------------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def validate_forces(forces):
    """Return forces as a list of 3 floats, or raise ValueError/TypeError."""
    try:
        values = list(forces)
    except TypeError:
        raise TypeError("forces must be a sequence of 3 numbers") from None
    if len(values) != 3:
        raise ValueError(f"need exactly 3 forces [fa, fb, fc], got {len(values)}")
    out = []
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v:
            raise TypeError(f"forces must be numbers, got {v!r}")
        out.append(float(v))
    return out


def step_toward(current, target, max_delta):
    return current + clamp(target - current, -max_delta, max_delta)


class MockBus:
    """Stands in for TorqueBus: logs what it would send, touches no hardware."""

    def __init__(self, ids):
        self.ids = list(ids)

    def __enter__(self):
        print(f"[dry-run] open bus, servo IDs {self.ids}")
        return self

    def __exit__(self, *exc):
        print("[dry-run] close bus")
        return False

    def start(self):
        print("[dry-run] start: all limp")

    def run_in_background(self, rate_hz=100):
        pass

    def set_force_n(self, sid, newtons):
        print(f"[dry-run] servo {sid}: pull {newtons:.2f} N")

    def release_all(self):
        print("[dry-run] release all")

    def raise_if_error(self):
        pass


def make_bus(dry_run):
    if dry_run:
        return MockBus(SERVO_IDS)
    from multi_torque import MOTORS, TorqueBus   # imported here so --dry-run needs no serial setup
    motors = [m for m in MOTORS if m.id in SERVO_IDS]
    motors.sort(key=lambda m: SERVO_IDS.index(m.id))
    if len(motors) != 3:
        raise ValueError(f"SERVO_IDS {SERVO_IDS} not all found in multi_torque.MOTORS")
    return TorqueBus(PORT, motors, BAUDRATE, command_timeout=COMMAND_TIMEOUT_S,
                     watchdog_action="release", on_exit="release")


class Controller:
    """Holds targets, ramps the sent forces toward them, and talks to the bus."""

    def __init__(self, bus, ids=SERVO_IDS, fmin=FORCE_MIN, fmax=FORCE_MAX,
                 max_delta=MAX_DELTA_PER_TICK):
        self.bus, self.ids = bus, list(ids)
        self.fmin, self.fmax, self.max_delta = fmin, fmax, max_delta
        self.targets = [0.0, 0.0, 0.0]
        self.sent = [0.0, 0.0, 0.0]

    def set_forces(self, forces):
        """The one entry point: [fa, fb, fc], positive = pull, negative = release."""
        values = validate_forces(forces)
        self.targets = [clamp(v, self.fmin, self.fmax) for v in values]

    def tick(self):
        """Move each sent force one rate-limited step toward its target and command the bus."""
        self.sent = [step_toward(s, t, self.max_delta) for s, t in zip(self.sent, self.targets)]
        for sid, f in zip(self.ids, self.sent):
            self.bus.set_force_n(sid, f if f > 0 else RELEASE_FORCE_N)
        self.bus.raise_if_error()

    def release_all(self):
        self.targets = [0.0, 0.0, 0.0]
        self.sent = [0.0, 0.0, 0.0]
        self.bus.release_all()


# ---------------------------------------------------------------------------
# Input sources: poll(targets) -> new [fa, fb, fc] or None for "no change"; return the
# string "quit" to stop. A joystick source (axes -> 3 forces) only has to implement this.
# ---------------------------------------------------------------------------
class InputSource:
    def poll(self, targets):
        raise NotImplementedError

    def close(self):
        pass


class ArrayInput(InputSource):
    """Fixed forces, e.g. ArrayInput([10, 10, -5]). Sent once."""

    def __init__(self, forces):
        self.forces = validate_forces(forces)
        self._sent = False

    def poll(self, targets):
        if self._sent:
            return None
        self._sent = True
        return self.forces


class KeyboardInput(InputSource):
    """Reads keys from a curses window and adjusts the targets in KEY_STEP_N steps."""

    def __init__(self, stdscr, keymap=KEYMAP, step=KEY_STEP_N):
        self.scr, self.keymap, self.step = stdscr, keymap, step

    def poll(self, targets):
        new, changed = list(targets), False
        while True:
            try:
                key = self.scr.get_wch()
            except Exception:           # curses raises when no key is waiting
                break
            key = key.lower() if isinstance(key, str) else key
            if isinstance(key, int):    # special keys come back as ints
                import curses
                key = curses.keyname(key).decode()
            if key == QUIT_KEY:
                return "quit"
            if key in RELEASE_KEYS:
                new, changed = [0.0, 0.0, 0.0], True
            elif key in self.keymap:
                idx, sign = self.keymap[key]
                for i in ([0, 1, 2] if idx is None else [idx]):
                    new[i] += sign * self.step
                changed = True
        return new if changed else None


# ---------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------
def run_loop(ctrl, source, draw=None):
    """Poll the input, ramp, send - until the source says quit or Ctrl+C. Always releases."""
    try:
        while True:
            result = source.poll(ctrl.targets)
            if result == "quit":
                break
            if result is not None:
                ctrl.set_forces(result)
            ctrl.tick()
            if draw:
                draw(ctrl)
            time.sleep(TICK_S)
    except KeyboardInterrupt:
        print("stopping")
    finally:
        ctrl.release_all()


def keyboard_session(bus):
    import curses

    def main(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        ctrl = Controller(bus)

        def draw(c):
            stdscr.erase()
            stdscr.addstr(0, 0, "Tendon keyboard control")
            for i, line in enumerate([
                "Up/Down    tendon A  +/-",
                "Right/Left tendon B  +/-",
                "W/S        tendon C  +/-",
                "D/A        all tendons +/-",
                "Space/Esc  release all     Q  quit",
            ]):
                stdscr.addstr(2 + i, 0, line)
            for i, n in enumerate(NAMES):
                stdscr.addstr(8 + i, 0, f"{n}: target {c.targets[i]:6.2f} N   sent {c.sent[i]:6.2f} N")
            stdscr.refresh()

        run_loop(ctrl, KeyboardInput(stdscr), draw)

    curses.wrapper(main)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="log commands instead of moving motors")
    ap.add_argument("--forces", nargs=3, type=float, metavar=("FA", "FB", "FC"),
                    help="hold these forces (N) instead of using the keyboard")
    args = ap.parse_args(argv)

    with make_bus(args.dry_run) as bus:
        bus.start()                                    # all limp
        bus.run_in_background(100)
        try:
            if args.forces:
                ctrl = Controller(bus)
                run_loop(ctrl, ArrayInput(args.forces))
            else:
                keyboard_session(bus)
        finally:
            try:
                bus.release_all()
            except Exception as exc:
                print(f"release failed: {exc}")
    # leaving the with-block stops all motors (on_exit="release": they go limp)


if __name__ == "__main__":
    main()
