#!/usr/bin/env python
"""
torque_control.py - load / torque control for Feetech STS and Waveshare ST servos (e.g. ST3215)

Builds on the sms_sts SDK you already have. It adds what that SDK leaves out
(reading load and current, setting the live torque limit, PWM mode, safe mode
switching) and four demos:

    monitor   print position / speed / load / current (use it to calibrate)
    push      position mode + torque limit: drive toward a target with at most X % torque
    pwm       open-loop PWM mode: apply a fixed duty cycle (~ fixed torque while stalled)
    hold      closed loop: change speed until the measured current (torque) hits a target

Run from this folder, with the venv active, e.g. in a terminal (macOS: ls /dev/cu.* shows the port; it is auto-detected if you omit --port):

    python torque_control.py monitor --id 1
    python torque_control.py push    --id 1 --limit 300 --steps 800
    python torque_control.py pwm     --id 1 --duty 150
    python torque_control.py hold    --id 1 --ma 500 --log run.csv

Positive / negative values pick the direction. Ctrl+C stops any demo, and the
servo is put back in position mode holding where it is (or use --on-exit release).
"""

import argparse
import csv
import glob
import sys
import time
from dataclasses import dataclass

import os
_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _here)                    # the scservo_sdk folder shipped next to this file
sys.path.append(os.path.dirname(_here))      # or one level up, as in the original SDK layout
from scservo_sdk import *  # noqa: E402,F403



def find_port(port=None):
    """Return `port` if given, else the first USB serial device for this OS.
    macOS: /dev/cu.usbmodem* etc. (cu.*, not tty.*). Linux: ttyUSB*/ttyACM*. Windows: COM3."""
    if port:
        return port
    if sys.platform == "darwin":
        patterns = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*"]
    elif sys.platform.startswith("linux"):
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
    else:
        return "COM3"
    for pat in patterns:
        found = sorted(glob.glob(pat))
        if found:
            return found[0]
    raise SystemExit("No USB serial port found; plug in the servo adapter or pass --port")


DEFAULT_PORT = None  # None = auto-detect via find_port(); override with --port

# ---------------------------------------------------------------------------
# Registers sms_sts.py doesn't wrap (or doesn't define at all)
# ---------------------------------------------------------------------------
ADDR_MODE = 33              # EEPROM  0 position, 1 speed (closed loop), 2 PWM (open loop), 3 step
ADDR_HOLDING_TORQUE = 34    # EEPROM  % torque left after overload protection trips (default 20)
ADDR_OC_PROTECT_TIME = 38   # EEPROM  overcurrent time before protection trips, x10 ms (default 200 = 2 s)
ADDR_TORQUE_ENABLE = 40     # SRAM    0 off, 1 on, 2 damping
ADDR_PWM = 44               # SRAM    mode 2 only: duty cycle -1000..1000 (0.1 %), sign in bit 10
ADDR_TORQUE_LIMIT = 48      # SRAM    live torque cap 0..1000 (0.1 % of stall torque); reset from addr 16 at power-on
ADDR_FEEDBACK = 56          # position 56, speed 58, load 60, voltage 62, temp 63, ..., current 69
FEEDBACK_LEN = 15           # bytes 56..70 in one read

MODE_POSITION, MODE_SPEED, MODE_PWM = 0, 1, 2

LOAD_SIGN_BIT = 10
PWM_SIGN_BIT = 10           # if a negative --duty doesn't reverse the motor on your firmware, try 11
CURRENT_MA_PER_COUNT = 6.5
STEPS_PER_REV = 4096


class CommError(IOError):
    pass


class TravelLimit(RuntimeError):
    pass


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Feedback:
    position: int       # steps, 0..4095 per revolution
    speed: int          # steps/s with default settings (signed)
    load: int           # PWM duty the servo is applying, -1000..1000 = -100..100 % (signed)
    voltage: float      # supply volts
    temperature: int    # deg C
    current_ma: float   # motor current magnitude, mA - the best torque signal you have


class TorqueServo(sms_sts):
    """sms_sts plus load/torque helpers. Every method raises CommError on a failed transfer."""

    def __init__(self, port_handler):
        super().__init__(port_handler)
        self._last_status = 0

    # --- plumbing -----------------------------------------------------------
    def _check(self, result, error, what):
        if result != COMM_SUCCESS:
            raise CommError(f"{what}: {self.getTxRxResult(result)}")
        if error != self._last_status:  # only report status-bit changes, not every packet
            self._last_status = error
            if error:
                print(f"\n[servo status] {what}: {self.getRxPacketError(error)} (bits 0x{error:02X})")

    # --- reads --------------------------------------------------------------
    def read_feedback(self, sid):
        """Position, speed, load, voltage, temperature and current in ONE round trip."""
        data, result, error = self.readTxRx(sid, ADDR_FEEDBACK, FEEDBACK_LEN)
        self._check(result, error, "read feedback")
        if len(data) < FEEDBACK_LEN:
            raise CommError("read feedback: short packet")

        def word(i):
            return self.scs_makeword(data[i], data[i + 1])

        return Feedback(
            position=self.scs_tohost(word(0), 15),
            speed=self.scs_tohost(word(2), 15),
            load=self.scs_tohost(word(4), LOAD_SIGN_BIT),
            voltage=data[6] / 10.0,
            temperature=data[7],
            current_ma=abs(self.scs_tohost(word(13), 15)) * CURRENT_MA_PER_COUNT,
        )

    def read_mode(self, sid):
        value, result, error = self.read1ByteTxRx(sid, ADDR_MODE)
        self._check(result, error, "read mode")
        return value

    def read_torque_limit(self, sid):
        value, result, error = self.read2ByteTxRx(sid, ADDR_TORQUE_LIMIT)
        self._check(result, error, "read torque limit")
        return value

    # --- writes -------------------------------------------------------------
    def torque_enable(self, sid, on=True):
        self._check(*self.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1 if on else 0), "torque enable")

    def write_torque_limit(self, sid, limit):
        """0..1000 = 0..100 % of stall torque. RAM only, so it's safe to change every loop."""
        self._check(*self.write2ByteTxRx(sid, ADDR_TORQUE_LIMIT, int(clamp(limit, 0, 1000))), "torque limit")

    def write_pwm(self, sid, duty):
        """Mode 2 only. -1000..1000 = -100..100 % duty cycle."""
        duty = int(round(clamp(duty, -1000, 1000)))
        self._check(*self.write2ByteTxRx(sid, ADDR_PWM, self.scs_toscs(duty, PWM_SIGN_BIT)), "pwm")

    def write_speed(self, sid, speed, acc=0):
        """Mode 1 only. Signed steps/s. acc=0 means no acceleration ramp (fastest response)."""
        self._check(*self.WriteSpec(sid, int(round(speed)), acc), "speed")

    def write_position(self, sid, position, speed=0, acc=0):
        """Mode 0 only."""
        self._check(*self.WritePosEx(sid, int(position), int(speed), int(acc)), "position")

    def set_mode(self, sid, mode):
        """Mode lives in EEPROM, so only write it when it actually changes (written the same way
        as the SDK's WheelMode()). The demos switch back to position mode on exit, so
        read_write.py keeps working afterwards."""
        if self.read_mode(sid) != mode:
            self._check(*self.write1ByteTxRx(sid, ADDR_MODE, mode), "set mode")
            time.sleep(0.02)

    def enter_mode(self, sid, mode):
        """Switch modes without a lurch and without dropping torque (which would let a loaded
        cable unwind). The goal registers keep stale values across modes - WriteSpec leaves goal
        position = 0, for example, so switching to position mode blindly would swing the servo
        to 0. So: first write a neutral command for the NEW mode, which is harmless in the old
        one, then switch."""
        if mode == MODE_SPEED:
            # only the speed register: WriteSpec would also write goal position 0, and in
            # position mode that would send the servo straight to 0
            self._check(*self.write2ByteTxRx(sid, SMS_STS_GOAL_SPEED_L, 0), "zero speed")
        elif mode == MODE_PWM:
            self.write_pwm(sid, 0)
        else:
            self.write_position(sid, self.read_feedback(sid).position)  # also zeroes speed and PWM
        self.set_mode(sid, mode)
        self.torque_enable(sid, True)


class CurrentRegulator:
    """'Go whatever speed it takes to reach this torque.'

    Measured current (a torque proxy) is low-pass filtered and compared with the target; the
    error, minus a deadband, times kp becomes the speed command for mode 1. Because the servo's
    own speed loop integrates speed into position, this P-on-speed loop already behaves like an
    integral controller on torque, so no I term is needed against a cable or a hard stop.

    Built for loads that push back in one direction (a cable being wound in, a finger pressing
    on something): `direction` (+1 / -1) is the way the motor turns to INCREASE the load, and the
    current magnitude is used, so it doesn't depend on the sign convention of the load register.
    """

    def __init__(self, target_ma, direction=+1, kp=1.0, deadband_ma=30.0, vmax=400.0, alpha=0.3):
        self.target_ma = abs(target_ma)
        self.direction = 1 if direction >= 0 else -1
        self.kp = kp                    # (steps/s) per mA of error
        self.deadband_ma = deadband_ma  # no motion inside +/- this error
        self.vmax = vmax                # steps/s cap in either direction
        self.alpha = alpha              # low-pass factor per sample, 0..1 (1 = no filtering)
        self.current_f = None
        self.correcting = False

    def update(self, fb):
        i = fb.current_ma
        self.current_f = i if self.current_f is None else self.current_f + self.alpha * (i - self.current_f)
        err = self.target_ma - self.current_f   # > 0: not pulling hard enough
        # Hysteresis: start correcting when the error leaves the deadband, but keep going until
        # it's within a quarter of it. That ignores noise while holding, without settling at
        # the edge of the band.
        if self.correcting and abs(err) < 0.25 * self.deadband_ma:
            self.correcting = False
        elif not self.correcting and abs(err) > self.deadband_ma:
            self.correcting = True
        if not self.correcting:
            return 0.0
        return self.direction * clamp(self.kp * err, -self.vmax, self.vmax)


# ---------------------------------------------------------------------------
# Demo helpers
# ---------------------------------------------------------------------------
class CsvLog:
    def __init__(self, path):
        self.file = open(path, "w", newline="") if path else None
        if self.file:
            self.writer = csv.writer(self.file)
            self.writer.writerow(["t_s", "position", "speed", "load_pct", "current_ma",
                                  "voltage", "temp_c", "command"])

    def row(self, t, fb, command=""):
        if self.file:
            self.writer.writerow([f"{t:.4f}", fb.position, fb.speed, fb.load / 10.0,
                                  round(fb.current_ma, 1), fb.voltage, fb.temperature, command])

    def close(self):
        if self.file:
            self.file.close()


def describe(fb, extra=""):
    return (f"pos {fb.position:5d}  spd {fb.speed:6d}  load {fb.load / 10:6.1f} %  "
            f"I {fb.current_ma:6.0f} mA  {fb.voltage:4.1f} V  {fb.temperature:3d} C  {extra}")


def run_loop(step, seconds=0.0, rate_hz=100.0, print_hz=5.0):
    """Call step(t) -> status string at up to rate_hz, for `seconds` (0 = until Ctrl+C)."""
    period = 1.0 / rate_hz
    t0 = time.perf_counter()
    next_print = 0.0
    n = misses = 0
    try:
        while seconds <= 0 or time.perf_counter() - t0 < seconds:
            start = time.perf_counter()
            t = start - t0
            try:
                status = step(t)
                misses = 0
            except CommError as exc:
                misses += 1
                status = None
                if misses >= 10:
                    raise CommError(f"10 failed transfers in a row, last: {exc}")
            n += 1
            if status and t >= next_print:
                next_print = t + 1.0 / print_hz
                print(status)
            leftover = period - (time.perf_counter() - start)
            if leftover > 0:
                time.sleep(leftover)
    except KeyboardInterrupt:
        pass
    elapsed = max(time.perf_counter() - t0, 1e-9)
    print(f"\n{n} loop iterations in {elapsed:.1f} s = {n / elapsed:.0f} Hz")


# ---------------------------------------------------------------------------
# Demos
# ---------------------------------------------------------------------------
def cmd_monitor(servo, sid, args, log):
    """Watch feedback. --hold makes the servo hold its current position, so you can hang
    known weights on it and write down current vs. torque for calibration."""
    if args.hold:
        servo.enter_mode(sid, MODE_POSITION)

    def step(t):
        fb = servo.read_feedback(sid)
        log.row(t, fb)
        return describe(fb)

    run_loop(step, args.seconds, args.rate)


def cmd_push(servo, sid, args, log):
    """Option A: torque-limited position move. The servo drives toward a target it can't reach,
    and the torque limit caps how hard it pushes once it's stopped."""
    servo.enter_mode(sid, MODE_POSITION)
    start = servo.read_feedback(sid).position
    goal = clamp(start + args.steps, 0, STEPS_PER_REV - 1)
    if goal != start + args.steps:
        print(f"note: goal clamped to {goal} (single-turn range is 0..4095)")
    servo.write_torque_limit(sid, args.limit)
    servo.write_position(sid, goal, args.speed, 0)
    print(f"pushing from {start} toward {goal} with torque limit {args.limit / 10:.1f} %")

    def step(t):
        fb = servo.read_feedback(sid)
        log.row(t, fb, goal)
        return describe(fb, f"goal {goal}")

    run_loop(step, args.seconds, args.rate)


def cmd_pwm(servo, sid, args, log):
    """Option B: open-loop PWM. While stalled, torque is roughly duty x stall torque;
    once the shaft moves, back-EMF eats into it and torque drops."""
    servo.enter_mode(sid, MODE_PWM)
    servo.write_pwm(sid, args.duty)
    print(f"PWM duty {args.duty / 10:.1f} %")

    def step(t):
        fb = servo.read_feedback(sid)
        log.row(t, fb, args.duty)
        return describe(fb, f"duty {args.duty / 10:.1f} %")

    try:
        run_loop(step, args.seconds, args.rate)
    finally:
        servo.write_pwm(sid, 0)


def cmd_hold(servo, sid, args, log):
    """Option C: closed loop on measured current using speed mode."""
    reg = CurrentRegulator(args.ma, direction=1 if args.ma >= 0 else -1, kp=args.kp,
                           deadband_ma=args.deadband, vmax=args.vmax, alpha=args.filter)
    servo.write_torque_limit(sid, args.limit)   # hard safety cap while the loop runs
    servo.enter_mode(sid, MODE_SPEED)
    print(f"regulating to {abs(args.ma):.0f} mA, turning {'+' if reg.direction > 0 else '-'} to increase load; "
          f"torque cap {args.limit / 10:.1f} %, travel cap {args.max_travel} steps")

    last_pos = None
    travel = 0

    def step(t):
        nonlocal last_pos, travel
        fb = servo.read_feedback(sid)
        if last_pos is not None:            # unwrap so travel can be tracked across 4095 -> 0
            d = fb.position - last_pos
            if d > STEPS_PER_REV // 2:
                d -= STEPS_PER_REV
            elif d < -STEPS_PER_REV // 2:
                d += STEPS_PER_REV
            travel += d
        last_pos = fb.position
        if abs(travel) > args.max_travel:
            servo.write_speed(sid, 0)
            raise TravelLimit(f"moved {travel} steps without reaching the target - slack cable or "
                              f"wrong direction? (raise --max-travel if this is expected)")
        speed = reg.update(fb)
        servo.write_speed(sid, speed)
        log.row(t, fb, round(speed, 1))
        return describe(fb, f"I_filt {reg.current_f:6.0f} / {reg.target_ma:.0f} mA  cmd {speed:7.1f}")

    try:
        run_loop(step, args.seconds, args.rate)
    finally:
        servo.write_speed(sid, 0)


# ---------------------------------------------------------------------------
def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default=DEFAULT_PORT, help="serial port, e.g. /dev/cu.usbmodem1101 (default: auto-detect)")
    common.add_argument("--baud", type=int, default=1000000)
    common.add_argument("--id", type=int, default=1, help="servo ID (default 1)")
    common.add_argument("--seconds", type=float, default=0, help="run time, 0 = until Ctrl+C")
    common.add_argument("--rate", type=float, default=100, help="loop rate cap in Hz (default 100)")
    common.add_argument("--log", help="write every sample to this CSV file")
    common.add_argument("--on-exit", choices=["hold", "release"], default="hold",
                        help="hold = position mode at the current spot (default); release = torque off")

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("monitor", parents=[common], help="print feedback")
    m.add_argument("--hold", action="store_true", help="hold the current position while monitoring")

    pu = sub.add_parser("push", parents=[common], help="torque-limited position move (option A)")
    pu.add_argument("--limit", type=int, default=300, help="torque limit 0..1000 = 0..100 %% (default 300)")
    pu.add_argument("--steps", type=int, default=800, help="how far past the start to aim, signed (default 800)")
    pu.add_argument("--speed", type=int, default=500, help="approach speed, steps/s (default 500)")

    pw = sub.add_parser("pwm", parents=[common], help="open-loop duty cycle (option B)")
    pw.add_argument("--duty", type=int, required=True, help="-1000..1000 = -100..100 %%")

    h = sub.add_parser("hold", parents=[common], help="closed-loop current regulation (option C)")
    h.add_argument("--ma", type=float, required=True, help="target current in mA; sign = direction that increases load")
    h.add_argument("--kp", type=float, default=1.0,
                   help="steps/s of speed per mA of error (default 1.0): raise if it creeps, lower if it buzzes")
    h.add_argument("--deadband", type=float, default=30, help="mA of error to ignore (default 30)")
    h.add_argument("--vmax", type=float, default=400, help="max speed command, steps/s (default 400)")
    h.add_argument("--filter", type=float, default=0.3, help="low-pass factor 0..1, lower = smoother (default 0.3)")
    h.add_argument("--limit", type=int, default=600, help="torque safety cap 0..1000 (default 600 = 60 %%)")
    h.add_argument("--max-travel", type=int, default=3 * STEPS_PER_REV,
                   help="stop if it turns this far without reaching the target (default 12288 = 3 turns)")
    return p


COMMANDS = {"monitor": cmd_monitor, "push": cmd_push, "pwm": cmd_pwm, "hold": cmd_hold}


def connect(port, baud):
    port = find_port(port)
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        sys.exit(f"Could not open {port}")
    if not port_handler.setBaudRate(baud):
        sys.exit(f"Could not set baud rate {baud}")
    return port_handler, TorqueServo(port_handler)


def run(args, port_handler, servo):
    sid = args.id
    log = CsvLog(args.log)
    original_limit = servo.read_torque_limit(sid)
    try:
        COMMANDS[args.cmd](servo, sid, args, log)
    except (CommError, TravelLimit) as exc:
        print(f"\nstopped: {exc}")
    finally:
        log.close()
        if args.cmd == "monitor" and not args.hold:
            return  # we only watched; leave the servo exactly as we found it
        try:
            if args.on_exit == "release":
                servo.torque_enable(sid, False)
                servo.set_mode(sid, MODE_POSITION)
                servo.write_position(sid, servo.read_feedback(sid).position)  # no stale goal left behind
                servo.torque_enable(sid, False)
                state = "torque off"
            else:
                servo.enter_mode(sid, MODE_POSITION)   # hold here first...
                state = "holding its current position"
            servo.write_torque_limit(sid, original_limit)  # ...then it's safe to restore full torque
            print(f"servo back in position mode, {state}; torque limit restored to {original_limit / 10:.1f} %")
        except CommError as exc:
            print(f"could not reset servo cleanly: {exc}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    port_handler, servo = connect(args.port, args.baud)
    try:
        run(args, port_handler, servo)
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    main()
