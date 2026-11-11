#!/usr/bin/env python
"""
multi_torque.py - independent tension control of several daisy-chained ST3215 servos,
written for tendon-driven robots.

Running this file does NOTHING to the servos unless you pass one of the options below.
Import TorqueBus into your controller code and give each tendon a pull.

    import time
    from multi_torque import TorqueBus, PORT, MOTORS

    with TorqueBus(PORT, MOTORS) as bus:        # opens the port - nothing moves
        bus.start()                             # all motors limp (torque off)
        bus.run_in_background(rate_hz=100)      # control loop runs in its own thread
        while True:                             # <- your controller loop goes here
            bus.set_force_n(1, 10.0)            # pull tendon A with ~10 N
            bus.set_force_n(4, 6.0)             # central tendon ~6 N
            bus.set_force_n(2, 0)               # 0 = limp: motor torque off
            bus.hold(3)                         # freeze tendon C where it is
            bus.raise_if_error()                # surface any problem from the loop thread
            time.sleep(0.02)
    # Leaving the with-block (normally, by Ctrl+C or by an exception) stops everything; motors
    # that were pulling are left holding in position mode, limp ones stay limp.

What each command does to one motor:
    set_torque(id, x > 0)   regulate: winds in while the pull is below x, PAYS OUT while it's
                            above x (so the arm can pull the tendon out), stays put in between
    set_torque(id, 0)       limp: motor torque off. The 1:345 gearbox still resists being
                            pulled, so for a tendon that should give way easily a small pull
                            (say 100-150 mA) works better than 0, because it pays out actively
    hold(id)                freeze: torque on, locked where it is, will NOT pay out

How the pull is measured: motor current (6.5 mA steps), signed by the direction the servo is
driving. It's a torque estimate, not a load cell: gear friction makes it read high while
winding in and low while paying out, so the real tension sits in a friction band around the
target. Calibrate with known weights (torque_control.py monitor --hold), or add load cells
if you need accurate force.

Command-line options (all optional; the first two only read):
    python multi_torque.py --check        ping the configured servos and show their feedback
    python multi_torque.py --scan         list every servo ID that answers on the bus
    python multi_torque.py --set-id 1 2   change a servo's ID from 1 to 2 (connect ONLY that servo)

Needs torque_control.py in the same folder.
"""

import argparse
import threading
import time
from dataclasses import dataclass
from typing import Optional

import torque_control as tc   # also brings in the SDK (PortHandler, GroupSyncRead, ...)

# ---------------------------------------------------------------------------
# Configuration - edit these
# ---------------------------------------------------------------------------
PORT = None                  # None = auto-detect (macOS /dev/cu.usbmodem*), or set e.g. "/dev/cu.usbmodem1101"
BAUDRATE = 1000000


@dataclass
class MotorConfig:
    id: int
    name: str = ""
    direction: int = +1          # way the servo turns to PULL its tendon in. If it pays out instead, use -1
    spool_radius_m: Optional[float] = None  # tendon spool radius in metres, for set_force_n() and status
    max_ma: float = 1200.0       # pull targets above this are clamped (12 V ST3215 stalls around 2700 mA)
    torque_limit: int = 600      # servo-side hard cap while running: 0..1000 = 0..100 % of stall torque
    kp: float = 1.0              # (steps/s) of speed per mA of error: raise if sluggish, lower if it buzzes
    deadband_ma: float = 30.0    # error ignored while holding (current is measured in 6.5 mA steps)
    vmax_in: float = 400.0       # fastest it winds in while regulating, steps/s
    vmax_out: float = 800.0      # fastest it pays out when over-tensioned, steps/s (servo max ~3000)
    filter_alpha: float = 0.3    # tension low-pass, 0..1 per sample (lower = smoother but slower)
    max_travel: int = 3 * 4096   # steps it may wind in without reaching the target before faulting (slack guard)
    kt_nm_per_a: float = 1.08    # 12 V ST3215 spec value (11 kg*cm/A), used for N*m and N. Calibrate!
    load_sign: int = +1          # only change if the built-in check says the load sign is inverted


SPOOL_RADIUS_M = 0.01825   # support-free spool (36 mm groove) wound with 0.5 mm line

MOTORS = [
    MotorConfig(id=1, name="tendon A", spool_radius_m=SPOOL_RADIUS_M),
    MotorConfig(id=2, name="tendon B", spool_radius_m=SPOOL_RADIUS_M),
    MotorConfig(id=3, name="tendon C", spool_radius_m=SPOOL_RADIUS_M)
]

ERRBIT_OVERLOAD = 32
SIGN_CHECK_SAMPLES = 25


class TendonRegulator:
    """Turns a measured pull and a target pull into a speed command.

    Pull is motor current, signed by which way the servo is driving: positive = pulling the
    tendon in, negative = the servo pushing it out (overcoming gear friction while paying out).
    The sign matters: with magnitude alone, a small target while paying out would see friction
    current, read it as tension, and keep unwinding into slack.

    speed = kp * (target - pull), so the servo's speed loop integrates the error into position:
    an integral controller on tension, which needs no I term. Hysteresis around the deadband
    keeps it still while holding without settling at the edge of the band.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.tension_ma = None
        self.correcting = False

    def measure(self, fb):
        cfg = self.cfg
        drive = 1 if fb.load * cfg.direction * cfg.load_sign >= 0 else -1
        pull = drive * fb.current_ma
        if self.tension_ma is None:
            self.tension_ma = pull
        else:
            self.tension_ma += cfg.filter_alpha * (pull - self.tension_ma)
        return self.tension_ma

    def reset(self):
        self.correcting = False

    def speed(self, target_ma):
        """Speed command in the servo's own sign convention (steps/s)."""
        cfg = self.cfg
        err = target_ma - self.tension_ma          # > 0: not pulling hard enough
        if self.correcting and abs(err) < 0.25 * cfg.deadband_ma:
            self.correcting = False
        elif not self.correcting and abs(err) > cfg.deadband_ma:
            self.correcting = True
        if not self.correcting:
            return 0.0
        v = cfg.kp * err
        v = min(v, cfg.vmax_in) if v > 0 else max(v, -cfg.vmax_out)
        return cfg.direction * v


class _Motor:
    """Runtime state for one servo."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reg = TendonRegulator(cfg)
        self.request = "release"      # what was asked for: "release", "hold" or "regulate"
        self.target_ma = 0.0
        self.released = True          # actual torque state
        self.feedback = None
        self.status_bits = 0
        self.speed_cmd = 0.0
        self.fault = None
        self.fault_limp = False       # some faults are safer limp than locked
        self.offline = False
        self.misses = 0
        self.travel = 0
        self.last_pos = None
        self.original_limit = None
        self.sign_votes = [0, 0]      # [agree, disagree]
        self.sign_confirmed = False


class TorqueBus:
    """Several servos on one serial bus, each regulated to its own pull.

    Lifecycle: open() (or `with`) -> start() -> run_in_background() (or call update() yourself)
    -> set_torque() / hold() whenever you like -> close(). Nothing moves before start(); start()
    leaves every motor limp; a motor only pulls once it gets a target above 0.
    """

    def __init__(self, port=PORT, motors=MOTORS, baudrate=BAUDRATE, command_timeout=None,
                 watchdog_action="hold", miss_limit=5, on_exit="hold"):
        """
        command_timeout: seconds. If set and no command arrives for that long, every motor does
                         `watchdog_action` - a safety net for when a controller disconnects.
        watchdog_action: "hold" (freeze in place, the default) or "release" (go limp).
        miss_limit:      consecutive missed replies before a servo is marked offline.
        on_exit:         what pulling/holding motors do at close(): "hold" (position mode,
                         holding where they are) or "release". Limp motors always stay limp.
        """
        ids = [m.id for m in motors]
        if len(set(ids)) != len(ids):
            raise ValueError(f"every motor needs its own ID, got {ids}")
        if on_exit not in ("hold", "release") or watchdog_action not in ("hold", "release"):
            raise ValueError("on_exit and watchdog_action must be 'hold' or 'release'")
        self.port_name, self.baudrate = tc.find_port(port), baudrate
        self.motors = {m.id: _Motor(m) for m in motors}
        self.command_timeout, self.watchdog_action = command_timeout, watchdog_action
        self.miss_limit, self.on_exit = miss_limit, on_exit
        self.port = self.servo = None
        self.started = False
        self.use_sync_read = True
        self.error = None
        self.loop_hz = 0.0
        self._lock = threading.Lock()   # requests and targets shared with the loop thread
        self._io = threading.RLock()    # the serial port
        self._thread = None
        self._stop = threading.Event()
        self._last_command = time.monotonic()
        self._watchdog_tripped = False
        self._last_update = None
        self._poll_ids = None

    # ------------------------------------------------------------------ connection
    def open(self):
        """Open the serial port. Sends nothing to the servos."""
        self.port = tc.PortHandler(self.port_name)
        if not self.port.openPort():
            raise tc.CommError(f"could not open {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            raise tc.CommError(f"could not set baud rate {self.baudrate}")
        self.servo = tc.TorqueServo(self.port)
        self._sync_read = tc.GroupSyncRead(self.servo, tc.ADDR_FEEDBACK, tc.FEEDBACK_LEN)
        self._sync_write = tc.GroupSyncWrite(self.servo, tc.SMS_STS_GOAL_SPEED_L, 2)
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    def start(self):
        """Check every servo answers, then set each one up in speed mode with torque OFF.
        Nothing moves; a motor engages when it gets a target above 0 (or hold())."""
        with self._io:
            missing = [sid for sid in self.motors if not self._answers(sid)]
            if missing:
                raise tc.CommError(
                    f"servo ID(s) {missing} not answering. Run `python multi_torque.py --scan` to see "
                    f"which IDs are on the bus (new servos all ship as ID 1, so daisy-chained ones "
                    f"need --set-id first).")
            for sid, m in self.motors.items():
                m.original_limit = self.servo.read_torque_limit(sid)
                self.servo.write_torque_limit(sid, m.cfg.torque_limit)
                self.servo._check(*self.servo.write2ByteTxRx(sid, tc.SMS_STS_GOAL_SPEED_L, 0), "zero speed")
                self.servo.set_mode(sid, tc.MODE_SPEED)
                self.servo.torque_enable(sid, False)
                m.released = True
                m.last_pos = self.servo.read_feedback(sid).position
            self._probe_sync_read()
            self.started = True
            self._last_command = time.monotonic()

    def close(self):
        """Stop the loop and every motor, leave them in position mode (pulling/holding motors
        hold, limp ones stay limp), and restore the original torque limits."""
        self.stop_background()
        with self._io:
            if self.servo is not None and self.started:
                try:
                    self._write_speeds({sid: 0 for sid, m in self.motors.items()
                                        if not m.offline and not m.released})
                except tc.CommError:
                    pass
                for sid, m in self.motors.items():
                    try:
                        if self.on_exit == "release" or m.released:
                            self.servo.torque_enable(sid, False)
                            self.servo.set_mode(sid, tc.MODE_POSITION)
                            self.servo.write_position(sid, self.servo.read_feedback(sid).position)
                            self.servo.torque_enable(sid, False)
                        else:
                            self.servo.enter_mode(sid, tc.MODE_POSITION)
                        if m.original_limit is not None:
                            self.servo.write_torque_limit(sid, m.original_limit)
                    except tc.CommError as exc:
                        print(f"[{self._label(m)}] could not reset cleanly: {exc}")
                self.started = False
            if self.port is not None:
                self.port.closePort()
                self.port = None

    # ------------------------------------------------------------------ commands (thread-safe)
    def set_torque(self, sid, ma):
        """Pull target for one motor in mA of motor current (proportional to torque).
        0 = limp (torque off). Above 0 = regulate: wind in or pay out to hold that pull.
        Values above MotorConfig.max_ma are clamped."""
        m = self._motor(sid)
        if ma < 0:
            raise ValueError("pull targets are >= 0 (a tendon can't push); 0 = limp")
        with self._lock:
            if ma == 0:
                m.request, m.target_ma = "release", 0.0
            else:
                m.request, m.target_ma = "regulate", min(float(ma), m.cfg.max_ma)
            self._touch()

    def set_torque_nm(self, sid, nm):
        """Same as set_torque, in N*m at the servo output (uses MotorConfig.kt_nm_per_a)."""
        self.set_torque(sid, nm / self._motor(sid).cfg.kt_nm_per_a * 1000.0)

    def set_force_n(self, sid, newtons):
        """Same as set_torque, as tendon tension in newtons (uses spool_radius_m and kt_nm_per_a)."""
        r = self._motor(sid).cfg.spool_radius_m
        if not r:
            raise ValueError(f"set MotorConfig(spool_radius_m=...) for motor {sid} to use newtons")
        self.set_torque_nm(sid, newtons * r)

    def set_torques(self, values):
        """Several at once: a dict {id: mA} or a list in the same order as the motors."""
        items = values.items() if isinstance(values, dict) else zip(self.motors, values)
        for sid, ma in items:
            self.set_torque(sid, ma)

    def hold(self, sid):
        """Freeze one motor where it is: torque on, won't wind in or pay out."""
        m = self._motor(sid)
        with self._lock:
            m.request, m.target_ma = "hold", 0.0
            self._touch()

    def release(self, sid):
        """Limp - same as set_torque(sid, 0)."""
        self.set_torque(sid, 0)

    def hold_all(self):
        for sid in self.motors:
            self.hold(sid)

    def release_all(self):
        for sid in self.motors:
            self.release(sid)

    def clear_fault(self, sid):
        """Let a faulted motor run again (and retry talking to it if it had gone offline).
        It comes back holding still (or limp, if the fault had released it) until you
        give it a new command."""
        m = self._motor(sid)
        with self._io, self._lock:   # _io: wait for any update() in progress to finish
            m.request = "release" if m.released else "hold"
            m.target_ma = 0.0
            m.fault, m.fault_limp, m.offline, m.misses, m.travel, m.last_pos = None, False, False, 0, 0, None
            m.sign_votes, m.sign_confirmed = [0, 0], False
            m.reg.reset()

    # ------------------------------------------------------------------ the control loop
    def update(self):
        """One control step for all motors: one read of everyone, one write of everyone.
        Call this regularly yourself, or use run_in_background() instead."""
        if not self.started:
            raise RuntimeError("call start() first")
        if self._thread is not None and threading.current_thread() is not self._thread:
            raise RuntimeError("the background loop is already calling update()")

        now = time.monotonic()
        if self._last_update is not None:
            dt = now - self._last_update
            if dt > 0:
                self.loop_hz = 1 / dt if self.loop_hz == 0 else 0.9 * self.loop_hz + 0.1 / dt
        self._last_update = now

        with self._lock:
            if self.command_timeout and now - self._last_command > self.command_timeout:
                if not self._watchdog_tripped:
                    print(f"[bus] no command for {self.command_timeout} s - all motors: {self.watchdog_action}")
                    self._watchdog_tripped = True
                for m in self.motors.values():
                    m.request, m.target_ma = self.watchdog_action, 0.0
            plan = {sid: (m.request, m.target_ma) for sid, m in self.motors.items()}

        with self._io:
            polled = [sid for sid, m in self.motors.items() if not m.offline]
            feedback = self._read_all(polled)
            commands = {}
            for sid in polled:
                m = self.motors[sid]
                fb = feedback.get(sid)
                if fb is None:
                    m.misses += 1
                    if m.misses >= self.miss_limit:
                        m.offline, m.last_pos, m.speed_cmd = True, None, 0.0
                        self._set_fault(m, f"no reply {m.misses} times in a row - check wiring and power")
                    if not m.released:
                        commands[sid] = m.speed_cmd   # ride through a dropped packet on the last command
                    continue
                m.misses = 0
                m.feedback = fb
                moved = self._track(m, fb.position)
                m.reg.measure(fb)

                request, target = plan[sid]
                if m.fault:
                    want_on = not (m.fault_limp or m.released)
                else:
                    want_on = request != "release"
                if not self._set_torque_state(m, want_on):
                    continue
                if m.released:
                    m.speed_cmd, m.travel = 0.0, 0
                    continue
                if m.fault or request == "hold":
                    m.reg.reset()
                    m.travel = 0
                    speed = 0.0
                else:
                    speed = self._guard(m, moved, m.reg.speed(target), target)
                m.speed_cmd = speed
                commands[sid] = speed
            if commands:
                self._write_speeds(commands)

    def run_in_background(self, rate_hz=100.0):
        """Run update() at up to rate_hz in a daemon thread until close() / stop_background().
        If the loop hits an error it stops all motors and stores it in .error (see raise_if_error)."""
        if not self.started:
            raise RuntimeError("call start() first")
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, args=(rate_hz,), daemon=True, name="TorqueBus")
        self._thread.start()

    def stop_background(self):
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=2)
            self._thread = None

    def raise_if_error(self):
        if self.error is not None:
            raise RuntimeError(f"torque loop stopped: {self.error}") from self.error

    # ------------------------------------------------------------------ reading state
    def status(self):
        """Snapshot of every motor, for display or logging: {id: {...}}."""
        out = {}
        for sid, m in self.motors.items():
            fb, cfg = m.feedback, m.cfg
            tension = m.reg.tension_ma
            if m.offline:
                state = "offline"
            elif m.fault:
                state = "fault"
            elif m.released:
                state = "limp"
            elif m.request == "regulate":
                state = "regulating"
            else:
                state = "holding"
            force = None
            if tension is not None and cfg.spool_radius_m:
                force = tension / 1000.0 * cfg.kt_nm_per_a / cfg.spool_radius_m
            out[sid] = {
                "name": self._label(m),
                "state": state,
                "target_ma": m.target_ma,
                "tension_ma": tension,            # filtered, signed pull estimate
                "force_n": force,                 # None unless spool_radius_m is set
                "current_ma": fb.current_ma if fb else None,
                "position": fb.position if fb else None,
                "load_pct": fb.load / 10 if fb else None,
                "temperature_c": fb.temperature if fb else None,
                "voltage": fb.voltage if fb else None,
                "speed_cmd": m.speed_cmd,
                "overload_flag": bool(m.status_bits & ERRBIT_OVERLOAD),
                "fault": m.fault,
            }
        return out

    def print_status(self):
        print(f"--- {self.loop_hz:5.0f} Hz ---")
        for sid, s in self.status().items():
            pull = "  --  " if s["tension_ma"] is None else f"{s['tension_ma']:5.0f}"
            force = "" if s["force_n"] is None else f" ({s['force_n']:5.2f} N)"
            extra = (" OVERLOAD" if s["overload_flag"] else "") + (f"  {s['fault']}" if s["fault"] else "")
            print(f"  [{sid}] {s['name']:<14} {s['state']:<10} target {s['target_ma']:5.0f} mA  "
                  f"pull {pull} mA{force}  cmd {s['speed_cmd']:6.0f}  pos {s['position']}  "
                  f"{s['temperature_c']} C{extra}")

    # ------------------------------------------------------------------ internals
    def _touch(self):
        self._last_command = time.monotonic()
        self._watchdog_tripped = False

    def _motor(self, sid):
        if sid not in self.motors:
            raise KeyError(f"no motor with ID {sid}; configured IDs are {list(self.motors)}")
        return self.motors[sid]

    @staticmethod
    def _label(m):
        return m.cfg.name or f"ID {m.cfg.id}"

    def _answers(self, sid):
        try:
            self.servo.read_feedback(sid)
            return True
        except tc.CommError:
            return False

    def _set_fault(self, m, why, limp=False):
        if m.fault is None:
            print(f"[{self._label(m)}] FAULT: {why}. It's {'limp' if limp else 'stopped'}; "
                  f"call clear_fault({m.cfg.id}) to resume.")
        m.fault = why
        m.fault_limp = m.fault_limp or limp
        m.reg.reset()

    def _set_torque_state(self, m, want_on):
        """Engage or release a motor if needed. Returns False if the servo didn't take it."""
        if want_on == (not m.released):
            return True
        sid = m.cfg.id
        try:
            if want_on:
                self.servo.enter_mode(sid, tc.MODE_SPEED)   # zero speed first, then torque on
            else:
                self.servo.torque_enable(sid, False)
        except tc.CommError as exc:
            print(f"[{self._label(m)}] could not {'engage' if want_on else 'release'}: {exc}")
            return False
        m.released = not want_on
        m.reg.reset()
        m.travel = 0
        m.speed_cmd = 0.0
        return True

    def _track(self, m, pos):
        """Steps moved since the last sample (servo coordinates), unwrapped across 4095 -> 0."""
        d = 0
        if m.last_pos is not None:
            d = pos - m.last_pos
            if d > tc.STEPS_PER_REV // 2:
                d -= tc.STEPS_PER_REV
            elif d < -tc.STEPS_PER_REV // 2:
                d += tc.STEPS_PER_REV
        m.last_pos = pos
        return d

    def _guard(self, m, moved, speed, target):
        """Safety checks on a regulating motor. Returns the speed to actually send."""
        cfg = m.cfg
        pulled_in = moved * cfg.direction
        was_winding = m.speed_cmd * cfg.direction > 0

        # Slack guard: winding in a long way without the pull ever reaching the target.
        # Paying out (the arm pulling the tendon) is normal, so it doesn't count.
        if m.reg.correcting and speed * cfg.direction > 0:
            m.travel += max(0, pulled_in)
        else:
            m.travel = 0
        if m.travel > cfg.max_travel:
            self._set_fault(m, f"wound in {m.travel} steps without reaching {target:.0f} mA - "
                               f"slack tendon or wrong direction? (MotorConfig.max_travel)")
            return 0.0

        # Load-sign check: while the servo winds in under its own power it must be driving in
        # the pull direction. If it reports the opposite, the tension estimate has the wrong
        # sign and the loop would pull ever harder - so go limp and say so.
        if not m.sign_confirmed and was_winding and pulled_in >= 2 and m.feedback.current_ma > 2 * cfg.deadband_ma:
            agrees = m.feedback.load * cfg.direction * cfg.load_sign > 0
            m.sign_votes[0 if agrees else 1] += 1
            n = sum(m.sign_votes)
            if n >= SIGN_CHECK_SAMPLES:
                if m.sign_votes[1] > 0.8 * n:
                    self._set_fault(m, f"load sign looks inverted (servo reports driving outward while "
                                       f"winding in). Set MotorConfig(id={cfg.id}, load_sign={-cfg.load_sign})",
                                    limp=True)
                    return 0.0
                m.sign_confirmed = True
        return speed

    def _probe_sync_read(self):
        """Older firmware may not support sync read; fall back to one read per servo if so."""
        self.use_sync_read = True
        result = self._read_all(list(self.motors))
        if all(fb is None for fb in result.values()):
            self.use_sync_read = False
            print("[bus] sync read not answered - falling back to one read per servo (slower loop)")

    def _read_all(self, sids):
        if not sids:
            return {}
        if not self.use_sync_read:
            out = {}
            for sid in sids:
                try:
                    out[sid] = self.servo.read_feedback(sid)
                except tc.CommError:
                    out[sid] = None
            return out
        g = self._sync_read
        if self._poll_ids != sids:        # membership changes only when a motor goes offline or returns
            g.clearParam()
            for sid in sids:
                g.addParam(sid)
            self._poll_ids = list(sids)
        for sid in sids:
            g.data_dict[sid] = []         # so a missing reply can't masquerade as last cycle's data
        g.txRxPacket()
        out = {}
        for sid in sids:
            ok, error = g.isAvailable(sid, tc.ADDR_FEEDBACK, tc.FEEDBACK_LEN)
            if ok:
                self.motors[sid].status_bits = error
                out[sid] = self._parse(g.data_dict[sid][1:])
            else:
                out[sid] = None
        return out

    def _parse(self, data):
        s = self.servo

        def word(i):
            return s.scs_makeword(data[i], data[i + 1])

        return tc.Feedback(
            position=s.scs_tohost(word(0), 15),
            speed=s.scs_tohost(word(2), 15),
            load=s.scs_tohost(word(4), tc.LOAD_SIGN_BIT),
            voltage=data[6] / 10.0,
            temperature=data[7],
            current_ma=abs(s.scs_tohost(word(13), 15)) * tc.CURRENT_MA_PER_COUNT,
        )

    def _write_speeds(self, commands):
        """One broadcast packet sets every listed motor's speed at once."""
        s, g = self.servo, self._sync_write
        g.clearParam()
        for sid, speed in commands.items():
            raw = s.scs_toscs(int(round(speed)), 15)
            g.addParam(sid, [s.scs_lobyte(raw), s.scs_hibyte(raw)])
        result = g.txPacket()
        if result != tc.COMM_SUCCESS:
            raise tc.CommError(f"sync write: {s.getTxRxResult(result)}")

    def _loop(self, rate_hz):
        period = 1.0 / rate_hz
        try:
            while not self._stop.is_set():
                start = time.perf_counter()
                self.update()
                leftover = period - (time.perf_counter() - start)
                if leftover > 0:
                    time.sleep(leftover)
        except Exception as exc:   # stop everything, then report through raise_if_error()
            self.error = exc
            try:
                with self._io:
                    self._write_speeds({sid: 0 for sid, m in self.motors.items()
                                        if not m.offline and not m.released})
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Opt-in command-line tools
# ---------------------------------------------------------------------------
def cmd_check(bus):
    """Read only: ping each configured servo, show its feedback, and test sync read."""
    print(f"Checking {len(bus.motors)} servo(s) on {bus.port_name}:")
    ok = []
    for sid, m in bus.motors.items():
        model, result, _ = bus.servo.ping(sid)
        if result != tc.COMM_SUCCESS:
            print(f"  [{sid}] {TorqueBus._label(m):<14} NOT FOUND ({bus.servo.getTxRxResult(result)})")
            continue
        fb = bus.servo.read_feedback(sid)
        ok.append(sid)
        print(f"  [{sid}] {TorqueBus._label(m):<14} model {model}  pos {fb.position}  "
              f"{fb.current_ma:.0f} mA  {fb.voltage:.1f} V  {fb.temperature} C")
    if ok:
        bus.use_sync_read = True
        got = bus._read_all(ok)
        n = sum(fb is not None for fb in got.values())
        print(f"  sync read: {n}/{len(ok)} answered" + ("" if n == len(ok) else
              " - the bus will fall back to one read per servo"))
    if len(ok) == len(bus.motors):
        print("All servos found. Nothing was moved.")
    else:
        print("Missing servos: run --scan to see which IDs are present. Factory-new servos are all ID 1,\n"
              "so give each its own ID with --set-id while it's the only one connected.")


def cmd_scan(bus, max_id):
    """Read only: ping IDs 0..max_id."""
    print(f"Scanning IDs 0-{max_id} on {bus.port_name} ...")
    found = []
    for sid in range(max_id + 1):
        model, result, _ = bus.servo.ping(sid)
        if result == tc.COMM_SUCCESS:
            found.append(sid)
            print(f"  ID {sid}: model {model}")
    print(f"Found {len(found)} servo(s): {found}" if found else "No servos answered.")


def cmd_set_id(bus, old, new):
    """Change a servo's ID permanently (EEPROM). Connect only that one servo."""
    if not 0 <= new <= 253:
        raise SystemExit("new ID must be 0-253")
    s = bus.servo
    if s.ping(old)[1] != tc.COMM_SUCCESS:
        raise SystemExit(f"no servo answers as ID {old}; run --scan")
    if old != new and s.ping(new)[1] == tc.COMM_SUCCESS:
        raise SystemExit(f"ID {new} is already taken on this bus")
    s._check(*s.unLockEprom(old), "unlock EEPROM")
    # The reply to this write may come back from the NEW id, which the SDK's TxRx calls would
    # reject, so send it without waiting and throw away whatever reply arrives.
    if s.write1ByteTxOnly(old, tc.SMS_STS_ID, new) != tc.COMM_SUCCESS:
        raise SystemExit("could not send the new ID")
    time.sleep(0.1)
    while bus.port.readPort(64):
        pass
    s._check(*s.LockEprom(new), "lock EEPROM")
    if s.ping(new)[1] != tc.COMM_SUCCESS:
        raise SystemExit(f"wrote ID {new}, but nothing answers as {new} - power-cycle and run --scan")
    print(f"Servo ID changed {old} -> {new}. Label it, then do the next one.")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=PORT, help="serial port (default: auto-detect)")
    p.add_argument("--baud", type=int, default=BAUDRATE)
    action = p.add_mutually_exclusive_group()
    action.add_argument("--check", action="store_true", help="ping configured servos and show feedback (read only)")
    action.add_argument("--scan", nargs="?", const=20, type=int, metavar="MAX_ID",
                        help="list servo IDs that answer, 0..MAX_ID (default 20, read only)")
    action.add_argument("--set-id", nargs=2, type=int, metavar=("OLD", "NEW"),
                        help="change one servo's ID (connect only that servo)")
    args = p.parse_args(argv)

    if not (args.check or args.scan is not None or args.set_id):
        print(__doc__)
        print("Nothing was sent to the servos.")
        return

    with TorqueBus(args.port, MOTORS, args.baud) as bus:   # never start()ed, so close() sends nothing
        if args.check:
            cmd_check(bus)
        elif args.scan is not None:
            cmd_scan(bus, args.scan)
        else:
            cmd_set_id(bus, *args.set_id)


if __name__ == "__main__":
    main()
