"""
spirob_teleop.py - drive the three-tendon SpiRobs-style arm the way the paper's 3-cable videos do.

    python spirob_teleop.py --dry-run     practise in the window, no servos touched
    python spirob_teleop.py               real servos (auto-detects /dev/cu.usbmodem*)
    python -m unittest test_spirob_teleop

Needs pygame (pip install pygame) and, for real servos, multi_torque.py in the same folder.

WHAT IS COMMANDED: CABLE LENGTH, NOT FORCE
Each servo is told how many mm of cable to have pulled in (+) or let out (-) compared with
where it was when the program started, and it turns either way to get there. (Force control
was tried first; this arm bends at 1-3 N and the servos cannot feel less than about 6 N, so
they wound in forever and never reversed.) START WITH THE ARM STRAIGHT AND THE CABLES JUST
TAUT - use the trim keys below to get there; that state is "zero".

THE IDEA (from the SpiRobs paper, "Bioinspired grasping strategy" and "3-cable SpiRobs")
    AIM    a compass direction around the base: where the object is
    NEAR   0..1  pull on the aim side      (the paper's F2: reach, wrap, grasp)
    FAR    0..1  pull on the opposite side (the paper's F1: curl up / pack)
Bending toward a direction means pulling the cables on that side AND letting out the ones on
the other side: each cable moves PULL_MAX_MM * (NEAR - FAR) * cos(aim - its angle). NEAR and
FAR together cancel as bending and instead shorten all three a little (SQUEEZE_MM), which
stiffens the arm - the paper's "reach" stage.

CONTROLS (also listed in the window)
    Gamepad   left stick = aim, and how far you push = NEAR.  RT = NEAR.  LT = FAR.
              A = next grasp stage   B = relax   X = whip   Y = whip + grab
    Mouse     click or drag in the circle = aim and NEAR (stays where you leave it)
              scroll = FAR up/down   right-click = relax
    Keyboard  arrows = push the bend left/right/up/down   A/D = swing the aim around
              W/S = FAR up/down   G = next grasp stage (Pack > Reach > Wrap > Grasp > Release)
              H = whip   J = whip + grab   Space = relax (straight)   C = clear faults
              [ ] = rotate the picture to match where you sit   Q or Esc = quit (goes limp)
    Trim      hold 1 / 2 / 3 = wind tendon A / B / C in; with Shift = let it out
              hold = / - = all three in / out (hold - to free a collapsed arm)
"""
import argparse
import math
from dataclasses import replace

# ---------------------------------------------------------------------------
# Configuration - edit these
# ---------------------------------------------------------------------------
PORT = None                        # None = auto-detect; or "/dev/cu.usbmodem1101", or --port
SERVO_IDS = [1, 2, 3]              # tendon A, B, C
NAMES = ["A", "B", "C"]
TENDON_ANGLES_DEG = [32.0, 152.0, 272.0]   # where each tendon sits around the base (MATLAB legend)
SCREEN_ROTATION_DEG = 0.0          # turns the picture so "up" on screen is "away from you"; [ ] keys

PULL_MAX_MM = 50.0                 # cable pulled on the aim side at full NEAR. The MATLAB model's
                                   # fullest curls use 76-107 mm: raise in steps once 50 looks safe
SQUEEZE_MM = 4.0                   # all three shortened by this when NEAR and FAR are both full
PULL_LIMIT_MM = 150.0              # no cable is ever asked to move further than this from zero
SLEW_MM_PER_S = 30.0               # how fast a cable target may move under manual control
MACRO_SLEW_MM_PER_S = 1000.0       # ... and during whip / stages (then only servo speed limits it)
TRIM_MM_PER_S = 10.0               # trim keys
TICK_HZ = 50

# Servo side. Speed and strength are tied together: TORQUE_LIMIT caps how hard a servo can
# pull, and also how fast it can spin. Raise both for a snappier whip, carefully.
TORQUE_LIMIT = 400                 # 0..1000 = 0..100 % of stall torque
MAX_SPEED_STEPS_S = 1200.0         # 4096 steps = 1 turn = ~115 mm of cable, so 1200 is ~33 mm/s
POSITION_GAIN = 8.0                # steps/s of speed per step of error; lower if a servo buzzes
WIND_STOP_MA = 250.0               # stop winding IN while motor current is above this (jam guard)

KEY_RATE = 0.7                     # NEAR / FAR change per second while a key is held
AIM_RATE_DEG_S = 120.0             # A / D
SCROLL_STEP = 0.05
STICK_DEADZONE = 0.2

# Grasp sequence: (name, seconds, NEAR, FAR). These are the paper's piecewise-linear stages.
STAGES = [
    ("Pack",    2.0, 0.0, 1.0),    # curl up away from the object
    ("Reach",   2.0, 0.8, 1.0),    # add the aim side: uncurls and stiffens toward the object
    ("Wrap",    3.0, 0.8, 0.0),    # slowly let the far side go: curls around the object
    ("Grasp",   0.5, 1.0, 0.0),    # squeeze
    ("Release", 2.0, 0.0, 0.0),
]
GRASP_STAGE = 4                    # 1-based position of "Grasp" in STAGES
# Whip: sawtooth on the far side (wind up, then let go at once); "grab" then pulls the aim side.
WHIP_WIND_S, WHIP_HOLD_S, WHIP_GRAB_DELAY_S = 2.0, 0.3, 0.5
COMMAND_TIMEOUT_S = 0.5


# ---------------------------------------------------------------------------
# Cable mapping
# ---------------------------------------------------------------------------
def pulls(aim, near, far, pull_max=PULL_MAX_MM, squeeze=SQUEEZE_MM, angles_deg=TENDON_ANGLES_DEG):
    """mm of cable to pull in (+) or let out (-) on each tendon.

    A tendon sitting exactly on the aim side must shorten the most, one on the opposite side
    must lengthen the most, and one at right angles does not change: that is cos(aim - angle).
    (Same rule as the pull1/2/3_mm columns of the MATLAB CSV.) The three always add up to 0."""
    bend = pull_max * (near - far)
    both = squeeze * min(near, far)
    return [bend * math.cos(aim - math.radians(a)) + both for a in angles_deg]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Timeline:
    """Plays (seconds, near, far) steps as straight ramps; 0 seconds = jump."""

    def __init__(self, near, far, steps):
        self.start = self.cur = (near, far)
        self.steps, self.t = list(steps), 0.0

    @property
    def done(self):
        return not self.steps

    def advance(self, dt):
        while self.steps:
            dur, n1, f1 = self.steps[0]
            if dur <= 0 or self.t + dt >= dur:
                dt = max(0.0, dt - max(0.0, dur - self.t))
                self.start = self.cur = (n1, f1)
                self.t = 0.0
                self.steps.pop(0)
            else:
                self.t += dt
                k = self.t / dur
                self.cur = (self.start[0] + k * (n1 - self.start[0]),
                            self.start[1] + k * (f1 - self.start[1]))
                break
        return self.cur


class Teleop:
    """Holds AIM / NEAR / FAR, runs the grasp stages and whip, and sends cable pulls to the bus."""

    def __init__(self, bus, ids=SERVO_IDS, pull_max=PULL_MAX_MM):
        self.bus, self.ids, self.pull_max = bus, list(ids), pull_max
        self.trim = [0.0] * len(self.ids)      # mm; moves each cable's "zero"
        self.aim = math.radians(90.0)
        self.near = self.far = 0.0             # set by keys, mouse, stages; stay where left
        self.live_near = self.live_far = 0.0   # gamepad: spring back when you let go
        self.stage = 0                         # 0 = free, else 1-based index into STAGES
        self.timeline = None
        self.slew = SLEW_MM_PER_S
        self.sent = [0.0] * len(self.ids)
        self.faults = {}

    # ---- manual input (anything here stops a running stage/whip where it is)
    def _manual(self):
        self.timeline, self.slew = None, SLEW_MM_PER_S

    def set_vector(self, x, y):
        """Bend vector in the arm's frame: direction = aim, length (0..1) = NEAR."""
        self._manual()
        r = math.hypot(x, y)
        if r > 1e-6:
            self.aim = math.atan2(y, x)
        self.near = min(1.0, r)

    def nudge(self, dx, dy):
        self.set_vector(self.near * math.cos(self.aim) + dx, self.near * math.sin(self.aim) + dy)

    def rotate(self, d_rad):
        self.aim += d_rad                      # aiming never interrupts a stage

    def add_far(self, d):
        self._manual()
        self.far = clamp(self.far + d, 0.0, 1.0)

    def set_live(self, aim=None, stick=0.0, near=0.0, far=0.0):
        """Gamepad state every tick. The stick's push counts as NEAR only in free steering."""
        if aim is not None:
            self.aim = aim
        free = self.stage == 0 and self.timeline is None
        self.live_near = clamp(max(near, stick if free else 0.0), 0.0, 1.0)
        self.live_far = clamp(far, 0.0, 1.0)

    def add_trim(self, i, d_mm):
        """Wind tendon i in (+) or out (-); i = None for all three. Does not touch a stage."""
        for k in (range(len(self.ids)) if i is None else [i]):
            self.trim[k] = clamp(self.trim[k] + d_mm, -PULL_LIMIT_MM, PULL_LIMIT_MM)

    def relax(self):
        self._manual()
        self.near = self.far = 0.0
        self.stage = 0

    # ---- stages and whip
    def next_stage(self):
        self.stage = self.stage % len(STAGES) + 1
        _, dur, n, f = STAGES[self.stage - 1]
        self.timeline, self.slew = Timeline(self.near, self.far, [(dur, n, f)]), MACRO_SLEW_MM_PER_S
        if self.stage == len(STAGES):
            self.stage = 0

    def whip(self, grab=False):
        steps = [(WHIP_WIND_S, 0.0, 1.0), (WHIP_HOLD_S, 0.0, 1.0), (0.0, 0.0, 0.0)]
        if grab:
            steps += [(WHIP_GRAB_DELAY_S, 0.0, 0.0), (0.0, 1.0, 0.0)]
        self.timeline, self.slew = Timeline(self.near, self.far, steps), MACRO_SLEW_MM_PER_S
        self.stage = GRASP_STAGE if grab else 0    # after a grab, G = Release

    # ---- every tick
    def totals(self):
        return max(self.near, self.live_near), max(self.far, self.live_far)

    def targets(self):
        near, far = self.totals()
        return [clamp(p, -PULL_LIMIT_MM, PULL_LIMIT_MM) + t
                for p, t in zip(pulls(self.aim, near, far, self.pull_max), self.trim)]

    def tick(self, dt):
        if self.timeline:
            self.near, self.far = self.timeline.advance(dt)
            if self.timeline.done:
                self.timeline = None
        step = self.slew * dt
        self.sent = [s + clamp(t - s, -step, step) for s, t in zip(self.sent, self.targets())]
        for sid, f in zip(self.ids, self.sent):
            self.bus.set_pull_mm(sid, f)
        self.bus.raise_if_error()
        status = self.bus.status()
        faults = {sid: status[sid]["fault"] for sid in self.ids if status[sid].get("fault")}
        if set(faults) - set(self.faults):     # a new fault: straighten the others too
            self.relax()
        self.faults = faults
        return status

    def clear_faults(self):
        for sid in list(self.faults):
            self.bus.clear_fault(sid)


# ---------------------------------------------------------------------------
# Buses
# ---------------------------------------------------------------------------
class MockBus:
    """Stands in for the servo bus in --dry-run: no hardware, 'measured' pull lags the target."""

    def __init__(self, ids=SERVO_IDS):
        self.ids = list(ids)
        self.target = {i: 0.0 for i in self.ids}
        self.pull = {i: 0.0 for i in self.ids}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def start(self):
        pass

    def run_in_background(self, rate_hz=100):
        pass

    def set_pull_mm(self, sid, mm):
        self.target[sid] = mm

    def release_all(self):
        pass

    def clear_fault(self, sid):
        pass

    def raise_if_error(self):
        pass

    def status(self):
        out = {}
        for i in self.ids:
            self.pull[i] += 0.15 * (self.target[i] - self.pull[i])
            out[i] = {"state": "regulating", "pull_mm": self.pull[i], "current_ma": 0.0, "fault": None}
        return out


def with_length_control(bus_class):
    """Subclass of TorqueBus that moves each cable to a length instead of a force.

    TorqueBus already reads every servo, counts the steps each has turned, sends speeds, and
    handles the watchdog and lost servos. This keeps all of that and replaces one decision:
    the speed sent to a motor is POSITION_GAIN * (steps it should have wound in - steps it
    has), in either direction. Hooks the private _track() (called with every position
    reading) and _guard() (called once per control step for every running motor)."""

    class LengthBus(bus_class):
        def __init__(self, *args, gain=POSITION_GAIN, max_speed=MAX_SPEED_STEPS_S,
                     wind_stop_ma=WIND_STOP_MA, **kwargs):
            super().__init__(*args, **kwargs)
            self._gain, self._max_speed, self._wind_stop_ma = gain, max_speed, wind_stop_ma
            self._wound = {sid: 0 for sid in self.motors}      # steps wound in since start
            self._goal = {sid: 0.0 for sid in self.motors}
            self._mm_per_step = {sid: 2 * math.pi * m.cfg.spool_radius_m * 1000 / 4096
                                 for sid, m in self.motors.items()}

        def set_pull_mm(self, sid, mm):
            self._motor(sid)
            with self._lock:
                self._goal[sid] = mm / self._mm_per_step[sid]
            self.set_torque(sid, 100)          # any pull above 0: keeps the motor on, feeds the watchdog

        def _track(self, m, pos):
            d = super()._track(m, pos)
            self._wound[m.cfg.id] += d * m.cfg.direction
            return d

        def _guard(self, m, moved, speed, target):
            sid = m.cfg.id
            with self._lock:
                err = self._goal[sid] - self._wound[sid]
            m.travel = 0
            if abs(err) < 3:                   # a few steps (~0.1 mm): close enough, sit still
                return 0.0
            v = clamp(self._gain * err, -self._max_speed, self._max_speed)
            pull = m.reg.tension_ma
            if v > 0 and pull is not None and pull > self._wind_stop_ma:
                v = 0.0                        # something is resisting hard: don't wind in further
            return m.cfg.direction * v

        def status(self):
            out = super().status()
            for sid in out:
                out[sid]["pull_mm"] = self._wound[sid] * self._mm_per_step[sid]
            return out

    return LengthBus


def make_bus(dry_run, port=None):
    if dry_run:
        return MockBus(SERVO_IDS)
    from multi_torque import MOTORS, TorqueBus    # imported here so --dry-run needs no serial setup
    motors = sorted((m for m in MOTORS if m.id in SERVO_IDS), key=lambda m: SERVO_IDS.index(m.id))
    if len(motors) != len(SERVO_IDS):
        raise SystemExit(f"SERVO_IDS {SERVO_IDS} not all found in multi_torque.MOTORS")
    motors = [replace(m, torque_limit=TORQUE_LIMIT) for m in motors]
    return with_length_control(TorqueBus)(port or PORT, motors, command_timeout=COMMAND_TIMEOUT_S,
                                          watchdog_action="release", on_exit="release")


# ---------------------------------------------------------------------------
# Window: keyboard + mouse + gamepad
# ---------------------------------------------------------------------------
BG, FG, DIM = (24, 26, 30), (230, 230, 230), (120, 124, 132)
NEAR_C, FAR_C, WARN_C = (80, 190, 255), (255, 160, 60), (255, 90, 90)
TENDON_C = [(230, 80, 80), (90, 200, 110), (240, 190, 70)]     # as in the MATLAB plots
HELP = [
    ("Stick / drag / arrows", "aim + NEAR (bend toward)"),
    ("RT", "NEAR"),
    ("LT / scroll / W S", "FAR (curl back)"),
    ("A D", "swing the aim"),
    ("G  (pad A)", "next grasp stage"),
    ("H / J  (pad X / Y)", "whip / whip + grab"),
    ("Space / right-click  (pad B)", "relax"),
    ("1 2 3   (+Shift)", "trim tendon in (out)"),
    ("=   -", "trim all in / out"),
    ("C   [ ]   Q", "clear faults, rotate view, quit"),
]


def run_window(bus, dry_run):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((1080, 620))
    pygame.display.set_caption("SpiRob teleop" + ("  [dry run]" if dry_run else ""))
    font, small = pygame.font.SysFont(None, 26), pygame.font.SysFont(None, 21)
    clock = pygame.time.Clock()

    pad = None
    try:
        from pygame._sdl2 import controller as sdl_pad
        sdl_pad.init()
        if sdl_pad.get_count() and sdl_pad.is_controller(0):
            pad = sdl_pad.Controller(0)
    except Exception as exc:                   # no gamepad support: keyboard and mouse still work
        print(f"gamepad unavailable: {exc}")

    teleop = Teleop(bus)
    rot = math.radians(SCREEN_ROTATION_DEG)
    cx, cy, R = 300, 310, 230
    dragging, status, running = False, {}, True

    def to_screen(angle, r):
        return cx + r * math.cos(angle + rot), cy - r * math.sin(angle + rot)

    def text(s, x, y, color=FG, f=None):
        screen.blit((f or font).render(s, True, color), (x, y))

    def draw():
        screen.fill(BG)
        near, far = teleop.totals()
        pygame.draw.circle(screen, DIM, (cx, cy), R, 1)
        pygame.draw.circle(screen, DIM, (cx, cy), R // 2, 1)
        for i, a in enumerate(TENDON_ANGLES_DEG):               # tendons: dot grows with tension
            k = (teleop.sent[i] - teleop.trim[i]) / teleop.pull_max
            x, y = to_screen(math.radians(a), R + 22)
            pygame.draw.circle(screen, TENDON_C[i], (x, y), 11 + 10 * clamp(k, 0, 1))
            text(NAMES[i], x - 6, y - 9, BG, small)
        pygame.draw.line(screen, DIM, to_screen(teleop.aim + math.pi, R), to_screen(teleop.aim, R), 1)
        pygame.draw.line(screen, FAR_C, (cx, cy), to_screen(teleop.aim + math.pi, far * R), 6)
        pygame.draw.line(screen, NEAR_C, (cx, cy), to_screen(teleop.aim, near * R), 6)
        pygame.draw.circle(screen, NEAR_C, to_screen(teleop.aim, near * R), 9)
        pygame.draw.circle(screen, FG, to_screen(teleop.aim, R), 6)             # the aim
        text("AIM", *to_screen(teleop.aim, R + 48), DIM, small)
        text(f"NEAR {near:4.2f}", 20, 16, NEAR_C)
        text(f"FAR  {far:4.2f}", 20, 42, FAR_C)

        x0, span = 620, max(teleop.pull_max, 1.0) * 1.3
        for i, sid in enumerate(teleop.ids):                    # cable bars: middle = zero, right = pulled in
            y = 30 + i * 46
            st = status.get(sid, {})
            mid = x0 + 130

            def bar_x(mm):
                return mid + 100 * clamp(mm / span, -1, 1)
            text(f"{NAMES[i]}", x0, y, TENDON_C[i])
            pygame.draw.rect(screen, DIM, (x0 + 30, y, 200, 20), 1)
            x1 = bar_x(teleop.sent[i])
            pygame.draw.rect(screen, TENDON_C[i], (min(mid, x1), y, abs(x1 - mid), 20))
            pygame.draw.line(screen, DIM, (mid, y - 3), (mid, y + 23), 1)
            meas = st.get("pull_mm")
            if meas is not None:
                pygame.draw.line(screen, FG, (bar_x(meas), y - 4), (bar_x(meas), y + 24), 2)
            label = f"{teleop.sent[i]:+6.1f} mm" + ("" if meas is None else f"  ({meas:+6.1f})")
            text(label, x0 + 240, y, FG, small)
            cur = st.get("current_ma")
            text(f"{st.get('state', '')}   trim {teleop.trim[i]:+.1f}" + ("" if cur is None else f"   {cur:.0f} mA"),
                 x0 + 30, y + 22, DIM, small)
        text("bar = asked, white tick and (..) = where the servo is", x0, 172, DIM, small)

        text("Grasp sequence", x0, 210)
        for i, (name, *_rest) in enumerate(STAGES):
            on = teleop.stage == i + 1
            text(("> " if on else "  ") + name, x0 + 10 + (0 if on else 4), 238 + i * 22, FG if on else DIM, small)
        if teleop.timeline:
            text("running ...", x0 + 150, 210, NEAR_C, small)
        for i, (what, does) in enumerate(HELP):
            text(what, x0 - 40, 360 + i * 21, FG, small)
            text(does, x0 + 170, 360 + i * 21, DIM, small)
        y = 580
        if teleop.faults:
            text("FAULT  " + "; ".join(f"{NAMES[teleop.ids.index(s)]}: {f}" for s, f in teleop.faults.items())[:95]
                 + "   (C to clear)", 20, y, WARN_C, small)
        else:
            text(("DRY RUN - no servos.  " if dry_run else "") + (f"Gamepad: {pad.name}" if pad else "No gamepad"),
                 20, y, DIM, small)
        pygame.display.flip()

    try:
        while running:
            dt = min(clock.tick(TICK_HZ) / 1000.0, 0.1)
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    if e.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif e.key == pygame.K_SPACE:
                        teleop.relax()
                    elif e.key == pygame.K_g:
                        teleop.next_stage()
                    elif e.key == pygame.K_h:
                        teleop.whip()
                    elif e.key == pygame.K_j:
                        teleop.whip(grab=True)
                    elif e.key == pygame.K_c:
                        teleop.clear_faults()
                    elif e.key in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                        rot += math.radians(15 if e.key == pygame.K_LEFTBRACKET else -15)
                elif e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1 and math.hypot(e.pos[0] - cx, e.pos[1] - cy) <= R + 40:
                        dragging = True
                    elif e.button == 3:
                        teleop.relax()
                elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                    dragging = False
                elif e.type == pygame.MOUSEWHEEL:
                    teleop.add_far(SCROLL_STEP * e.y)
                elif e.type == pygame.CONTROLLERDEVICEADDED and pad is None:
                    pad = sdl_pad.Controller(e.device_index)
                elif e.type == pygame.CONTROLLERDEVICEREMOVED:
                    pad = None
                    teleop.set_live()
                elif e.type == pygame.CONTROLLERBUTTONDOWN:
                    if e.button == pygame.CONTROLLER_BUTTON_A:
                        teleop.next_stage()
                    elif e.button == pygame.CONTROLLER_BUTTON_B:
                        teleop.relax()
                    elif e.button == pygame.CONTROLLER_BUTTON_X:
                        teleop.whip()
                    elif e.button == pygame.CONTROLLER_BUTTON_Y:
                        teleop.whip(grab=True)

            keys = pygame.key.get_pressed()
            dx, dy = keys[pygame.K_RIGHT] - keys[pygame.K_LEFT], keys[pygame.K_UP] - keys[pygame.K_DOWN]
            if dx or dy:                                        # screen direction -> arm direction
                a, step = math.atan2(dy, dx) - rot, KEY_RATE * dt
                teleop.nudge(step * math.cos(a), step * math.sin(a))
            if keys[pygame.K_a] != keys[pygame.K_d]:
                teleop.rotate(math.radians(AIM_RATE_DEG_S) * dt * (1 if keys[pygame.K_a] else -1))
            shift = -1 if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] else 1
            for i, k in enumerate((pygame.K_1, pygame.K_2, pygame.K_3)):
                if keys[k]:
                    teleop.add_trim(i, shift * TRIM_MM_PER_S * dt)
            if keys[pygame.K_EQUALS] != keys[pygame.K_MINUS]:
                teleop.add_trim(None, TRIM_MM_PER_S * dt * (1 if keys[pygame.K_EQUALS] else -1))
            if keys[pygame.K_w] != keys[pygame.K_s]:
                teleop.add_far(KEY_RATE * dt * (1 if keys[pygame.K_w] else -1))
            if dragging:
                mx, my = pygame.mouse.get_pos()
                a, r = math.atan2(cy - my, mx - cx) - rot, min(1.0, math.hypot(mx - cx, my - cy) / R)
                teleop.set_vector(r * math.cos(a), r * math.sin(a))
            if pad:
                sx = pad.get_axis(pygame.CONTROLLER_AXIS_LEFTX) / 32767.0
                sy = -pad.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / 32767.0
                mag = math.hypot(sx, sy)
                aim = math.atan2(sy, sx) - rot if mag > STICK_DEADZONE else None
                stick = clamp((mag - STICK_DEADZONE) / (1 - STICK_DEADZONE), 0.0, 1.0)
                teleop.set_live(aim, stick,
                                pad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0,
                                pad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0)

            status = teleop.tick(dt)
            draw()

        teleop.set_live()                                       # normal quit: ease back to straight first
        teleop.relax()
        for _ in range(int(2.5 * TICK_HZ)):
            status = teleop.tick(1.0 / TICK_HZ)
            draw()
            clock.tick(TICK_HZ)
    finally:
        pygame.quit()
        print(f"SCREEN_ROTATION_DEG was {math.degrees(rot) % 360:.0f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="window only; no servos")
    ap.add_argument("--port", help="serial port, e.g. /dev/cu.usbmodem1101 (default: auto-detect)")
    args = ap.parse_args(argv)

    with make_bus(args.dry_run, args.port) as bus:
        bus.start()                                    # all limp
        bus.run_in_background(100)
        try:
            run_window(bus, args.dry_run)
        except KeyboardInterrupt:
            print("stopping")
        finally:
            try:
                bus.release_all()
            except Exception as exc:
                print(f"release failed: {exc}")
    # leaving the with-block stops all motors (on_exit="release": they go limp)


if __name__ == "__main__":
    main()