"""
spirob_teleop.py - drive the three-tendon SpiRobs-style arm the way the paper's 3-cable videos do.

    python spirob_teleop.py --dry-run     practise in the window, no servos touched
    python spirob_teleop.py               real servos (auto-detects /dev/cu.usbmodem*)
    python spirob_teleop.py --calibrate   measure how well each tendon releases (no window)
    python -m unittest test_spirob_teleop

Needs pygame (pip install pygame) and, for real servos, multi_torque.py in the same folder.

THE IDEA (from the SpiRobs paper, "Bioinspired grasping strategy" and "3-cable SpiRobs")
The paper works a 2-cable arm with two forces: F1 curls it up, F2 on the other side makes it
reach out, wrap and grasp. With three cables the same thing is done about any direction. So
this controller has just three numbers, and everything is tendon TENSION (set_force_n):

    AIM    a compass direction around the base: where the object is
    NEAR   0..1  pull on the aim side      (the paper's F2: reach, wrap, grasp)
    FAR    0..1  pull on the opposite side (the paper's F1: curl up / pack)

    NEAR alone          bends toward the aim (free steering); sweep the aim to circle
    FAR alone           curls up away from the aim (packing)
    FAR, then add NEAR  uncurls from the base and reaches toward the aim
    then lower FAR      wraps around whatever it touched
    then NEAR to full   firm grasp

CONTROLS (also listed in the window)
    Gamepad   left stick = aim, and how far you push = NEAR.  RT = NEAR.  LT = FAR.
              A = next grasp stage   B = relax   X = whip   Y = whip + grab
    Mouse     click or drag in the circle = aim and NEAR (stays where you leave it)
              scroll = FAR up/down   right-click = relax
    Keyboard  arrows = push the bend left/right/up/down   A/D = swing the aim around
              W/S = FAR up/down   G = next grasp stage (Pack > Reach > Wrap > Grasp > Release)
              H = whip   J = whip + grab   Space = relax (straight)   C = clear faults
              [ ] = rotate the picture to match where you sit   Q or Esc = quit (goes limp)
"""
import argparse
import math
import time
from dataclasses import replace

# ---------------------------------------------------------------------------
# Configuration - edit these
# ---------------------------------------------------------------------------
PORT = None                        # None = auto-detect; or "/dev/cu.usbmodem1101", or --port
SERVO_IDS = [1, 2, 3]              # tendon A, B, C
NAMES = ["A", "B", "C"]
TENDON_ANGLES_DEG = [32.0, 152.0, 272.0]   # where each tendon sits around the base (MATLAB legend)
SCREEN_ROTATION_DEG = 0.0          # turns the picture so "up" on screen is "away from you"; [ ] keys

T_REST_N = 6.0                     # N on every tendon at rest. README: below ~6 N is lost in gear friction
T_MAX_N = 20.0                     # N ceiling for any tendon. A guess for this arm: raise in small steps
SLEW_N_PER_S = 30.0                # how fast a tension may change under manual control
MACRO_SLEW_N_PER_S = 400.0         # ... and during whip / grasp stages (the whip needs a sudden drop)
TICK_HZ = 50

# Release assist. The servo library only pays cable out when it MEASURES more pull than the
# target, and a stopped 1:345 gearbox hides the tendon's pull from the motor current, so a
# lowered target often does nothing. With this above 0, every newton a target drops also
# unwinds this many mm of cable; the normal force loop then re-tightens to the new target.
# Run --calibrate to see whether you need it and what number to use. 0 = off.
RELEASE_ASSIST_MM_PER_N = 0.0
ASSIST_SPEED_STEPS_S = 800.0       # unwind speed (servo max ~3000; faster = snappier whip)
ASSIST_MAX_MM = 150.0              # never owe more unwinding than this

KEY_RATE = 0.7                     # NEAR / FAR change per second while a key is held
AIM_RATE_DEG_S = 120.0             # A / D
SCROLL_STEP = 0.05
STICK_DEADZONE = 0.2

# Grasp sequence: (name, seconds, NEAR, FAR). These are the paper's piecewise-linear stages.
STAGES = [
    ("Pack",    1.5, 0.0, 1.0),    # curl up away from the object
    ("Reach",   2.0, 0.8, 1.0),    # add the aim side: uncurls from the base toward the object
    ("Wrap",    3.0, 0.8, 0.0),    # slowly let the far side go: climbs around the object
    ("Grasp",   0.5, 1.0, 0.0),    # squeeze
    ("Release", 1.5, 0.0, 0.0),
]
GRASP_STAGE = 4                    # 1-based position of "Grasp" in STAGES
# Whip: sawtooth on the far side (wind up, then let go at once); "grab" then pulls the aim side.
WHIP_WIND_S, WHIP_HOLD_S, WHIP_GRAB_DELAY_S = 1.0, 0.2, 0.3

CALIBRATE_TEST_N = 15.0            # --calibrate pulls each tendon to this, then lets it back down
COMMAND_TIMEOUT_S = 0.5


# ---------------------------------------------------------------------------
# Tension mapping
# ---------------------------------------------------------------------------
def pattern(theta, angles_deg=TENDON_ANGLES_DEG):
    """Share (0..1) of pull on each tendon that bends the arm toward angle theta (radians).

    Each tendon tugs the tip toward its own angle. 2/3*cos(theta - angle) is the mix of three
    tugs that adds up to exactly direction theta; tendons can't push, so the whole set is
    shifted up until the smallest is 0, then scaled so no share ever exceeds 1."""
    raw = [2.0 / 3.0 * math.cos(theta - math.radians(a)) for a in angles_deg]
    lo = min(raw)
    return [(r - lo) * math.sqrt(3) / 2 for r in raw]


def tensions(aim, near, far, rest=T_REST_N, tmax=T_MAX_N, angles_deg=TENDON_ANGLES_DEG):
    """Absolute tension (N) for each tendon: rest + NEAR toward the aim + FAR away from it."""
    p, q = pattern(aim, angles_deg), pattern(aim + math.pi, angles_deg)
    return [min(tmax, rest + (tmax - rest) * (near * p[i] + far * q[i])) for i in range(len(p))]


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
    """Holds AIM / NEAR / FAR, runs the grasp stages and whip, and sends tensions to the bus."""

    def __init__(self, bus, ids=SERVO_IDS, rest=T_REST_N, tmax=T_MAX_N):
        self.bus, self.ids, self.rest, self.tmax = bus, list(ids), rest, tmax
        self.aim = math.radians(90.0)
        self.near = self.far = 0.0             # set by keys, mouse, stages; stay where left
        self.live_near = self.live_far = 0.0   # gamepad: spring back when you let go
        self.stage = 0                         # 0 = free, else 1-based index into STAGES
        self.timeline = None
        self.slew = SLEW_N_PER_S
        self.sent = [rest] * len(self.ids)
        self.faults = {}

    # ---- manual input (anything here stops a running stage/whip where it is)
    def _manual(self):
        self.timeline, self.slew = None, SLEW_N_PER_S

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

    def relax(self):
        self._manual()
        self.near = self.far = 0.0
        self.stage = 0

    # ---- stages and whip
    def next_stage(self):
        self.stage = self.stage % len(STAGES) + 1
        _, dur, n, f = STAGES[self.stage - 1]
        self.timeline, self.slew = Timeline(self.near, self.far, [(dur, n, f)]), MACRO_SLEW_N_PER_S
        if self.stage == len(STAGES):
            self.stage = 0

    def whip(self, grab=False):
        steps = [(WHIP_WIND_S, 0.0, 1.0), (WHIP_HOLD_S, 0.0, 1.0), (0.0, 0.0, 0.0)]
        if grab:
            steps += [(WHIP_GRAB_DELAY_S, 0.0, 0.0), (0.0, 1.0, 0.0)]
        self.timeline, self.slew = Timeline(self.near, self.far, steps), MACRO_SLEW_N_PER_S
        self.stage = GRASP_STAGE if grab else 0    # after a grab, G = Release

    # ---- every tick
    def totals(self):
        return max(self.near, self.live_near), max(self.far, self.live_far)

    def targets(self):
        near, far = self.totals()
        return tensions(self.aim, near, far, self.rest, self.tmax)

    def tick(self, dt):
        if self.timeline:
            self.near, self.far = self.timeline.advance(dt)
            if self.timeline.done:
                self.timeline = None
        step = self.slew * dt
        self.sent = [s + clamp(t - s, -step, step) for s, t in zip(self.sent, self.targets())]
        for sid, f in zip(self.ids, self.sent):
            self.bus.set_force_n(sid, f)
        self.bus.raise_if_error()
        status = self.bus.status()
        faults = {sid: status[sid]["fault"] for sid in self.ids if status[sid].get("fault")}
        if set(faults) - set(self.faults):     # a new fault: stop pulling with the others too
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
    """Stands in for TorqueBus in --dry-run: no hardware, 'measured' force just lags the target."""

    def __init__(self, ids=SERVO_IDS):
        self.ids = list(ids)
        self.target = {i: 0.0 for i in self.ids}
        self.force = {i: 0.0 for i in self.ids}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def start(self):
        pass

    def run_in_background(self, rate_hz=100):
        pass

    def set_force_n(self, sid, newtons):
        self.target[sid] = newtons

    def release_all(self):
        self.target = {i: 0.0 for i in self.ids}

    def clear_fault(self, sid):
        pass

    def raise_if_error(self):
        pass

    def status(self):
        out = {}
        for i in self.ids:
            self.force[i] += 0.15 * (self.target[i] - self.force[i])
            out[i] = {"state": "regulating" if self.target[i] else "limp", "force_n": self.force[i],
                      "position": 0, "temperature_c": 25, "fault": None}
        return out


def with_release_assist(bus_class):
    """Subclass of TorqueBus that also unwinds cable whenever a force target is lowered.

    It keeps a per-motor count of steps "owed". Lowering a target adds to it, raising a target
    cancels it, and while any is owed the motor unwinds at a fixed speed instead of listening
    to the (friction-blinded) force reading. Hooks set_torque() and the private _guard(), which
    TorqueBus calls once per control step for every regulating motor to get the speed to send."""

    class AssistBus(bus_class):
        def __init__(self, *args, assist_mm_per_n=0.0, assist_speed=ASSIST_SPEED_STEPS_S,
                     assist_max_mm=ASSIST_MAX_MM, **kwargs):
            super().__init__(*args, **kwargs)
            self._owed, self._steps_per_ma, self._owed_max = {}, {}, {}
            self._assist_speed = assist_speed
            for sid, m in self.motors.items():
                r = m.cfg.spool_radius_m or 0.0
                mm_per_step = 2 * math.pi * r * 1000 / 4096
                n_per_ma = m.cfg.kt_nm_per_a / 1000 / r if r else 0.0
                self._owed[sid] = 0.0
                self._steps_per_ma[sid] = assist_mm_per_n / mm_per_step * n_per_ma if r else 0.0
                self._owed_max[sid] = assist_max_mm / mm_per_step if r else 0.0

        def set_torque(self, sid, ma):
            m = self._motor(sid)
            with self._lock:
                old = m.target_ma if m.request == "regulate" else None
            super().set_torque(sid, ma)
            with self._lock:
                if old is None or m.request != "regulate":
                    self._owed[sid] = 0.0
                else:
                    owed = self._owed[sid] + (old - m.target_ma) * self._steps_per_ma[sid]
                    self._owed[sid] = clamp(owed, 0.0, self._owed_max[sid])

        def _guard(self, m, moved, speed, target):
            sid = m.cfg.id
            with self._lock:
                owed = self._owed[sid]
                if owed >= 8:                                   # a few steps: not worth a twitch
                    paid = max(0, -moved * m.cfg.direction) if m.speed_cmd * m.cfg.direction < 0 else 0
                    self._owed[sid] = owed = max(0.0, owed - paid)
            if owed < 8:
                return super()._guard(m, moved, speed, target)
            m.travel = 0
            m.reg.reset()
            return -m.cfg.direction * self._assist_speed

    return AssistBus


def make_bus(dry_run, port=None, assist_mm_per_n=RELEASE_ASSIST_MM_PER_N):
    if dry_run:
        return MockBus(SERVO_IDS)
    from multi_torque import MOTORS, TorqueBus    # imported here so --dry-run needs no serial setup
    motors = sorted((m for m in MOTORS if m.id in SERVO_IDS), key=lambda m: SERVO_IDS.index(m.id))
    if len(motors) != len(SERVO_IDS):
        raise SystemExit(f"SERVO_IDS {SERVO_IDS} not all found in multi_torque.MOTORS")
    motors = [replace(m, vmax_out=max(m.vmax_out, ASSIST_SPEED_STEPS_S)) for m in motors]
    kwargs = dict(command_timeout=COMMAND_TIMEOUT_S, watchdog_action="release", on_exit="release")
    if assist_mm_per_n > 0:
        return with_release_assist(TorqueBus)(port or PORT, motors, assist_mm_per_n=assist_mm_per_n, **kwargs)
    return TorqueBus(port or PORT, motors, **kwargs)


# ---------------------------------------------------------------------------
# --calibrate: does lowering the force actually pay cable out?
# ---------------------------------------------------------------------------
def unwrap(prev, pos, steps_per_rev=4096):
    """Change in position between two samples, across the 4095 -> 0 rollover."""
    d = pos - prev
    if d > steps_per_rev // 2:
        d -= steps_per_rev
    elif d < -steps_per_rev // 2:
        d += steps_per_rev
    return d


def calibrate(bus, test_n=CALIBRATE_TEST_N, rest=T_REST_N):
    from multi_torque import MOTORS
    cfg = {m.id: m for m in MOTORS}
    test_n = min(test_n, T_MAX_N)
    forces = {sid: rest for sid in SERVO_IDS}
    last, total = {}, {sid: 0 for sid in SERVO_IDS}

    def run(seconds, sid=None, goal=None, rate=5.0):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if sid is not None:
                forces[sid] += clamp(goal - forces[sid], -rate * 0.02, rate * 0.02)
            for s, f in forces.items():
                bus.set_force_n(s, f)
            bus.raise_if_error()
            for s, st in bus.status().items():
                if st["fault"]:
                    raise SystemExit(f"tendon {s} faulted: {st['fault']}")
                if st["position"] is not None:
                    if s in last:
                        total[s] += unwrap(last[s], st["position"]) * cfg[s].direction
                    last[s] = st["position"]
            time.sleep(0.02)

    print(f"All tendons to {rest:.0f} N ...")
    run(4.0)
    rows = []
    for sid, name in zip(SERVO_IDS, NAMES):
        mm = 2 * math.pi * cfg[sid].spool_radius_m * 1000 / 4096
        p0 = total[sid]
        print(f"Tendon {name}: pulling to {test_n:.0f} N ...")
        run((test_n - rest) / 5.0 + 3.0, sid, test_n)
        p1 = total[sid]
        print(f"Tendon {name}: lowering to {rest:.0f} N and waiting ...")
        run((test_n - rest) / 5.0 + 5.0, sid, rest)
        p2 = total[sid]
        rows.append((name, (p1 - p0) * mm, (p1 - p2) * mm))
    print(f"\n{'tendon':<8}{'wound in':>10}{'came back':>11}{'':>6}  suggested RELEASE_ASSIST_MM_PER_N")
    for name, wound, back in rows:
        pct = 100 * back / wound if wound > 1 else float("nan")
        print(f"{name:<8}{wound:>8.1f}mm{back:>9.1f}mm{pct:>5.0f}%  {0.8 * wound / (test_n - rest):.1f}")
    print("\nIf 'came back' is well under 100 %, set RELEASE_ASSIST_MM_PER_N to about the smallest\n"
          "suggested value (it is already 80 % of what was measured, to err on the taut side).")


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
    ("H  (pad X)", "whip"),
    ("J  (pad Y)", "whip + grab"),
    ("Space / right-click  (pad B)", "relax"),
    ("C   [ ]   Q", "clear faults, rotate view, quit"),
]


def run_window(bus, dry_run):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((980, 620))
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
            k = (teleop.sent[i] - teleop.rest) / (teleop.tmax - teleop.rest)
            x, y = to_screen(math.radians(a), R + 22)
            pygame.draw.circle(screen, TENDON_C[i], (x, y), 8 + 12 * clamp(k, 0, 1))
            text(NAMES[i], x - 6, y - 9, BG, small)
        pygame.draw.line(screen, DIM, to_screen(teleop.aim + math.pi, R), to_screen(teleop.aim, R), 1)
        pygame.draw.line(screen, FAR_C, (cx, cy), to_screen(teleop.aim + math.pi, far * R), 6)
        pygame.draw.line(screen, NEAR_C, (cx, cy), to_screen(teleop.aim, near * R), 6)
        pygame.draw.circle(screen, NEAR_C, to_screen(teleop.aim, near * R), 9)
        pygame.draw.circle(screen, FG, to_screen(teleop.aim, R), 6)             # the aim
        text("AIM", *to_screen(teleop.aim, R + 48), DIM, small)
        text(f"NEAR {near:4.2f}", 20, 16, NEAR_C)
        text(f"FAR  {far:4.2f}", 20, 42, FAR_C)

        x0 = 620
        for i, sid in enumerate(teleop.ids):                    # tension bars: sent vs measured
            y = 30 + i * 46
            st = status.get(sid, {})
            text(f"{NAMES[i]}", x0, y, TENDON_C[i])
            pygame.draw.rect(screen, DIM, (x0 + 30, y, 200, 20), 1)
            pygame.draw.rect(screen, TENDON_C[i], (x0 + 30, y, 200 * clamp(teleop.sent[i] / teleop.tmax, 0, 1), 20))
            meas = st.get("force_n")
            if meas is not None:
                mx = x0 + 30 + 200 * clamp(meas / teleop.tmax, 0, 1)
                pygame.draw.line(screen, FG, (mx, y - 4), (mx, y + 24), 2)
            label = f"{teleop.sent[i]:4.1f} N" + ("" if meas is None else f"  ({meas:4.1f})")
            text(label, x0 + 240, y, FG, small)
            text(st.get("state", ""), x0 + 30, y + 22, DIM, small)
        text("bar = sent, white tick and (..) = measured", x0, 172, DIM, small)

        text("Grasp sequence", x0, 210)
        for i, (name, *_rest) in enumerate(STAGES):
            on = teleop.stage == i + 1
            text(("> " if on else "  ") + name, x0 + 10 + (0 if on else 4), 238 + i * 22, FG if on else DIM, small)
        if teleop.timeline:
            text("running ...", x0 + 150, 210, NEAR_C, small)
        for i, (what, does) in enumerate(HELP):
            text(what, x0 - 40, 372 + i * 21, FG, small)
            text(does, x0 + 170, 372 + i * 21, DIM, small)
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

        teleop.set_live()                                       # normal quit: ease back to rest first
        teleop.relax()
        for _ in range(int(1.0 * TICK_HZ)):
            status = teleop.tick(1.0 / TICK_HZ)
            draw()
            clock.tick(TICK_HZ)
    finally:
        pygame.quit()
        print(f"SCREEN_ROTATION_DEG was {math.degrees(rot) % 360:.0f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="window only; no servos")
    ap.add_argument("--calibrate", action="store_true", help="measure wind-in vs release for each tendon")
    ap.add_argument("--port", help="serial port, e.g. /dev/cu.usbmodem1101 (default: auto-detect)")
    ap.add_argument("--assist", type=float, default=RELEASE_ASSIST_MM_PER_N, metavar="MM_PER_N",
                    help="release assist, mm of cable unwound per newton lowered (0 = off)")
    args = ap.parse_args(argv)
    if args.calibrate and args.dry_run:
        raise SystemExit("--calibrate needs the real servos")

    with make_bus(args.dry_run, args.port, 0.0 if args.calibrate else args.assist) as bus:
        bus.start()                                    # all limp
        bus.run_in_background(100)
        try:
            if args.calibrate:
                calibrate(bus)
            else:
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