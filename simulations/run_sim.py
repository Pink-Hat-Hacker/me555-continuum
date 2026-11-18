"""Simulate the CR_3D_V17 tendon-driven continuum arm in MuJoCo (headless).

    python run_sim.py                 # run all demos, write results/ (plots, gif, csv)
    python run_sim.py --no-gravity    # same, gravity off (matches the kinematics notes)
    python run_sim.py --show 1.2 0 0  # hold tensions (N) on tendons 1,2,3 and save one picture

For the interactive window instead:  python -m mujoco.viewer --mjcf cr_v17.xml
(drag the three 'pull' sliders in the Control panel on the right).

Control input: ctrl[j] = TENSION in newtons on tendon j (>= 0, it can only pull).
That is the same thing your ST3215 TorqueBus does (it regulates force, not length).
"""
import argparse
import os

import sys
if sys.platform.startswith("linux"):
    os.environ.setdefault("MUJOCO_GL", "egl")  # headless rendering, Linux only
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TENDON_DEG = np.array([31.8, 151.8, 271.8])     # direction of each tendon hole about the axis


class Arm:
    def __init__(self, gravity=True):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "cr_v19.xml"))
        if not gravity:
            self.m.opt.gravity[:] = 0
        self.d = mujoco.MjData(self.m)
        self.tip_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "tip")
        mujoco.mj_forward(self.m, self.d)
        self.L0 = self.d.ten_length.copy()       # straight-arm tendon lengths (302.15 mm)
        self.renderer = None

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

    def step(self, tensions, n=1):
        self.d.ctrl[:] = np.clip(tensions, 0, self.m.actuator_ctrlrange[:, 1])
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)

    @property
    def tip(self):                               # mm
        return self.d.site_xpos[self.tip_id] * 1e3

    @property
    def pulled(self):                            # how much each tendon has shortened, mm
        return (self.L0 - self.d.ten_length) * 1e3

    def settle(self, tensions, seconds=4.0):
        self.step(tensions, int(seconds / self.m.opt.timestep))

    def frame(self, w=640, h=720):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.m, height=h, width=w)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.07, 0.03, 0.17]
        cam.distance, cam.azimuth, cam.elevation = 0.78, 50, -8
        self.renderer.update_scene(self.d, camera=cam)
        return self.renderer.render().copy()


def bend_forces(phi_deg, tension_n, bias=None):
    """Three tendon tensions that bend toward direction phi (0 = +x).
    Each tendon pulls in proportion to cos(phi - alpha_j) (the lever-arm idea from your
    kinematics notes); a bias keeps all three >= 0 because a tendon can only pull."""
    c = np.cos(np.deg2rad(phi_deg - TENDON_DEG))
    bias = tension_n if bias is None else bias
    return np.clip(bias + tension_n * c, 0, None)


def demo_single_pull(arm, out):
    """Pull tendon 1 alone, 0 -> 1.5 N, then let go."""
    import matplotlib.pyplot as plt
    arm.reset()
    ts, tips, pulls, F = [], [], [], []
    for step in range(9000):
        t = step * arm.m.opt.timestep
        f1 = 1.5 * min(t / 4.0, 1.0) if t < 6.0 else 0.0
        arm.step([f1, 0, 0])
        if step % 20 == 0:
            ts.append(t); tips.append(arm.tip.copy()); pulls.append(arm.pulled.copy()); F.append(f1)
    ts, tips, pulls, F = map(np.array, (ts, tips, pulls, F))
    np.savetxt(os.path.join(out, "single_pull.csv"), np.c_[ts, F, tips, pulls], delimiter=",",
               header="t_s,F1_N,tip_x_mm,tip_y_mm,tip_z_mm,pull1_mm,pull2_mm,pull3_mm", comments="")
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].plot(pulls[:, 0], F, label="tendon 1 tension")
    ax[0].set_xlabel("tendon 1 shortening (mm)"); ax[0].set_ylabel("tension (N)")
    ax[0].set_title("Force vs. pull (loading, then release)"); ax[0].grid(alpha=.3)
    ax[1].plot(tips[:, 0], tips[:, 2], label="tip path (x-z)")
    ax[1].set_aspect("equal"); ax[1].set_xlabel("x (mm)"); ax[1].set_ylabel("z (mm)")
    ax[1].set_title("Tip path"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "single_pull.png"), dpi=130); plt.close(fig)
    return float(pulls[:, 0].max()), tips[np.argmax(pulls[:, 0])]


def demo_match_notes(arm):
    """Find the tension that shortens tendon 1 by 40 mm and report the tip, to compare with the
    '40 mm pull' numbers in CR_V17_kinematics_notes.md (tip [149 92 201] compliance rule)."""
    lo, hi = 0.0, 3.0
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        arm.reset(); arm.settle([mid, 0, 0], 3.0)
        lo, hi = (mid, hi) if arm.pulled[0] < 40.0 else (lo, mid)
    return mid, arm.pulled.copy(), arm.tip.copy()


def demo_circle(arm, out, tension=0.7, seconds=8.0, fps=12):
    """Sweep the bend direction phi through 360 deg and record the tip + a GIF."""
    import imageio.v2 as imageio
    import matplotlib.pyplot as plt
    arm.reset()
    arm.settle(bend_forces(0, tension), 3.0)
    steps_per_frame = int(1.0 / fps / arm.m.opt.timestep)
    frames, tips = [], []
    n = int(seconds * fps)
    for i in range(n):
        phi = 360.0 * i / n
        arm.step(bend_forces(phi, tension), steps_per_frame)
        tips.append(arm.tip.copy())
        if i % 1 == 0:
            frames.append(arm.frame(320, 400))
    imageio.mimsave(os.path.join(out, "circle_sweep.gif"), frames, duration=1.0 / fps, loop=0)
    tips = np.array(tips)
    fig, ax = plt.subplots(figsize=(4.6, 4.6))
    ax.plot(tips[:, 0], tips[:, 1]); ax.plot(0, 0, "k+")
    ax.set_aspect("equal"); ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_title("Tip, seen from above, as the bend direction rotates"); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "circle_sweep_topview.png"), dpi=130); plt.close(fig)
    return tips


def snapshots(arm, out):
    """Picture strip: increasing tension on tendon 1."""
    import imageio.v2 as imageio
    imgs = []
    for f in (0.0, 0.4, 0.8, 1.2, 1.6):
        arm.reset(); arm.settle([f, 0, 0], 4.0)
        imgs.append(arm.frame(360, 540))
    imageio.imwrite(os.path.join(out, "tendon1_pull_strip.png"), np.hstack(imgs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gravity", action="store_true")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    ap.add_argument("--show", nargs=3, type=float, metavar=("F1", "F2", "F3"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    arm = Arm(gravity=not a.no_gravity)
    print(f"model: {arm.m.nbody-1} links, {arm.m.njnt} neck joints, {arm.m.ntendon} tendons, "
          f"mass {arm.m.body_mass.sum()*1e3:.1f} g, gravity {'on' if not a.no_gravity else 'off'}")
    print("straight tendon length (mm):", np.round(arm.L0 * 1e3, 2))
    if a.show:
        import imageio.v2 as imageio
        arm.settle(a.show, 4.0)
        imageio.imwrite(os.path.join(a.out, "show.png"), arm.frame())
        print("tip (mm):", arm.tip.round(1), " tendon shortening (mm):", arm.pulled.round(1))
        return
    mx, tip_at_max = demo_single_pull(arm, a.out)
    print(f"single pull: max shortening {mx:.1f} mm, tip at that moment {tip_at_max.round(1)} mm")
    f40, pulled, tip40 = demo_match_notes(arm)
    print(f"40 mm pull on tendon 1 needs {f40:.2f} N -> tendon shortening {pulled.round(1)} mm, tip {tip40.round(1)} mm")
    print("   (kinematics notes: tip [149 92 201] mm compliance rule, [168 104 194] mm equal-joint rule)")
    tips = demo_circle(arm, a.out)
    r = np.hypot(tips[:, 0], tips[:, 1])
    print(f"circle sweep: tip radius {r.min():.0f}-{r.max():.0f} mm, tip z {tips[:,2].min():.0f}-{tips[:,2].max():.0f} mm")
    snapshots(arm, a.out)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
