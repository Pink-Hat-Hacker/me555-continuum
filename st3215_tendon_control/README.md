# ST3215 tendon control

Python code for setting the tension on four tendons, each driven by a Waveshare/Feetech
ST3215 bus servo (12 V) with a printed spool. You give each tendon a force, and it winds
in or pays out to hold that force.

| Motor ID | Name           |
|----------|----------------|
| 1        | tendon A       |
| 2        | tendon B       |
| 3        | tendon C       |
| 4        | central tendon |

## Files

- `example_controller.py`: a minimal controller to start from. Running it pulls all four tendons until Ctrl+C.
- `multi_torque.py`: the interface. Import `TorqueBus` from here. The motor list (`MOTORS`) and port are at the top.
- `torque_control.py`: lower-level servo helpers that `multi_torque.py` uses, plus single-servo test tools.
- `scservo_sdk/`: the servo maker's communication library. Don't edit it.

## Setup

Python 3.9 or newer. In PowerShell, from this folder:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Each servo needs its own ID (1–4). New servos all ship as ID 1, so connect them one at a
time and run `python multi_torque.py --set-id 1 <new id>`. Then connect all four and run:

```powershell
python multi_torque.py --check
```

This only reads from the servos; it should list all four as found. If the port isn't
`COM3`, change `PORT` at the top of `multi_torque.py` or pass `--port COM5`.

## Using it

Put your controller script in this folder, so it can import `multi_torque`. The easiest
start is to edit `example_controller.py`. Its core is:

```python
from multi_torque import TorqueBus, PORT, MOTORS

with TorqueBus(PORT, MOTORS, command_timeout=0.5) as bus:
    bus.start()                      # every motor limp; nothing moves yet
    bus.run_in_background(100)       # regulation loop runs in its own thread
    while True:
        bus.set_force_n(1, 10.0)     # your controller decides these each cycle
        bus.set_force_n(4, 6.0)
        bus.raise_if_error()
        ...
```

Leaving the `with` block stops everything, including on Ctrl+C or an error. Motors that
were pulling are left holding their position, and limp ones stay limp.

## Functions

| Call | What it does |
|---|---|
| `TorqueBus(PORT, MOTORS, command_timeout=0.5)` | Opens the port (nothing moves). With `command_timeout`, if no command arrives for that many seconds (for example your loop hangs), every motor holds still. |
| `bus.start()` | Checks all servos answer and sets them up, limp. |
| `bus.run_in_background(hz)` | Starts the loop that keeps each tendon at its target. |
| `bus.set_force_n(id, newtons)` | **Main command.** Above 0: holds that tension, winding in if it's too low and paying out if the arm pulls harder. 0: limp. |
| `bus.hold(id)` | Freezes the motor where it is: torque stays on, but it won't wind in or pay out. |
| `bus.release(id)` | Limp (motor torque off). Same as a force of 0. |
| `bus.hold_all()` / `bus.release_all()` | Same, for every motor. |
| `bus.status()` | Snapshot of every motor: `{id: {"state", "force_n", "target_ma", "position", "temperature_c", "fault", ...}}`. `state` is `limp`, `holding`, `regulating`, `fault` or `offline`. |
| `bus.print_status()` | Prints a table of the above. |
| `bus.raise_if_error()` | Raises if the background loop stopped, for example if USB was unplugged. Call it in your loop. |
| `bus.clear_fault(id)` | Lets a faulted motor run again. It holds still (or stays limp, if it was limp) until its next command. |

Also available: `set_torque(id, mA)` and `set_torque_nm(id, N·m)` (the same thing in other
units), and `set_torques({id: mA})` to set several motors at once.

## Things to know

- **Force is estimated from motor current, not measured directly.** Gear friction puts the
  real tension in a band around the target (expect being off by a newton or two). The
  conversion uses `kt_nm_per_a` and `spool_radius_m` in `MOTORS`; calibrate them with known
  weights if accuracy matters.
- **Keep a pretension of about 6 N or more** on tendons that should give way. Smaller pulls
  are lost in gear friction. A limp motor (0) still resists being pulled, because of its
  1:345 gearbox.
- **Each motor stops on its own fault without affecting the others:** a slack or snapped
  tendon (it winds in too far without reaching its target), a servo that stops replying,
  or a servo reporting it is driving the wrong way. `status()[id]["fault"]` says which.
- **If a motor unwinds when it should tighten,** set `direction=-1` for it in `MOTORS`.
- **To check one servo at a time:** `python torque_control.py monitor --id 1 --hold` holds
  it in place and prints the live current while you push on it.
