# ME555 Continuum Robotics

## Multi-tendon Retractable Continuum Robot
*Design based on [SpiRobs](https://arxiv.org/pdf/2303.09861)*

## Team
- Dylan | [LinkedIn]() | [Github]()
- Jarvis | [LinkedIn]() | [Github]()
- Kevin | [LinkedIn]() | [Github]()
- Zoë | [LinkedIn]() | [Github]()

---

## How to Run Simulation

### Virtual Env and Dependencies

**macos & linux**
```
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

**windows**
*I'll update this soon, idk yet*

### Run Interactive Mujoco Window
1. `python3 -m mujoco.viewer --mjcf cr_v19.xml`
2. Drag and drop cr_v19.xml into the window
3. Open control dropdown & change tendon values 
