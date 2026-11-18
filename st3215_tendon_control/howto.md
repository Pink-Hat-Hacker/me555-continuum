# Set Up Environment

### Create Virtual Environment
`python3 -m venv .venv`

*If you do not have python management system follow this documentation by [UV](https://docs.astral.sh/uv/#python-versions)*

### Activate
`source .venv/bin/activate`

### PIP install all dependencies
`python3 -m pip install -r requirements.txt`

# Set Up Controller Code
## Initializing Motor ID

1. Connect Driver and 1 Servo
2. `cd st3215_tendon_control/`
3. `python3 multi_torque.py --scan`
4. Per Servo do the following
`python3 multi_torque.py --set-id 1 [new ID number]`