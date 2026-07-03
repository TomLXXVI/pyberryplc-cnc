# pyberryplc-cnc

`pyberryplc-cnc` connects Cartesian path planning to stepper-motor execution for
small CNC-style automation projects.

The package compiles XYZ paths into the pulse-delay JSON format consumed by
`pyberryplc-stepper` motor controllers. It also provides an `XYZMotionController`
for PLC applications that coordinate X, Y, and Z stepper axes.

This package is intended as a practical bridge between:

- `python-robot` for Cartesian trajectory generation;
- `pyberryplc-stepper` for stepper motor pulse execution;
- `pyberryplc` for PLC-style scan-cycle control.

## What It Offers

- XYZ vertex paths with optional blended Cartesian motion.
- Feed-rate based or explicit segment-duration based path timing.
- Axis calibration from the same TOML motor configuration used by the runtime
  controller.
- Conversion from Cartesian axis motion to per-axis STEP pulse delays and motor
  directions.
- JSON trajectory export for later execution by a PLC application.
- `XYZMotionController` for coordinated multi-axis execution and jog mode.
- A compact CNC PLC demo with auto mode, manual jog mode, and subroutine-style
  mode controllers.
- Optional NiceGUI path editor through the `pyberryplc-cnc-ui` command.

## Installation

The package requires Python `>=3.10,<3.13`.

From the repository root, install the local dependencies first:

```bash
python -m pip install -e packages/automation-motion
python -m pip install -e packages/pyberryplc
python -m pip install -e packages/pyberryplc-stepper
python -m pip install -e packages/python-robot
python -m pip install -e packages/pyberryplc-cnc
```

For development:

```bash
python -m pip install -e "packages/pyberryplc-cnc[dev]"
```

To install the optional browser-based path editor:

```bash
python -m pip install -e "packages/pyberryplc-cnc[ui]"
```

Runtime dependencies include `numpy`, `python-robot`, `pyberryplc-stepper`, and
`tomli` on Python versions before 3.11.

## Quick Start

Compile a square path in the XY plane to a stepper trajectory:

```python
from pyberryplc_cnc import compile_xyz_path, save_stepper_trajectory


compiled = compile_xyz_path(
    vertices=[
        (0.0, 0.0, 0.0),
        (0.0, 50.0, 0.0),
        (50.0, 50.0, 0.0),
        (50.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ],
    motor_config_filepath="demos/demo_1/motor_config.toml",
    feed_rate=2.0,
    dt_blends=0.5,
)

save_stepper_trajectory(
    "demos/demo_1/stepper_trajectory.json",
    compiled.stepper_trajectory,
)
```

The generated JSON contains one or more trajectory segments. For each moving
axis, a segment contains:

- a list of delays between STEP pulses;
- a rotation direction such as `"clockwise"` or `"counterclockwise"`.

`XYZMotionController` can then load and execute that trajectory from a PLC
application.

## Motor Configuration

The compiler and controller share the same TOML configuration file. Each axis
uses a section such as:

```toml
[x_motor]
step_pin_ID = 27
dir_pin_ID = 17
comm_port = "/dev/ttyUSB1"
pitch = 0.25
rdir_ref = "clockwise"

[x_motor.microstepping]
resolution = "full"
full_steps_per_rev = 200

[x_motor.current]
run_current_pct = 77.0
hold_current_pct = 10.0
```

`pitch` is expressed in motor revolutions per path unit. If path coordinates
are millimetres, `pitch = 0.25` means one quarter motor revolution per
millimetre of axis travel.

`rdir_ref` defines which motor rotation direction produces positive travel on
that axis.

## Optional Path Editor

With the `ui` extra installed, launch the browser-based path editor:

```bash
pyberryplc-cnc-ui
```

The editor lets you define XYZ vertices, select a motor configuration file,
choose feed rate and blend time, and write the compiled trajectory JSON.

## Documentation And Demos

- `docs/trajectory_compiler_explained.md`: how XYZ paths become step-pulse
  timing data.
- `docs/cnc_plc_demo.md`: walkthrough of the CNC PLC demo.
- `demos/demo_1/create_trajectory.py`: compiles the example path.
- `demos/demo_1/cnc_demo.py`: keyboard-driven PLC application with auto and
  manual modes.
- `demos/demo_1/motor_config.toml`: example three-axis motor configuration.

## Demos

Generate the demo trajectory:

```bash
python demos/demo_1/create_trajectory.py
```

Run the PLC demo on a Raspberry Pi style target:

```bash
cd demos/demo_1
sudo python cnc_demo.py
```

Or use the included shell script:

```bash
./demos/demo_1/run_demo.sh
```

## Safety Notes

`pyberryplc-cnc` is not a complete CNC machine controller. The demo code is
educational and intentionally compact.

Before adapting it to real machinery, add at least:

- physical emergency-stop wiring;
- limit switches and homing;
- soft travel limits;
- coordinate-system and work-offset handling;
- feed hold and resume behaviour;
- clear operator feedback;
- a deliberate recovery strategy after emergency or fault conditions.

## Package Map

```text
pyberryplc_cnc.trajectory_compiler
    XYZVertex, XYZPath, AxisCalibration, blended profile creation, XYZ path
    compilation, and JSON trajectory export.

pyberryplc_cnc.controller
    XYZMotionController, MotionStatus, and MotorStatus for PLC-side multi-axis
    motion control.

pyberryplc_cnc.ui
    Optional NiceGUI path editor exposed through pyberryplc-cnc-ui.
```
