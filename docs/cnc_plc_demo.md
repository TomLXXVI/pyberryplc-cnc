# CNC PLC Demo

This demo shows how `pyberryplc-cnc`, `pyberryplc-stepper`, and
`pyberryplc` can be combined into a small three-axis CNC application. The PLC
has two operating modes:

- auto mode loads a precomputed JSON trajectory and executes it segment by
  segment;
- manual mode jogs one axis while a keyboard key is held.

The demo is intentionally compact. It is not a complete machine controller:
there is no homing routine, no limit switch handling, no coordinate system
management, and no feed hold. The value of the example is that the structure is
small enough to read while still resembling a real PLC program.

## Files

The demo lives in `demos/demo_1`.

- `cnc_demo.py` contains the `AbstractPLC` application.
- `create_trajectory.py` compiles the example path to
  `stepper_trajectory.json`.
- `motor_config.toml` contains the pinout, UART ports, motor pitch,
  microstepping, current settings, and positive-travel direction for each axis.
- `run_demo.sh` creates the trajectory and starts the PLC on a Raspberry Pi
  style target.

`stepper_trajectory.json` is generated locally and is not committed to the
repository.

## Motor Configuration

The motor configuration file uses one section per axis:

```toml
[x_motor]
step_pin_ID = 27
dir_pin_ID = 17
comm_port = "/dev/ttyUSB1"
pitch = 0.25
rdir_ref = "clockwise"
```

`pitch` is expressed in motor revolutions per path unit. In the demo comments
the path unit is millimetres, so `pitch = 0.25` means that the motor turns one
quarter revolution per millimetre of axis travel.

`rdir_ref` defines which motor rotation direction corresponds to positive axis
travel. The trajectory compiler uses this value to choose the direction string
stored in the JSON trajectory. Manual mode uses the same value to decide which
direction is "plus" for jogging.

## Creating The Trajectory

Before the PLC can run auto mode, generate the trajectory:

```bash
python demos/demo_1/create_trajectory.py
```

The default path is a square in the XY plane:

```text
(0, 0, 0) -> (0, 50, 0) -> (50, 50, 0) -> (50, 0, 0) -> (0, 0, 0)
```

`create_trajectory.py` calls `compile_xyz_path()` and writes the result with
`save_stepper_trajectory()`. The output format is the low-level timing recipe
expected by `XYZMotionController`: for each trajectory segment, every moving
axis receives a list of STEP-pulse delays and one rotation direction.

## Main PLC Sequence

`CNCPLC` owns the scan cycle. Its main sequence has only three steps:

- `S0`: idle and mode selection;
- `S1`: auto-mode subroutine active;
- `S2`: manual-mode subroutine active.

The main sequence decides when to enter a mode. It does not contain the details
of trajectory execution or jogging. That separation is deliberate: it keeps the
top-level GRAFCET readable and lets each mode manage its own state.

Keyboard inputs are mapped to `MemoryVariable` instances:

- `a`: select auto mode;
- `m`: select manual mode;
- `o`: switch off mode selection;
- `shift+s`: start;
- `q`: exit;
- `e`: emergency;
- `r`: reset after emergency.

## Subroutine Concept

`AutoMode` and `ManualMode` are not threads. They are ordinary Python objects
called from the parent PLC scan. This is close to a GRAFCET subroutine or
macrostep:

1. The parent sequence activates a mode step.
2. The parent calls `subroutine.call()` during each scan while that step is
   active.
3. The subroutine owns its own markers and transitions.
4. When the parent leaves the mode, the subroutine can be reset and later
   reused.

The markers still belong to the parent PLC because they are created with
`main.add_marker(...)`. This matters for edge detection: a subroutine step has
the same `active`, `rising_edge`, and `falling_edge` behaviour as a main PLC
step.

## AutoMode

`AutoMode` has three steps:

- `AutoMode.S0`: waiting for the start command inside auto mode;
- `AutoMode.S1`: loading `stepper_trajectory.json`;
- `AutoMode.S2`: executing trajectory segments.

In `S1`, the trajectory is loaded into `XYZMotionController`. In `S2`, the
subroutine waits until all motor controllers report ready and then sends the
next segment. When the controller reports that all segment motions are
finished, `TrajectoryFinished` is set. The main PLC sees that flag and returns
to `S0`.

## ManualMode

`ManualMode` maps the jog keys to axis-specific jog commands:

- `shift+x` / `x`: jog X plus/minus;
- `shift+y` / `y`: jog Y plus/minus;
- `shift+z` / `z`: jog Z plus/minus.

For each jog direction there is a start step and a stop step. The start step is
entered on a key rising edge and calls `start_jog_mode()`. The stop step is
entered on the key falling edge and calls `stop_jog_mode()`. The subroutine
returns to its idle step after the controller reports that jog mode is no
longer active for that axis.

This shows why a subroutine can be useful: manual mode needs many small states,
but none of them have to clutter the top-level PLC sequence.

## Running The Demo

On a Raspberry Pi style target with the virtual environment available, run:

```bash
cd packages/pyberryplc-cnc/demos/demo_1
python create_trajectory.py
sudo python cnc_demo.py
```

Or use:

```bash
./run_demo.sh
```

The shell script expects the project virtual environment by default. Set
`PYBERRYPLC_VENV` when the environment is located elsewhere.

## Safety Notes

This demo is educational. Before using the structure on a real machine, add at
least:

- physical emergency-stop wiring;
- limit switches and homing;
- soft travel limits;
- a controlled recovery strategy after emergency mode;
- feed hold and resume behaviour;
- clear operator feedback for the current machine state.
