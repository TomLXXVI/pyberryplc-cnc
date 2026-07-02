# Trajectory Compiler Explained

The trajectory compiler converts a CNC path in Cartesian coordinates into step-pulse data that can be executed by `pyberryplc_cnc.controller.XYZMotionController`.

At a high level, it answers one practical question:

> Given a path in X/Y/Z space, when should each stepper motor receive its next STEP pulse, and in which direction should it turn?

The input path is defined as a list of XYZ vertices, for example:

```
(0, 0, 0) -> (0, 50, 0) -> (50, 50, 0)
```

The compiler first turns these vertices into a time-based Cartesian motion profile. Segment durations can be supplied directly, or they can be calculated from a feed rate. The feed rate is interpreted in path units per second. If the coordinates are millimetres, then the feed rate is in mm/s.

Next, the compiler reads the motor calibration from `motor_config.toml`. For each axis, it uses:

- the axis pitch, in motor revolutions per path unit;
- the number of full steps per motor revolution;
- the configured microstepping factor;
- the reference motor direction for positive axis travel.

From this, it calculates how many step pulses correspond to one unit of linear travel. For example, with `pitch = 0.25 rev/mm`, `200 steps/rev`, and full-step mode, the axis needs:

```
0.25 * 200 = 50 steps/mm
```

The Cartesian profile is then split into smaller monotonic intervals. This is important because one axis may change direction during a blended corner. Each interval is compiled separately so that every generated pulse train has one clear direction.

For each axis and each interval, the compiler calculates:

1. the start and end position of that axis;
2. the signed displacement;
3. the motor direction needed for that displacement;
4. the number of step pulses required;
5. the exact profile time at which each step position is reached.

It then converts those step times into delays between pulses. The result is stored as JSON in the format expected by the motion controller:

```
{
  "x": [[0.0012, 0.0011, 0.0010], "clockwise"],
  "y": [[], "counterclockwise"],
  "z": [[], "counterclockwise"]
}
```

Each segment can contain X, Y, and Z entries. A moving axis receives a list of delays. A stationary axis may receive an empty list, depending on the `include_stationary_axes` setting.

The important practical point is that the compiler does not directly command “move 50 mm”. Instead, it produces the low-level timing recipe for the stepper processes: a list of pulse delays plus a direction for each axis.

Blend time controls whether the generated pulse timing includes acceleration and deceleration. With `dt_blends = 0`, the path pieces are purely constant-speed linear moves. With a positive blend time, the profile includes parabolic acceleration regions, so the step delays gradually shrink during acceleration and grow during deceleration.
