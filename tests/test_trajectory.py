import math

from pyberryplc_cnc import (
    AxisCalibration,
    RotationDirection,
    compile_blended_profile,
    compile_xyz_path,
    create_blended_xyz_profile,
    load_axis_calibrations_from_toml,
)


def _write_motor_config(tmp_path):
    """
    Write a minimal XYZMotionController-style motor configuration.

    Parameters
    ----------
    tmp_path:
        Pytest temporary path fixture.

    Returns
    -------
    pathlib.Path
        Path to the generated TOML file.
    """
    config_path = tmp_path / "motor_config.toml"
    config_path.write_text(
        """
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

[y_motor]
step_pin_ID = 24
dir_pin_ID = 23
comm_port = "/dev/ttyUSB2"
pitch = 0.5
rdir_ref = "counterclockwise"

[y_motor.microstepping]
resolution = "1/16"
full_steps_per_rev = 200

[y_motor.current]
run_current_pct = 77.0
hold_current_pct = 10.0
""",
        encoding="utf-8",
    )
    return config_path


def test_load_axis_calibrations_from_motor_config(tmp_path):
    """
    Load pitch, microstepping, and direction from the controller TOML file.
    """
    calibrations = load_axis_calibrations_from_toml(_write_motor_config(tmp_path))

    assert set(calibrations) == {"x", "y"}
    assert calibrations["x"].direction_for_delta(1.0) == RotationDirection.CW
    assert calibrations["y"].direction_for_delta(1.0) == RotationDirection.CCW
    assert math.isclose(calibrations["x"].steps_per_unit, 50.0)
    assert math.isclose(calibrations["y"].steps_per_unit, 1600.0)


def test_compile_xyz_path_uses_motor_config_and_profile_pieces(tmp_path):
    """
    Compile a vertex path through the piece-based blended profile route.
    """
    compiled = compile_xyz_path(
        vertices=[
            (0.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
        ],
        motor_config_filepath=_write_motor_config(tmp_path),
        feed_rate=2.0,
        dt_blends=0.0,
        axes=("x",),
        include_stationary_axes=False,
    )

    assert compiled.profile.pieces
    assert len(compiled.stepper_trajectory) == 1
    delays, direction = compiled.stepper_trajectory[0]["x"]
    assert len(delays) == 200
    assert direction == RotationDirection.CW
    assert all(math.isclose(delay, 0.01 - 20e-6) for delay in delays)


def test_compile_blended_profile_splits_piece_when_axis_reverses():
    """
    Split blended profile pieces at velocity sign changes.
    """
    profile = create_blended_xyz_profile(
        vertices=[
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ],
        dt_segments=(1.0, 1.0),
        dt_blends=(0.0, 0.4, 0.0),
    )
    trajectory = compile_blended_profile(
        profile,
        {
            "x": AxisCalibration.from_pitch(
                pitch=10.0,
                full_steps_per_rev=10,
                positive_direction="counterclockwise",
            )
        },
        axes=("x",),
        include_stationary_axes=False,
    )

    directions = [
        segment["x"][1]
        for segment in trajectory
        if segment["x"][0]
    ]
    assert RotationDirection.CCW in directions
    assert RotationDirection.CW in directions
