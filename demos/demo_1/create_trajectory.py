"""
Create the stepper trajectory used by the CNC PLC demo.

The generated JSON file is consumed by
:class:`pyberryplc_cnc.controller.XYZMotionController` in ``cnc_demo.py``.
Run this script before starting the demo PLC.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from pyberryplc_cnc import XYZVertex, compile_xyz_path, save_stepper_trajectory


DEMO_DIR = Path(__file__).resolve().parent
DEFAULT_MOTOR_CONFIG = DEMO_DIR / "motor_config.toml"
DEFAULT_TRAJECTORY = DEMO_DIR / "stepper_trajectory.json"
DEFAULT_VERTICES = (
    (0.0, 0.0, 0.0),
    (0.0, 50.0, 0.0),
    (50.0, 50.0, 0.0),
    (50.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
)


def create_demo_trajectory(
    motor_config_filepath: str | Path = DEFAULT_MOTOR_CONFIG,
    output_filepath: str | Path = DEFAULT_TRAJECTORY,
    *,
    vertices: Sequence[XYZVertex | Sequence[float]] = DEFAULT_VERTICES,
    feed_rate: float = 2.0,
    dt_blends: float = 0.5,
    axes: Sequence[str] = ("x", "y", "z"),
    include_stationary_axes: bool = True,
) -> int:
    """
    Compile and save the demo trajectory.

    Parameters
    ----------
    motor_config_filepath:
        TOML file containing the motor pinout, microstepping, pitch, and
        direction reference for each CNC axis.
    output_filepath:
        JSON file to write. The PLC demo loads this file in auto mode.
    vertices:
        Cartesian path vertices in demo units. The default path is a square in
        the XY plane.
    feed_rate:
        Cartesian feed rate in path units per second.
    dt_blends:
        Blend time at the path vertices. Set this to ``0.0`` for sharp
        constant-speed segments.
    axes:
        Axis names to compile.
    include_stationary_axes:
        If True, every segment contains X, Y, and Z entries, even when an axis
        is stationary.

    Returns
    -------
    int
        Number of generated trajectory segments.
    """
    compiled = compile_xyz_path(
        vertices=vertices,
        motor_config_filepath=motor_config_filepath,
        feed_rate=feed_rate,
        dt_blends=dt_blends,
        axes=axes,
        include_stationary_axes=include_stationary_axes,
    )
    save_stepper_trajectory(output_filepath, compiled.stepper_trajectory)
    return len(compiled.stepper_trajectory)


def main() -> None:
    """
    Create ``stepper_trajectory.json`` next to this script.
    """
    num_segments = create_demo_trajectory()
    print(f"Wrote {num_segments} trajectory segments to {DEFAULT_TRAJECTORY}")


if __name__ == "__main__":
    main()
