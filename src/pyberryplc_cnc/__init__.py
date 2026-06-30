"""
CNC adapters for combining python-robot motion planning with pyberryplc-stepper.

The package exposes helpers that compile blended Cartesian XYZ paths into the
JSON-compatible step-pulse format consumed by
:class:`pyberryplc_stepper.controller.XYZMotionController`.
"""

from .trajectory import (
    AxisCalibration,
    CompiledXYZTrajectory,
    RotationDirection,
    XYZPath,
    XYZVertex,
    compile_blended_profile,
    compile_xyz_path,
    create_blended_xyz_profile,
    load_axis_calibrations_from_toml,
    save_stepper_trajectory,
)

__all__ = [
    "AxisCalibration",
    "CompiledXYZTrajectory",
    "RotationDirection",
    "XYZPath",
    "XYZVertex",
    "compile_blended_profile",
    "compile_xyz_path",
    "create_blended_xyz_profile",
    "load_axis_calibrations_from_toml",
    "save_stepper_trajectory",
]
