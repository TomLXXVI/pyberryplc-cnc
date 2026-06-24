"""
CNC adapters for combining python-robot motion planning with pyberryplc-stepper.

The package exposes helpers that convert sampled Cartesian XYZ trajectories
into the JSON-compatible step-pulse format consumed by
:class:`pyberryplc_stepper.controller.XYZMotionController`, without importing
GPIO or Raspberry Pi driver code during offline trajectory compilation.
"""

from .trajectory import (
    AxisCalibration,
    RotationDirection,
    compile_xyz_samples,
    compile_xyz_stepper_trajectory,
    save_stepper_trajectory,
)

__all__ = [
    "AxisCalibration", "RotationDirection", "compile_xyz_samples",
    "compile_xyz_stepper_trajectory", "save_stepper_trajectory",
]
