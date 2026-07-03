"""
CNC adapters for combining python-robot motion planning with pyberryplc-stepper.

The package exposes helpers that compile blended Cartesian XYZ paths into the
JSON-compatible step-pulse format consumed by
:class:`pyberryplc_cnc.controller.XYZMotionController`.
"""

from .controller import (
    MotorStatus,
    MotionStatus,
    XYZMotionController,
)
from pyberryplc_stepper.rotation_direction import RotationDirection

_TRAJECTORY_EXPORTS = {
    "AxisCalibration",
    "CompiledXYZTrajectory",
    "XYZPath",
    "XYZVertex",
    "compile_blended_profile",
    "compile_xyz_path",
    "create_blended_xyz_profile",
    "load_axis_calibrations_from_toml",
    "save_stepper_trajectory",
}


def __getattr__(name: str) -> object:
    """
    Load trajectory compiler exports on first access.

    Parameters
    ----------
    name:
        Public package attribute requested by the caller.

    Returns
    -------
    object
        Requested public object from ``pyberryplc_cnc.trajectory_compiler``.

    Raises
    ------
    AttributeError
        If ``name`` is not exported by this package.
    """
    if name in _TRAJECTORY_EXPORTS:
        from . import trajectory_compiler

        value = getattr(trajectory_compiler, name)
        globals()[name] = value
        return value

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


# noinspection PyUnresolvedReferences
__all__ = [
    "AxisCalibration",
    "CompiledXYZTrajectory",
    "MotorStatus",
    "MotionStatus",
    "RotationDirection",
    "XYZMotionController",
    "XYZPath",
    "XYZVertex",
    "compile_blended_profile",
    "compile_xyz_path",
    "create_blended_xyz_profile",
    "load_axis_calibrations_from_toml",
    "save_stepper_trajectory",
]
