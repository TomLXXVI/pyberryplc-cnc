from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Literal, Protocol, Sequence, Mapping

import numpy as np

Direction = Literal["counterclockwise", "clockwise"]

_CCW = "counterclockwise"
_CW = "clockwise"
_DIRECTION_ALIASES = {"counterclockwise": _CCW, "ccw": _CCW, "clockwise": _CW, "cw": _CW}


class XYZTrajectoryScheme(Protocol):
    """Small structural protocol implemented by python_robot Cartesian schemes."""

    @property
    def time_samples(self) -> np.ndarray:
        ...

    @property
    def trajectory_points(self) -> np.ndarray:
        ...


@dataclass
class AxisCalibration:
    """
    Conversion data for one linear CNC axis.

    travel_per_rev is expressed in the same length unit as the trajectory
    coordinates, for example meters per revolution or millimeters per revolution.
    """

    travel_per_rev: float
    full_steps_per_rev: int = 200
    microstep_factor: int = 1
    positive_direction: Direction | str = _CCW
    step_width: float = 20e-6

    def __post_init__(self) -> None:
        if self.travel_per_rev <= 0.0:
            raise ValueError("travel_per_rev must be greater than zero.")
        if self.full_steps_per_rev <= 0:
            raise ValueError("full_steps_per_rev must be greater than zero.")
        if self.microstep_factor <= 0:
            raise ValueError("microstep_factor must be greater than zero.")
        if self.step_width < 0.0:
            raise ValueError("step_width cannot be negative.")
        self.positive_direction = _normalize_direction(self.positive_direction)

    @property
    def steps_per_unit(self) -> float:
        return self.full_steps_per_rev * self.microstep_factor / self.travel_per_rev

    @property
    def unit_per_step(self) -> float:
        return 1.0 / self.steps_per_unit

    def direction_for_delta(self, delta: float) -> Direction:
        if delta >= 0.0:
            return self.positive_direction
        return _opposite_direction(self.positive_direction)


def compile_xyz_stepper_trajectory(
    scheme: XYZTrajectoryScheme,
    calibrations: Mapping[str, AxisCalibration],
    *,
    axes: Sequence[str] = ("x", "y", "z"),
    include_stationary_axes: bool = True,
) -> list[dict[str, list]]:
    """
    Compile a python_robot Cartesian trajectory scheme to XYZMotionController data.
    """
    return compile_xyz_samples(
        t_samples=scheme.time_samples,
        trajectory_points=scheme.trajectory_points,
        calibrations=calibrations,
        axes=axes,
        include_stationary_axes=include_stationary_axes,
    )


def compile_xyz_samples(
    t_samples: Sequence[float],
    trajectory_points: Sequence[Sequence[float]],
    calibrations: Mapping[str, AxisCalibration],
    *,
    axes: Sequence[str] = ("x", "y", "z"),
    include_stationary_axes: bool = True,
) -> list[dict[str, list]]:
    """
    Convert sampled XYZ positions to JSON-ready per-axis pulse trains.
    """
    t_arr, p_arr = _validate_samples(t_samples, trajectory_points)
    axis_indices = _axis_indices(axes)
    active_axes = tuple(axis for axis in axes if axis in calibrations)
    boundaries = _split_boundaries_on_direction_changes(p_arr, axis_indices)

    trajectory: list[dict[str, list]] = []
    for start, stop in zip(boundaries[:-1], boundaries[1:]):
        segment_t = t_arr[start : stop + 1]
        segment_points = p_arr[start : stop + 1]
        compiled_segment: dict[str, list] = {}

        for axis in active_axes:
            axis_index = axis_indices[axis]
            delays, direction = _compile_axis_segment(
                segment_t,
                segment_points[:, axis_index],
                calibrations[axis],
            )
            if delays or include_stationary_axes:
                compiled_segment[axis] = [delays, direction]

        if compiled_segment:
            trajectory.append(compiled_segment)

    return trajectory


def save_stepper_trajectory(
    filepath: str | Path,
    trajectory: list[dict[str, list]],
) -> None:
    """Save compiled XYZMotionController trajectory data as JSON."""
    with Path(filepath).open("w", encoding="utf-8") as fh:
        json.dump(trajectory, fh, indent=2)


def _validate_samples(
    t_samples: Sequence[float],
    trajectory_points: Sequence[Sequence[float]],
) -> tuple[np.ndarray, np.ndarray]:
    t_arr = np.asarray(t_samples, dtype=float)
    p_arr = np.asarray(trajectory_points, dtype=float)

    if t_arr.ndim != 1:
        raise ValueError("t_samples must be a 1D sequence.")
    if p_arr.ndim != 2 or p_arr.shape[1] < 3:
        raise ValueError("trajectory_points must have shape (n, 3) or wider.")
    if len(t_arr) != len(p_arr):
        raise ValueError("t_samples and trajectory_points must have the same length.")
    if len(t_arr) < 2:
        raise ValueError("At least two trajectory samples are required.")
    if np.any(np.diff(t_arr) <= 0.0):
        raise ValueError("t_samples must be strictly increasing.")

    return t_arr, p_arr[:, :3]


def _axis_indices(axes: Sequence[str]) -> dict[str, int]:
    valid_indices = {"x": 0, "y": 1, "z": 2}
    unknown = set(axes) - set(valid_indices)
    if unknown:
        raise ValueError(f"Unknown CNC axes: {sorted(unknown)}.")
    return {axis: valid_indices[axis] for axis in axes}


def _split_boundaries_on_direction_changes(
    points: np.ndarray,
    axis_indices: Mapping[str, int],
) -> list[int]:
    breaks: list[int] = []
    for axis_index in axis_indices.values():
        signs = _effective_displacement_signs(np.diff(points[:, axis_index]))
        flips = np.where(signs[1:] * signs[:-1] == -1)[0] + 1
        breaks.extend(int(index) for index in flips)

    boundaries = {0, len(points) - 1, *breaks}
    return sorted(boundaries)


def _effective_displacement_signs(displacements: np.ndarray) -> np.ndarray:
    signs = np.sign(displacements).astype(int)
    if len(signs) == 0:
        return signs

    for i in range(1, len(signs)):
        if signs[i] == 0:
            signs[i] = signs[i - 1]

    if signs[0] == 0:
        nonzero = np.flatnonzero(signs)
        if nonzero.size:
            signs[: nonzero[0]] = signs[nonzero[0]]

    return signs


def _compile_axis_segment(
    t_arr: np.ndarray,
    coords: np.ndarray,
    calibration: AxisCalibration,
) -> tuple[list[float], Direction]:
    delta = float(coords[-1] - coords[0])
    direction = calibration.direction_for_delta(delta)
    num_steps = int(round(abs(delta) * calibration.steps_per_unit))
    if num_steps == 0:
        return [], direction

    step_distance = calibration.unit_per_step
    sign = 1.0 if delta >= 0.0 else -1.0
    step_positions = coords[0] + sign * step_distance * np.arange(num_steps + 1)

    interp_coords = coords
    interp_t = t_arr
    if coords[-1] < coords[0]:
        interp_coords = coords[::-1]
        interp_t = t_arr[::-1]

    step_times = np.interp(step_positions, interp_coords, interp_t)
    delays = np.maximum(0.0, np.diff(step_times) - calibration.step_width)
    return delays.tolist(), direction


def _normalize_direction(direction: object) -> Direction:
    key = str(direction).lower()
    try:
        return _DIRECTION_ALIASES[key]  # type: ignore[return-value]
    except KeyError as exc:
        raise ValueError(
            "positive_direction must be 'counterclockwise', 'ccw', "
            "'clockwise', or 'cw'."
        ) from exc


def _opposite_direction(direction: Direction) -> Direction:
    if direction == _CCW:
        return _CW
    return _CCW
