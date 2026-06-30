"""
Compile blended Cartesian CNC paths to stepper pulse-train data.

This module forms the bridge between ``python_robot.motion`` and
``pyberryplc-stepper``. The preferred workflow is:

1. define an XYZ path as Cartesian vertices,
2. build a :class:`python_robot.motion.cartesian_space.BlendedPoseVectorProfile`,
3. compile the profile pieces analytically to the JSON-ready format consumed by
   :class:`pyberryplc_stepper.controller.XYZMotionController`.

The motor calibration is loaded from the same TOML file used by
``XYZMotionController``. This keeps pin configuration, microstepping, axis pitch,
and axis direction in one place.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

try:  # pragma: no cover - Python 3.11+ path.
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 fallback.
    import tomli as tomllib

from pyberryplc_stepper.rotation_direction import RotationDirection
from python_robot.motion.cartesian_space import (
    BlendedPoseVectorProfile,
    PoseProfileSegment,
)


MICROSTEP_FACTORS = {
    "full": 1,
    "1/2": 2,
    "1/4": 4,
    "1/8": 8,
    "1/16": 16,
    "1/32": 32,
    "1/64": 64,
    "1/128": 128,
    "1/256": 256,
}
TIME_TOL = 1.0e-12


@dataclass(frozen=True)
class XYZVertex:
    """
    Cartesian vertex of a CNC path.

    Parameters
    ----------
    x:
        X-coordinate of the path vertex.
    y:
        Y-coordinate of the path vertex.
    z:
        Z-coordinate of the path vertex.
    """

    x: float
    y: float
    z: float

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "XYZVertex":
        """
        Create a vertex from a three-value sequence.

        Parameters
        ----------
        values:
            Sequence containing ``x``, ``y``, and ``z`` coordinates.

        Returns
        -------
        XYZVertex
            Vertex with float coordinates.
        """
        if len(values) != 3:
            raise ValueError("An XYZ vertex must contain exactly three values.")
        return cls(float(values[0]), float(values[1]), float(values[2]))

    def as_tuple(self) -> tuple[float, float, float]:
        """
        Return the vertex as an ``(x, y, z)`` tuple.

        Returns
        -------
        tuple[float, float, float]
            Coordinates of this vertex.
        """
        return self.x, self.y, self.z

    def to_pose_vector(self) -> tuple[float, float, float, float, float, float]:
        """
        Return this vertex as a six-dimensional pose vector.

        Returns
        -------
        tuple[float, float, float, float, float, float]
            Pose vector ``(x, y, z, 0, 0, 0)``. CNC paths only use translation;
            orientation is kept fixed.
        """
        return self.x, self.y, self.z, 0.0, 0.0, 0.0


@dataclass(frozen=True)
class XYZPath:
    """
    Cartesian CNC path defined by XYZ vertices.

    Parameters
    ----------
    vertices:
        Sequence of :class:`XYZVertex` objects or ``(x, y, z)`` sequences.
    """

    vertices: Sequence[XYZVertex | Sequence[float]]

    @property
    def xyz_vertices(self) -> tuple[XYZVertex, ...]:
        """
        Return normalized path vertices.

        Returns
        -------
        tuple[XYZVertex, ...]
            Vertices converted to :class:`XYZVertex` instances.
        """
        vertices = tuple(
            vertex
            if isinstance(vertex, XYZVertex)
            else XYZVertex.from_sequence(vertex)
            for vertex in self.vertices
        )
        if len(vertices) < 2:
            raise ValueError("At least two XYZ vertices are required.")
        return vertices

    @property
    def points(self) -> np.ndarray:
        """
        Return the path vertices as an ``(n, 3)`` array.

        Returns
        -------
        np.ndarray
            XYZ coordinates of all path vertices.
        """
        return np.asarray(
            [vertex.as_tuple() for vertex in self.xyz_vertices],
            dtype=float,
        )

    @property
    def pose_vectors(self) -> np.ndarray:
        """
        Return path vertices as six-dimensional pose vectors.

        Returns
        -------
        np.ndarray
            Array with rows ``(x, y, z, 0, 0, 0)``.
        """
        return np.asarray(
            [vertex.to_pose_vector() for vertex in self.xyz_vertices],
            dtype=float,
        )

    @property
    def segment_distances(self) -> np.ndarray:
        """
        Return Euclidean distances between successive vertices.

        Returns
        -------
        np.ndarray
            One distance per Cartesian path segment.
        """
        return np.linalg.norm(np.diff(self.points, axis=0), axis=1)

    def segment_durations(self, feed_rate: float) -> tuple[float, ...]:
        """
        Calculate segment durations from a constant Cartesian feed rate.

        Parameters
        ----------
        feed_rate:
            Cartesian feed rate in path coordinate units per second.

        Returns
        -------
        tuple[float, ...]
            Duration of every path segment in seconds.
        """
        if feed_rate <= 0.0:
            raise ValueError("feed_rate must be greater than zero.")

        distances = self.segment_distances
        if np.any(distances <= 0.0):
            raise ValueError("Successive XYZ vertices must not be identical.")

        return tuple(float(distance / feed_rate) for distance in distances)


@dataclass(frozen=True)
class CompiledXYZTrajectory:
    """
    Result of compiling an XYZ path for ``XYZMotionController``.

    Parameters
    ----------
    profile:
        Blended pose-vector profile used for the CNC path.
    stepper_trajectory:
        JSON-ready list of motion segments for
        ``XYZMotionController.load_trajectory()``.
    """

    profile: BlendedPoseVectorProfile
    stepper_trajectory: list[dict[str, list]]

    def save(self, filepath: str | Path) -> None:
        """
        Save the compiled stepper trajectory as JSON.

        Parameters
        ----------
        filepath:
            Destination JSON file.
        """
        save_stepper_trajectory(filepath, self.stepper_trajectory)


@dataclass
class AxisCalibration:
    """
    Conversion data for one linear CNC axis.

    Parameters
    ----------
    travel_per_rev:
        Linear travel distance of the axis for one motor revolution. The unit
        must match the path coordinate unit.
    full_steps_per_rev:
        Number of full motor steps in one revolution before microstepping is
        applied.
    microstep_factor:
        Number of microsteps per full step.
    positive_direction:
        Motor direction that moves the axis in the positive coordinate
        direction.
    step_width:
        STEP pulse width in seconds. This value is subtracted from each
        interval between successive step times because the pulse itself consumes
        part of the interval.
    """

    travel_per_rev: float
    full_steps_per_rev: int = 200
    microstep_factor: int = 1
    positive_direction: RotationDirection | str = RotationDirection.CCW
    step_width: float = 20e-6

    def __post_init__(self) -> None:
        """
        Validate calibration values and normalize the positive direction.
        """
        if self.travel_per_rev <= 0.0:
            raise ValueError("travel_per_rev must be greater than zero.")
        if self.full_steps_per_rev <= 0:
            raise ValueError("full_steps_per_rev must be greater than zero.")
        if self.microstep_factor <= 0:
            raise ValueError("microstep_factor must be greater than zero.")
        if self.step_width < 0.0:
            raise ValueError("step_width cannot be negative.")
        self.positive_direction = RotationDirection.from_value(
            self.positive_direction
        )

    @classmethod
    def from_pitch(
        cls,
        pitch: float,
        *,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1,
        positive_direction: RotationDirection | str = RotationDirection.CCW,
        step_width: float = 20e-6,
    ) -> "AxisCalibration":
        """
        Create calibration data from an axis pitch.

        Parameters
        ----------
        pitch:
            Axis pitch in motor revolutions per path coordinate unit.
        full_steps_per_rev:
            Number of full motor steps per revolution.
        microstep_factor:
            Number of microsteps per full step.
        positive_direction:
            Motor direction corresponding to positive axis movement.
        step_width:
            STEP pulse width in seconds.

        Returns
        -------
        AxisCalibration
            Calibration data equivalent to the supplied pitch.
        """
        if pitch <= 0.0:
            raise ValueError("pitch must be greater than zero.")
        return cls(
            travel_per_rev=1.0 / pitch,
            full_steps_per_rev=full_steps_per_rev,
            microstep_factor=microstep_factor,
            positive_direction=positive_direction,
            step_width=step_width,
        )

    @property
    def pitch(self) -> float:
        """
        Return the axis pitch in revolutions per path coordinate unit.

        Returns
        -------
        float
            Motor revolutions per coordinate unit.
        """
        return 1.0 / self.travel_per_rev

    @property
    def steps_per_unit(self) -> float:
        """
        Return the number of microsteps for one coordinate unit.

        Returns
        -------
        float
            Microsteps per path coordinate unit.
        """
        return self.full_steps_per_rev * self.microstep_factor * self.pitch

    @property
    def unit_per_step(self) -> float:
        """
        Return the linear travel represented by one microstep.

        Returns
        -------
        float
            Coordinate units per microstep.
        """
        return 1.0 / self.steps_per_unit

    def direction_for_delta(self, delta: float) -> RotationDirection:
        """
        Return the motor direction for a signed axis displacement.

        Parameters
        ----------
        delta:
            Signed displacement in the axis coordinate system.

        Returns
        -------
        RotationDirection
            Motor direction for the displacement.
        """
        positive_direction = RotationDirection.from_value(
            self.positive_direction
        )
        if delta >= 0.0:
            return positive_direction
        return ~positive_direction


def load_axis_calibrations_from_toml(
    filepath: str | Path,
    axes: Sequence[str] = ("x", "y", "z"),
    *,
    step_width: float = 20e-6,
) -> dict[str, AxisCalibration]:
    """
    Load CNC axis calibrations from a motor configuration TOML file.

    Parameters
    ----------
    filepath:
        TOML file used by ``XYZMotionController``.
    axes:
        Axis names to load. Valid values are ``"x"``, ``"y"``, and ``"z"``.
    step_width:
        STEP pulse width in seconds.

    Returns
    -------
    dict[str, AxisCalibration]
        Calibration data keyed by axis name.
    """
    _axis_indices(axes)
    config = _load_toml(filepath)
    calibrations: dict[str, AxisCalibration] = {}

    for axis in axes:
        section_name = f"{axis}_motor"
        motor_config: dict = config.get(section_name, dict())
        if motor_config is None:
            continue

        pitch = float(_required_key(motor_config, "pitch", section_name))
        rdir_ref = _required_key(motor_config, "rdir_ref", section_name)
        microstepping = _required_key(
            motor_config,
            "microstepping",
            section_name,
        )
        if not isinstance(microstepping, Mapping):
            raise ValueError(f"{section_name}.microstepping must be a table.")

        resolution = str(
            _required_key(microstepping, "resolution", f"{section_name}.microstepping")
        )
        full_steps_per_rev = int(
            _required_key(
                microstepping,
                "full_steps_per_rev",
                f"{section_name}.microstepping",
            )
        )

        calibrations[axis] = AxisCalibration.from_pitch(
            pitch=pitch,
            full_steps_per_rev=full_steps_per_rev,
            microstep_factor=_microstep_factor_from_resolution(resolution),
            positive_direction=str(rdir_ref),
            step_width=step_width,
        )

    return calibrations


def create_blended_xyz_profile(
    vertices: Sequence[XYZVertex | Sequence[float]],
    *,
    dt_segments: Sequence[float] | None = None,
    feed_rate: float | None = None,
    dt_blends: float | Sequence[float] = 0.0,
) -> BlendedPoseVectorProfile:
    """
    Create a blended pose-vector profile from XYZ path vertices.

    Parameters
    ----------
    vertices:
        Cartesian path vertices.
    dt_segments:
        Optional duration of each path segment in seconds.
    feed_rate:
        Optional Cartesian feed rate. Used to derive ``dt_segments`` when
        explicit segment durations are not supplied.
    dt_blends:
        Blend time at each path point, or one value applied to all path points.

    Returns
    -------
    BlendedPoseVectorProfile
        Profile that can be compiled to stepper pulse trains.
    """
    path = XYZPath(vertices)
    if dt_segments is None:
        if feed_rate is None:
            raise ValueError("Either dt_segments or feed_rate must be provided.")
        dt_segments = path.segment_durations(feed_rate)
    elif feed_rate is not None:
        raise ValueError("Specify either dt_segments or feed_rate, not both.")

    return BlendedPoseVectorProfile(
        pose_vectors=path.pose_vectors,
        dt_segments=dt_segments,
        dt_blends=dt_blends,
    )


def compile_xyz_path(
    vertices: Sequence[XYZVertex | Sequence[float]],
    motor_config_filepath: str | Path,
    *,
    dt_segments: Sequence[float] | None = None,
    feed_rate: float | None = None,
    dt_blends: float | Sequence[float] = 0.0,
    axes: Sequence[str] = ("x", "y", "z"),
    include_stationary_axes: bool = True,
) -> CompiledXYZTrajectory:
    """
    Compile an XYZ vertex path using a ``XYZMotionController`` motor config.

    Parameters
    ----------
    vertices:
        Cartesian path vertices.
    motor_config_filepath:
        TOML file used by ``XYZMotionController``.
    dt_segments:
        Optional segment durations in seconds.
    feed_rate:
        Optional Cartesian feed rate used to derive segment durations.
    dt_blends:
        Blend time at each path point, or one value applied to all path points.
    axes:
        Axis names to compile.
    include_stationary_axes:
        If True, include axes with no motion in every JSON segment.

    Returns
    -------
    CompiledXYZTrajectory
        Profile and JSON-ready stepper trajectory.
    """
    calibrations = load_axis_calibrations_from_toml(
        motor_config_filepath,
        axes=axes,
    )
    profile = create_blended_xyz_profile(
        vertices,
        dt_segments=dt_segments,
        feed_rate=feed_rate,
        dt_blends=dt_blends,
    )
    stepper_trajectory = compile_blended_profile(
        profile,
        calibrations,
        axes=axes,
        include_stationary_axes=include_stationary_axes,
    )
    return CompiledXYZTrajectory(
        profile=profile,
        stepper_trajectory=stepper_trajectory,
    )


def compile_blended_profile(
    profile: BlendedPoseVectorProfile,
    calibrations: Mapping[str, AxisCalibration],
    *,
    axes: Sequence[str] = ("x", "y", "z"),
    include_stationary_axes: bool = True,
) -> list[dict[str, list]]:
    """
    Compile a blended Cartesian profile to stepper-controller JSON data.

    Parameters
    ----------
    profile:
        Blended pose-vector profile. The first three pose-vector components are
        interpreted as X, Y, and Z.
    calibrations:
        Per-axis calibration settings.
    axes:
        Axis names to compile.
    include_stationary_axes:
        If True, include axes with no movement as empty pulse trains.

    Returns
    -------
    list[dict[str, list]]
        JSON-ready trajectory segments for
        ``XYZMotionController.load_trajectory(trajectory=...)``.
    """
    axis_indices = _axis_indices(axes)
    active_axes = tuple(axis for axis in axes if axis in calibrations)
    if not active_axes:
        raise ValueError("No calibrated axes are available for compilation.")

    trajectory: list[dict[str, list]] = []
    for piece in profile.pieces:
        for tau_start, tau_stop in _piece_monotonic_intervals(
            piece,
            active_axes,
            axis_indices,
        ):
            compiled_segment: dict[str, list] = {}

            for axis in active_axes:
                delays, direction = _compile_axis_piece_interval(
                    piece=piece,
                    axis_index=axis_indices[axis],
                    tau_start=tau_start,
                    tau_stop=tau_stop,
                    calibration=calibrations[axis],
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
    """
    Save compiled ``XYZMotionController`` trajectory data as JSON.

    Parameters
    ----------
    filepath:
        Output JSON file path.
    trajectory:
        JSON-ready trajectory.
    """
    with Path(filepath).open("w", encoding="utf-8") as fh:
        json.dump(trajectory, fh, indent=2)


def _load_toml(filepath: str | Path) -> dict[str, Any]:
    """
    Load a TOML file.

    Parameters
    ----------
    filepath:
        Path to a TOML file.

    Returns
    -------
    dict[str, Any]
        Parsed TOML content.
    """
    with Path(filepath).open("rb") as fh:
        return tomllib.load(fh)


def _required_key(
    mapping: Mapping[str, Any],
    key: str,
    section_name: str,
) -> Any:
    """
    Return a required key from a mapping.

    Parameters
    ----------
    mapping:
        Mapping to inspect.
    key:
        Required key.
    section_name:
        Name used in the error message.

    Returns
    -------
    Any
        Value stored under ``key``.
    """
    try:
        return mapping[key]
    except KeyError as exc:
        raise KeyError(f"Missing required key: {section_name}.{key}") from exc


def _microstep_factor_from_resolution(resolution: str) -> int:
    """
    Convert a microstep resolution string to its numeric factor.

    Parameters
    ----------
    resolution:
        Resolution string such as ``"full"`` or ``"1/16"``.

    Returns
    -------
    int
        Microstep factor.
    """
    try:
        return MICROSTEP_FACTORS[resolution]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported microstep resolution: {resolution!r}."
        ) from exc


def _piece_monotonic_intervals(
    piece: PoseProfileSegment,
    axes: Sequence[str],
    axis_indices: Mapping[str, int],
) -> list[tuple[float, float]]:
    """
    Split a profile piece where any configured axis changes direction.

    Parameters
    ----------
    piece:
        Profile piece whose coordinate functions are quadratic in local time.
    axes:
        Active axis names.
    axis_indices:
        Mapping from axis names to pose-vector indices.

    Returns
    -------
    list[tuple[float, float]]
        Local-time intervals within the piece.
    """
    bounds = [0.0, float(piece.dt)]
    for axis in axes:
        axis_index = axis_indices[axis]
        acceleration = float(piece.a[axis_index])
        if abs(acceleration) <= TIME_TOL:
            continue

        velocity = float(piece.v0[axis_index])
        tau_zero = -velocity / acceleration
        if TIME_TOL < tau_zero < piece.dt - TIME_TOL:
            bounds.append(float(tau_zero))

    unique_bounds = sorted(set(round(bound, 15) for bound in bounds))
    return [
        (start, stop)
        for start, stop in zip(unique_bounds[:-1], unique_bounds[1:])
        if stop - start > TIME_TOL
    ]


def _compile_axis_piece_interval(
    piece: PoseProfileSegment,
    axis_index: int,
    tau_start: float,
    tau_stop: float,
    calibration: AxisCalibration,
) -> tuple[list[float], RotationDirection]:
    """
    Compile one monotonic profile-piece interval for one axis.

    Parameters
    ----------
    piece:
        Profile piece to compile.
    axis_index:
        Pose-vector index of the axis.
    tau_start:
        Start time within the piece.
    tau_stop:
        Stop time within the piece.
    calibration:
        Axis calibration.

    Returns
    -------
    tuple[list[float], RotationDirection]
        STEP delays and axis direction.
    """
    q_start = _axis_position_at(piece, axis_index, tau_start)
    q_stop = _axis_position_at(piece, axis_index, tau_stop)
    delta = q_stop - q_start
    direction = calibration.direction_for_delta(delta)
    num_steps = int(round(abs(delta) * calibration.steps_per_unit))
    if num_steps == 0:
        return [], direction

    step_positions = np.linspace(q_start, q_stop, num_steps + 1)
    tau_values = [tau_start]
    for step_position in step_positions[1:-1]:
        tau_values.append(
            _solve_tau_for_axis_position(
                piece,
                axis_index,
                float(step_position),
                tau_start,
                tau_stop,
            )
        )
    tau_values.append(tau_stop)

    delays = np.maximum(0.0, np.diff(tau_values) - calibration.step_width)
    return delays.tolist(), direction


def _axis_position_at(
    piece: PoseProfileSegment,
    axis_index: int,
    tau: float,
) -> float:
    """
    Return one axis position at local piece time ``tau``.

    Parameters
    ----------
    piece:
        Profile piece.
    axis_index:
        Pose-vector axis index.
    tau:
        Local time within the piece.

    Returns
    -------
    float
        Axis coordinate.
    """
    return float(
        piece.x0[axis_index]
        + piece.v0[axis_index] * tau
        + 0.5 * piece.a[axis_index] * tau**2
    )


def _solve_tau_for_axis_position(
    piece: PoseProfileSegment,
    axis_index: int,
    position: float,
    tau_start: float,
    tau_stop: float,
) -> float:
    """
    Solve the local time at which an axis reaches a position.

    Parameters
    ----------
    piece:
        Profile piece.
    axis_index:
        Pose-vector axis index.
    position:
        Target axis position.
    tau_start:
        Lower local-time bound.
    tau_stop:
        Upper local-time bound.

    Returns
    -------
    float
        Local time inside ``[tau_start, tau_stop]``.
    """
    x0 = float(piece.x0[axis_index])
    v0 = float(piece.v0[axis_index])
    a = float(piece.a[axis_index])

    if abs(a) <= TIME_TOL:
        if abs(v0) <= TIME_TOL:
            return tau_start
        tau = (position - x0) / v0
        return _clamp(tau, tau_start, tau_stop)

    discriminant = v0**2 - 2.0 * a * (x0 - position)
    if discriminant < 0.0 and abs(discriminant) <= TIME_TOL:
        discriminant = 0.0
    if discriminant < 0.0:
        raise ValueError("Axis position is outside the profile piece.")

    sqrt_discriminant = float(np.sqrt(discriminant))
    candidates = [
        (-v0 - sqrt_discriminant) / a,
        (-v0 + sqrt_discriminant) / a,
    ]
    valid = [
        tau
        for tau in candidates
        if tau_start - TIME_TOL <= tau <= tau_stop + TIME_TOL
    ]
    if not valid:
        closest = min(
            candidates,
            key=lambda tau_: min(abs(tau_ - tau_start), abs(tau_ - tau_stop)),
        )
        return _clamp(closest, tau_start, tau_stop)

    return _clamp(valid[0], tau_start, tau_stop)


def _clamp(value: float, lower: float, upper: float) -> float:
    """
    Clamp a value to a closed interval.

    Parameters
    ----------
    value:
        Value to clamp.
    lower:
        Lower bound.
    upper:
        Upper bound.

    Returns
    -------
    float
        Clamped value.
    """
    return min(max(float(value), lower), upper)


def _axis_indices(axes: Sequence[str]) -> dict[str, int]:
    """
    Return coordinate-column indices for the requested axes.

    Parameters
    ----------
    axes:
        Axis identifiers to map.

    Returns
    -------
    dict[str, int]
        Mapping from axis identifier to XYZ column index.
    """
    valid_indices = {"x": 0, "y": 1, "z": 2}
    unknown = set(axes) - set(valid_indices)
    if unknown:
        raise ValueError(f"Unknown CNC axes: {sorted(unknown)}.")
    return {axis: valid_indices[axis] for axis in axes}

