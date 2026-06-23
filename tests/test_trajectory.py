import math

import numpy as np

from pyberryplc_cnc import (
    AxisCalibration,
    compile_xyz_samples,
    compile_xyz_stepper_trajectory,
)


class FakeScheme:
    """
    Minimal python_robot-like trajectory scheme for adapter tests.
    """

    def __init__(self, t_samples, trajectory_points):
        """
        Store trajectory samples as NumPy arrays.

        Parameters
        ----------
        t_samples:
            Raw time samples.
        trajectory_points:
            Raw XYZ trajectory samples.
        """
        self._t_samples = np.asarray(t_samples, dtype=float)
        self._trajectory_points = np.asarray(trajectory_points, dtype=float)

    @property
    def time_samples(self):
        """
        Return test trajectory time samples.
        """
        return self._t_samples

    @property
    def trajectory_points(self):
        """
        Return test trajectory points.
        """
        return self._trajectory_points


def test_compile_linear_axis_to_delays_and_direction():
    """
    Compile a one-axis linear move to evenly spaced pulse delays.
    """
    trajectory = compile_xyz_samples(
        t_samples=[0.0, 1.0],
        trajectory_points=[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ],
        calibrations={
            "x": AxisCalibration(
                travel_per_rev=1.0,
                full_steps_per_rev=10,
                microstep_factor=1,
                positive_direction="counterclockwise",
            )
        },
        axes=("x",),
    )

    assert len(trajectory) == 1
    delays, direction = trajectory[0]["x"]
    assert len(delays) == 10
    assert direction == "counterclockwise"
    assert all(math.isclose(delay, 0.1 - 20e-6) for delay in delays)


def test_compile_splits_when_axis_direction_changes():
    """
    Split a trajectory when an axis reverses direction.
    """
    trajectory = compile_xyz_samples(
        t_samples=[0.0, 1.0, 2.0],
        trajectory_points=[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        calibrations={
            "x": AxisCalibration(
                travel_per_rev=1.0,
                full_steps_per_rev=10,
                positive_direction="counterclockwise",
            )
        },
        axes=("x",),
    )

    assert len(trajectory) == 2
    assert trajectory[0]["x"][1] == "counterclockwise"
    assert trajectory[1]["x"][1] == "clockwise"


def test_stationary_axes_are_included_by_default():
    """
    Include configured stationary axes as empty pulse trains by default.
    """
    trajectory = compile_xyz_samples(
        t_samples=[0.0, 1.0],
        trajectory_points=[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ],
        calibrations={
            "x": AxisCalibration(travel_per_rev=1.0, full_steps_per_rev=10),
            "y": AxisCalibration(travel_per_rev=1.0, full_steps_per_rev=10),
        },
        axes=("x", "y"),
    )

    assert "y" in trajectory[0]
    assert trajectory[0]["y"] == [[], "counterclockwise"]


def test_compile_accepts_python_robot_like_scheme_object():
    """
    Compile objects that expose the python_robot Cartesian scheme surface.
    """
    scheme = FakeScheme([0.0, 1.0], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    trajectory = compile_xyz_stepper_trajectory(
        scheme,
        {"x": AxisCalibration(travel_per_rev=1.0, full_steps_per_rev=10)},
        axes=("x",),
    )
    assert len(trajectory[0]["x"][0]) == 10
