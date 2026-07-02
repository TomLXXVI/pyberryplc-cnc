"""
CNC-oriented XYZ motion controller built on pyberryplc-stepper.

This module coordinates up to three stepper axes by composing the generic
:class:`pyberryplc_stepper.controller.MotorController` with CNC-specific
X/Y/Z motor configuration and trajectory loading.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Iterator, cast

try:  # pragma: no cover - Python 3.11+ path.
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 fallback.
    import tomli as tomllib

from automation_motion.profiles_1D.point_to_point import TriPhaseMotionProfile
from pyberryplc_stepper.controller import MotorController
from pyberryplc_stepper.driver.base import PinConfig, StepperMotor
from pyberryplc_stepper.driver.tmc2208 import TMC2208StepperMotor
from pyberryplc_stepper.rotation_direction import RotationDirection

if TYPE_CHECKING:
    from pyberryplc.core import TAbstractPLC


@dataclass
class MotorStatus:
    """
    Status snapshot for a single motor controller.
    """

    ready: bool = True
    motions_finished: bool = True
    travel_time: float = float("nan")
    jog_mode_active: bool = False


@dataclass
class MotionStatus:
    """
    Combined status snapshot for a multi-axis motion controller.

    Missing axes can be left as ``None``. The aggregate properties only consider
    motor statuses that are present.
    """

    x: MotorStatus | None = None
    y: MotorStatus | None = None
    z: MotorStatus | None = None

    def __post_init__(self) -> None:
        """
        Store the configured axis statuses for aggregate status checks.
        """
        self.motor_statuses = tuple(
            s for s in (self.x, self.y, self.z) if s is not None
        )

    @property
    def all_ready(self) -> bool:
        """
        Whether all configured motors are ready.
        """
        if all([m.ready for m in self.motor_statuses]):
            return True
        return False

    @property
    def all_finished(self) -> bool:
        """
        Whether all configured motors have finished their trajectories.
        """
        if all([m.motions_finished for m in self.motor_statuses]):
            return True
        return False

    @property
    def travel_time(self) -> float:
        """
        Maximum travel time reported by the configured motors.
        """
        travel_time = max([m.travel_time for m in self.motor_statuses])
        return travel_time

    @property
    def jog_mode_active(self) -> bool:
        """
        Whether any configured motor is currently in jog mode.
        """
        if any([m.jog_mode_active for m in self.motor_statuses]):
            return True
        return False


class XYZMotionController:
    """
    Multi-axis controller for up to three TMC2208-driven stepper motors.

    The controller creates one :class:`MotorController` per configured axis and
    coordinates X, Y, and Z motion by forwarding precomputed step-pulse delays
    to the individual motor processes. Motor configuration is loaded from a TOML
    file.
    """

    def __init__(
        self,
        master: TAbstractPLC,
        logger: logging.Logger,
        config_filepath: str = "motor_config.toml",
        jog_mode_profile: TriPhaseMotionProfile | None = None
    ) -> None:
        """
        Create a multi-axis motion controller.

        Parameters
        ----------
        master:
            PLC instance that owns this controller.
        config_filepath:
            Path to the TOML file with motor configuration settings.
        logger:
            Logger used by the motor controllers.
        jog_mode_profile:
            Tri-phase profile used for jog-mode acceleration and deceleration.
        """
        self.master = master
        self.motor_class: type[StepperMotor] = TMC2208StepperMotor
        self.config_filepath = config_filepath
        self.logger = logger
        self.jog_mode_profile = jog_mode_profile

        self.x_motor_cfg: dict | None = None
        self.y_motor_cfg: dict | None = None
        self.z_motor_cfg: dict | None = None

        self.x_motor_ctrl: MotorController = None  # type: ignore
        self.y_motor_ctrl: MotorController = None  # type: ignore
        self.z_motor_ctrl: MotorController = None  # type: ignore
        self.motor_ctrls: tuple[MotorController, ...] = None  # type: ignore

        TSegment = dict[str, tuple[list[float], RotationDirection]]
        TTrajectory = list[TSegment]
        self.trajectory: TTrajectory | None = None
        self.segments: Iterator[tuple[int, TSegment]] | None = None

        self._load_motor_configurations()
        self._create_motor_controllers()

    def _load_motor_configurations(self) -> None:
        """
        Load per-axis motor configuration from the TOML file.

        Missing axes are stored as ``None`` and are skipped when motor
        controllers are created.
        """
        with Path(self.config_filepath).open("rb") as f:
            config = tomllib.load(f)
            self.x_motor_cfg = config.get("x_motor")
            self.y_motor_cfg = config.get("y_motor")
            self.z_motor_cfg = config.get("z_motor")

    def _create_motor_controller(
        self,
        axis_name: str,
        cfg: dict
    ) -> MotorController:
        """
        Create a :class:`MotorController` for one configured axis.

        Parameters
        ----------
        axis_name:
            Human-readable axis name used in logs and status messages.
        cfg:
            Motor configuration dictionary loaded from TOML.
        """
        step_pin_ID: int = cast(int, cfg.get("step_pin_ID"))
        dir_pin_ID: int = cast(int, cfg.get("dir_pin_ID"))
        comm_port: str = cast(str, cfg.get("comm_port"))

        motor_controller = MotorController(
            motor_name=axis_name,
            motor_class=self.motor_class,
            pin_config=PinConfig(
                step_pin=step_pin_ID,
                dir_pin=dir_pin_ID,
                use_pigpio=True
            ),
            cfg_callback=lambda motor: self._config_motor(motor, cfg),
            comm_port=comm_port,
            logger=self.logger,
            jog_mode_profile=self.jog_mode_profile,
        )
        return motor_controller

    @staticmethod
    def _config_motor(motor: TMC2208StepperMotor, cfg: dict) -> None:
        """
        Configure a TMC2208 motor inside its child process.

        This callback is passed to :class:`SPMCProcess` through
        :class:`MotorController` and runs after the concrete motor has been
        created in the child process.
        """
        high_sensitivity = cfg.get("high_sensitivity", False)
        resolution = cfg["microstepping"]["resolution"]
        full_steps_per_rev = cfg["microstepping"]["full_steps_per_rev"]
        run_current_pct = cfg["current"]["run_current_pct"]
        hold_current_pct = cfg["current"]["hold_current_pct"]

        motor.enable(
            high_sensitivity=high_sensitivity
        )
        motor.configure_microstepping(
            resolution=resolution,
            ms_pins=None,
            full_steps_per_rev=full_steps_per_rev
        )
        motor.set_current_via_uart(
            run_current_pct=run_current_pct,
            hold_current_pct=hold_current_pct
        )

    def _create_motor_controllers(self) -> None:
        """
        Create motor controllers for all configured axes.
        """
        motor_ctrls: list[MotorController] = []

        if self.x_motor_cfg is not None:
            self.x_motor_ctrl = self._create_motor_controller(
                "X-axis",
                self.x_motor_cfg
            )
            motor_ctrls.append(self.x_motor_ctrl)

        if self.y_motor_cfg is not None:
            self.y_motor_ctrl = self._create_motor_controller(
                "Y-axis",
                self.y_motor_cfg
            )
            motor_ctrls.append(self.y_motor_ctrl)

        if self.z_motor_cfg is not None:
            self.z_motor_ctrl = self._create_motor_controller(
                "Z-axis",
                self.z_motor_cfg
            )
            motor_ctrls.append(self.z_motor_ctrl)

        self.motor_ctrls = tuple(motor_ctrls)

    def enable(self) -> None:
        """
        Start all configured motor controller processes.
        """
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.enable()

    def disable(self) -> None:
        """
        Shut down and join all configured motor controller processes.
        """
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.shutdown()

        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.disable()

    def get_motion_status(self) -> MotionStatus:
        """
        Return the current per-axis and aggregate motion status.
        """
        def get_motor_status(motor_ctrl_: MotorController) -> MotorStatus:
            return MotorStatus(
                ready=bool(motor_ctrl_.motor_ready),
                motions_finished=bool(motor_ctrl_.motions_finished),
                travel_time=motor_ctrl_.travel_time,
                jog_mode_active=motor_ctrl_.jog_mode_active
            )

        kwargs = {}
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.update_status()
            motor_status = get_motor_status(motor_ctrl)
            if motor_ctrl.motor_name == "X-axis":
                kwargs["x"] = motor_status
            if motor_ctrl.motor_name == "Y-axis":
                kwargs["y"] = motor_status
            if motor_ctrl.motor_name == "Z-axis":
                kwargs["z"] = motor_status
        return MotionStatus(**kwargs)

    def load_trajectory(
        self,
        filepath: str | None,
        trajectory: list | None = None
    ) -> int:
        """
        Load a precomputed trajectory from a JSON file or direct object.

        A trajectory is a list of segment dictionaries. Each segment contains
        optional ``"x"``, ``"y"``, and ``"z"`` entries with precomputed
        ``(delays, direction)`` data for that axis.

        Parameters
        ----------
        filepath:
            Path to a trajectory JSON file. If ``None``, ``trajectory`` is used
            directly.
        trajectory:
            In-memory trajectory data.

        Returns
        -------
        int
            Number of trajectory segments, or ``-1`` when no valid trajectory is
            available.
        """
        if filepath is not None:
            with open(filepath, "r") as f:
                self.trajectory = json.load(f)
        else:
            self.trajectory = trajectory

        if self.trajectory:
            # Assign the number of segments in the trajectory to each axis motor
            # controller if this is needed.
            num_segments = len(self.trajectory)
            segment1 = self.trajectory[0]
            if "x" in segment1.keys():
                self.x_motor_ctrl.set_init_state_trajectory(num_segments)
            if "y" in segment1.keys():
                self.y_motor_ctrl.set_init_state_trajectory(num_segments)
            if "z" in segment1.keys():
                self.z_motor_ctrl.set_init_state_trajectory(num_segments)

            # Create a generator to iterate on command over the segments in the
            # trajectory (see method `move()`).
            self.segments = ((i, s) for i, s in enumerate(self.trajectory))

            return num_segments
        return -1

    @staticmethod
    def get_rotation_direction(rdir: str) -> RotationDirection:
        """
        Convert a serialized direction string to :class:`RotationDirection`.
        """
        match rdir:
            case "counterclockwise" | "ccw":
                return RotationDirection.CCW
            case "clockwise" | "cw":
                return RotationDirection.CW
        raise ValueError(f"Invalid rotation direction: {rdir}")

    def _send_stepper_signals(
        self,
        segment: dict[str, tuple],
        pseudo: bool = False
    ) -> None:
        """
        Send one trajectory segment to the relevant axis controllers.
        """
        if self.x_motor_ctrl is not None and "x" in segment.keys():
            self.x_motor_ctrl.move(
                step_pulses=segment["x"][0],
                rdir=self.get_rotation_direction(segment["x"][1]),
                pseudo=pseudo
            )

        if self.y_motor_ctrl is not None and "y" in segment.keys():
            self.y_motor_ctrl.move(
                step_pulses=segment["y"][0],
                rdir=self.get_rotation_direction(segment["y"][1]),
                pseudo=pseudo
            )

        if self.z_motor_ctrl is not None and "z" in segment.keys():
            self.z_motor_ctrl.move(
                step_pulses=segment["z"][0],
                rdir=self.get_rotation_direction(segment["z"][1]),
                pseudo=pseudo
            )

    def move(self, pseudo: bool = False) -> int:
        """
        Send the next trajectory segment to the motor controllers.

        Returns
        -------
        int
            Segment index, or ``-1`` when the trajectory has finished.
        """
        if self.trajectory:
            try:
                i, segment = next(self.segments)  # type: ignore
            except StopIteration:
                return -1
            else:
                self._send_stepper_signals(segment, pseudo)
                return i
        else:
            raise AttributeError("No trajectory has been loaded yet.")

    def start_jog_mode(
        self,
        axis: str,
        rdir: RotationDirection = RotationDirection.CCW
    ) -> None:
        """
        Start jog mode for a single configured axis.

        Jog mode can only be started when all motors are at rest and the
        execution of a trajectory has been finished.

        Parameters
        ----------
        axis:
            Axis identifier: ``"x"``, ``"y"``, or ``"z"``.
        rdir:
            Rotation direction in jog mode.
        """
        motion = self.get_motion_status()
        if motion.all_ready and motion.all_finished:
            if axis == "x" and self.x_motor_ctrl is not None:
                motor_ctrl = self.x_motor_ctrl
            elif axis == "y" and self.y_motor_ctrl is not None:
                motor_ctrl = self.y_motor_ctrl
            elif axis == "z" and self.z_motor_ctrl is not None:
                motor_ctrl = self.z_motor_ctrl
            else:
                raise ValueError(f"Axis {axis} is undefined.")
            motor_ctrl.start_jog_mode(rdir)
        else:
            self.logger.warning("Jog mode cannot be started.")

    def stop_jog_mode(self, axis: str) -> None:
        """
        Stop jog mode for a single configured axis.

        Parameters
        ----------
        axis:
            Axis identifier: ``"x"``, ``"y"``, or ``"z"``.
        """
        motion_status = self.get_motion_status()
        if motion_status.jog_mode_active:
            if axis == "x" and self.x_motor_ctrl is not None:
                motor_ctrl = self.x_motor_ctrl
            elif axis == "y" and self.y_motor_ctrl is not None:
                motor_ctrl = self.y_motor_ctrl
            elif axis == "z" and self.z_motor_ctrl is not None:
                motor_ctrl = self.z_motor_ctrl
            else:
                raise ValueError(f"Axis {axis} is undefined.")
            motor_ctrl.stop_jog_mode()

    def set_jog_mode_profile(self, mp: TriPhaseMotionProfile) -> None:
        """
        Update the jog-mode profile for all configured motor controllers.
        """
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.set_jog_mode_profile(mp)
