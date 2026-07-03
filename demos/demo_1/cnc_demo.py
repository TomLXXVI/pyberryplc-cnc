"""
Keyboard-driven CNC PLC demo.

The demo has two operating modes:

- auto mode loads and executes ``stepper_trajectory.json``;
- manual mode jogs one configured axis while a keyboard key is held.

Generate the trajectory first by running ``create_trajectory.py``.
"""

from __future__ import annotations

from pathlib import Path

from pyberryplc.core import (
    AbstractPLC,
    EmergencyConfig,
    MemoryVariable,
    ThreePositionSwitch,
)
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.utils.log_utils import init_logger
from pyberryplc_stepper import RotationDirection, SCurvedProfile

from pyberryplc_cnc import MotionStatus, XYZMotionController


DEMO_DIR = Path(__file__).resolve().parent
MOTOR_CONFIG_FILEPATH = DEMO_DIR / "motor_config.toml"
TRAJECTORY_FILEPATH = DEMO_DIR / "stepper_trajectory.json"


class CNCPLC(AbstractPLC):
    """
    PLC application for the CNC demo.

    The main sequence only decides which operating mode is active. The detailed
    behaviour of auto mode and manual mode is delegated to two subroutine-style
    state machines: :class:`AutoMode` and :class:`ManualMode`.
    """

    def __init__(self) -> None:
        """
        Create the PLC, keyboard inputs, CNC controller, and mode subroutines.
        """
        self.key_input = KeyInput()
        self._add_keys()

        super().__init__(
            logger=init_logger(name="CNC-PLC"),
            emergency_config=EmergencyConfig(
                emergency_input=self.EmergencyButton,
                reset_input=self.ResetButton,
                emergency_nc_contact=False,
            ),
        )

        self.cnc_controller = self._setup_cnc_controller()
        self.cnc_status: MotionStatus | None = None

        self.ModeSwitch = ThreePositionSwitch(MemoryVariable())
        self.man_mode = ManualMode(self)
        self.aut_mode = AutoMode(self, trajectory_filepath=TRAJECTORY_FILEPATH)

        # Main sequence:
        # S0 = idle/mode selection, S1 = AutoMode active, S2 = ManualMode active.
        self.S0 = self.add_marker("S0")
        self.S1 = self.add_marker("S1")
        self.S2 = self.add_marker("S2")

    def _setup_cnc_controller(self) -> XYZMotionController:
        """
        Create the XYZ motion controller used by the PLC.

        Returns
        -------
        XYZMotionController
            Controller configured from the demo motor configuration file.
        """
        return XYZMotionController(
            master=self,
            logger=init_logger(name="CNC-CONTROLLER"),
            config_filepath=str(MOTOR_CONFIG_FILEPATH),
            jog_mode_profile=SCurvedProfile(
                ds_tot=720,
                a_max=1800,
                v_max=360,
                v_i=0,
                v_f=0,
            ),
        )

    def _add_keys(self) -> None:
        """
        Register keyboard keys as PLC input variables.

        The demo uses ``a``/``m``/``o`` for mode selection, ``shift+s`` for the
        start button, ``q`` to exit, ``e`` for emergency, and ``r`` for reset.
        Jog keys are grouped in ``JogButtons`` for use by :class:`ManualMode`.
        """
        self.key_input.keys["e"] = self.EmergencyButton = MemoryVariable()
        self.key_input.keys["r"] = self.ResetButton = MemoryVariable()

        self.key_input.keys["a"] = self.AutoButton = MemoryVariable()
        self.key_input.keys["m"] = self.ManButton = MemoryVariable()
        self.key_input.keys["o"] = self.OffButton = MemoryVariable()

        self.key_input.keys["shift+s"] = self.StartButton = MemoryVariable()
        self.key_input.keys["q"] = self.ExitButton = MemoryVariable()

        self.key_input.keys["shift+x"] = MemoryVariable()
        self.key_input.keys["x"] = MemoryVariable()
        self.key_input.keys["shift+y"] = MemoryVariable()
        self.key_input.keys["y"] = MemoryVariable()
        self.key_input.keys["shift+z"] = MemoryVariable()
        self.key_input.keys["z"] = MemoryVariable()

        self.JogButtons = {
            "x+": self.key_input.keys["shift+x"],
            "x-": self.key_input.keys["x"],
            "y+": self.key_input.keys["shift+y"],
            "y-": self.key_input.keys["y"],
            "z+": self.key_input.keys["shift+z"],
            "z-": self.key_input.keys["z"],
        }

    def _update_mode_switch(self) -> None:
        """
        Update the virtual three-position mode switch from keyboard buttons.
        """
        if self.AutoButton.rising_edge:
            self.ModeSwitch.force(ThreePositionSwitch.AUTO)
        elif self.ManButton.rising_edge:
            self.ModeSwitch.force(ThreePositionSwitch.MANUAL)
        elif self.OffButton.rising_edge:
            self.ModeSwitch.force(ThreePositionSwitch.OFF)
        else:
            self.ModeSwitch.force(self.ModeSwitch.curr_state)

    def _sequence_control(self) -> None:
        """
        Execute the main GRAFCET transitions.
        """
        if self.S0.active and self.StartButton.rising_edge:
            if self.ModeSwitch.auto:
                self.S0.deactivate()
                self.S1.activate()
            elif self.ModeSwitch.manual:
                self.S0.deactivate()
                self.S2.activate()
        elif self.S1.active and self.aut_mode.TrajectoryFinished.active:
            self.S1.deactivate()
            self.S0.activate()
        elif self.S2.active and not self.ModeSwitch.manual:
            self.S2.deactivate()
            self.S0.activate()

    def _execute_actions(self) -> None:
        """
        Execute the actions attached to the active main-sequence step.
        """
        if self.S0.active:
            match self.ModeSwitch.state:
                case self.ModeSwitch.MANUAL:
                    message = "Manual mode selected"
                case self.ModeSwitch.AUTO:
                    message = "Auto mode selected"
                case self.ModeSwitch.OFF:
                    message = "Press 'a' for auto mode or 'm' for manual mode"
                case _:
                    message = "Unknown mode"

            if self.S0.rising_edge or self.ModeSwitch.changed:
                self.logger.info(f"{message}. Press start to continue.")

            if self.aut_mode.TrajectoryFinished.active:
                self.aut_mode.TrajectoryFinished.deactivate()
                self.aut_mode.reset()

        elif self.S1.active:
            if self.S1.rising_edge:
                self.logger.info("Auto mode active.")
            self.aut_mode.call()

        elif self.S2.active:
            if self.S2.rising_edge:
                self.logger.info("Manual mode active.")
            self.man_mode.call()

    def on_scan_cycle_enter(self) -> None:
        """
        Refresh keyboard input and controller status at the start of each scan.
        """
        self.key_input.update()
        self._update_mode_switch()
        self.cnc_status = self.cnc_controller.get_motion_status()

    def startup_routine(self) -> None:
        """
        Start the motor controller processes and activate the idle step.
        """
        self.ModeSwitch.force(ThreePositionSwitch.OFF)
        self.cnc_controller.enable()
        self.S0.activate()

    def control_routine(self) -> None:
        """
        Execute one normal PLC scan.
        """
        if self.ExitButton.rising_edge:
            self.exit()

        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        """
        Shut down the CNC controller during normal PLC shutdown.
        """
        self.cnc_controller.disable()
        super().exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        """
        Shut down the CNC controller after an unexpected PLC failure.
        """
        self.cnc_controller.disable()
        super().crash_routine(exception)

    def on_emergency_enter(self) -> None:
        """
        Stop the CNC controller processes when emergency mode is entered.
        """
        self.cnc_controller.disable()
        super().on_emergency_enter()

    def recover_routine(self) -> None:
        """
        Recreate the controller and reset subroutines after emergency recovery.
        """
        super().recover_routine()
        self.cnc_controller = self._setup_cnc_controller()
        self.cnc_status = None
        self.man_mode.attach_controller(self.cnc_controller)
        self.aut_mode.attach_controller(self.cnc_controller)
        self.man_mode.reset()
        self.aut_mode.reset()
        self.startup_routine()


class ManualMode:
    """
    Subroutine-style state machine for manual jog mode.

    ``call()`` is invoked once per PLC scan while the main PLC sequence is in
    manual mode. The subroutine keeps its own markers, but those markers are
    registered on the parent PLC so their edge detection follows the same scan
    rules as the main sequence.
    """

    def __init__(self, main: CNCPLC) -> None:
        """
        Create the manual-mode subroutine.

        Parameters
        ----------
        main:
            Parent PLC that owns the subroutine markers and keyboard inputs.
        """
        self.main = main
        self.logger = main.logger
        self.JogButtons = main.JogButtons
        self.cnc_status: MotionStatus | None = None
        self.init_flag = True

        self.attach_controller(main.cnc_controller)

        # Pairs of steps start and then stop jogging for each signed axis.
        self.S0 = main.add_marker("ManualMode.S0")
        self.S1 = main.add_marker("ManualMode.S1")
        self.S2 = main.add_marker("ManualMode.S2")
        self.S3 = main.add_marker("ManualMode.S3")
        self.S4 = main.add_marker("ManualMode.S4")
        self.S5 = main.add_marker("ManualMode.S5")
        self.S6 = main.add_marker("ManualMode.S6")
        self.S7 = main.add_marker("ManualMode.S7")
        self.S8 = main.add_marker("ManualMode.S8")
        self.S9 = main.add_marker("ManualMode.S9")
        self.S10 = main.add_marker("ManualMode.S10")
        self.S11 = main.add_marker("ManualMode.S11")
        self.S12 = main.add_marker("ManualMode.S12")

    # noinspection PyAttributeOutsideInit
    def attach_controller(self, controller: XYZMotionController) -> None:
        """
        Attach a CNC controller to the subroutine.

        Parameters
        ----------
        controller:
            Controller used for jog commands.
        """
        self.cnc_controller = controller
        self.rdir_ref = self._get_rdir_ref_dict()

    def _get_rdir_ref_dict(self) -> dict[str, RotationDirection]:
        """
        Return the configured positive-travel direction for each axis.

        Returns
        -------
        dict[str, RotationDirection]
            Mapping from axis name to reference motor direction.
        """
        configs = (
            self.cnc_controller.x_motor_cfg,
            self.cnc_controller.y_motor_cfg,
            self.cnc_controller.z_motor_cfg,
        )
        rdir_ref = {}
        for axis, cfg in zip(("x", "y", "z"), configs):
            if cfg is None:
                raise ValueError(f"Axis {axis} is missing.")
            rdir_ref[axis] = self.cnc_controller.get_rotation_direction(
                cfg["rdir_ref"]
            )
        return rdir_ref

    def call(self) -> None:
        """
        Execute one scan of the manual-mode subroutine.
        """
        self.cnc_status = self.cnc_controller.get_motion_status()

        if self.init_flag:
            self.init_flag = False
            self.S0.activate()

        self._sequence_control()
        self._execute_actions()

    def reset(self) -> None:
        """
        Deactivate all manual-mode steps and arm the subroutine for reuse.
        """
        for step in (
            self.S0,
            self.S1,
            self.S2,
            self.S3,
            self.S4,
            self.S5,
            self.S6,
            self.S7,
            self.S8,
            self.S9,
            self.S10,
            self.S11,
            self.S12,
        ):
            step.deactivate()
        self.cnc_status = None
        self.init_flag = True

    def _sequence_control(self) -> None:
        """
        Execute the manual jog transitions.
        """
        if self.cnc_status is None:
            return

        # noinspection PyUnresolvedReferences
        if self.S0.active:
            if self.JogButtons["x+"].rising_edge:
                self.S0.deactivate()
                self.S1.activate()
            elif self.JogButtons["x-"].rising_edge:
                self.S0.deactivate()
                self.S3.activate()
            elif self.JogButtons["y+"].rising_edge:
                self.S0.deactivate()
                self.S5.activate()
            elif self.JogButtons["y-"].rising_edge:
                self.S0.deactivate()
                self.S7.activate()
            elif self.JogButtons["z+"].rising_edge:
                self.S0.deactivate()
                self.S9.activate()
            elif self.JogButtons["z-"].rising_edge:
                self.S0.deactivate()
                self.S11.activate()

        elif self.S1.active and self.JogButtons["x+"].falling_edge:
            self.S1.deactivate()
            self.S2.activate()
        elif self.S3.active and self.JogButtons["x-"].falling_edge:
            self.S3.deactivate()
            self.S4.activate()
        elif self.S5.active and self.JogButtons["y+"].falling_edge:
            self.S5.deactivate()
            self.S6.activate()
        elif self.S7.active and self.JogButtons["y-"].falling_edge:
            self.S7.deactivate()
            self.S8.activate()
        elif self.S9.active and self.JogButtons["z+"].falling_edge:
            self.S9.deactivate()
            self.S10.activate()
        elif self.S11.active and self.JogButtons["z-"].falling_edge:
            self.S11.deactivate()
            self.S12.activate()

        elif self.S2.active and not self.cnc_status.x.jog_mode_active:
            self.S2.deactivate()
            self.S0.activate()
        elif self.S4.active and not self.cnc_status.x.jog_mode_active:
            self.S4.deactivate()
            self.S0.activate()
        elif self.S6.active and not self.cnc_status.y.jog_mode_active:
            self.S6.deactivate()
            self.S0.activate()
        elif self.S8.active and not self.cnc_status.y.jog_mode_active:
            self.S8.deactivate()
            self.S0.activate()
        elif self.S10.active and not self.cnc_status.z.jog_mode_active:
            self.S10.deactivate()
            self.S0.activate()
        elif self.S12.active and not self.cnc_status.z.jog_mode_active:
            self.S12.deactivate()
            self.S0.activate()

    def _execute_actions(self) -> None:
        """
        Start or stop jog mode when a manual-mode step becomes active.
        """
        if self.S1.rising_edge:
            self.logger.info("Start X+ jogging")
            self.cnc_controller.start_jog_mode("x", self.rdir_ref["x"])
        elif self.S3.rising_edge:
            self.logger.info("Start X- jogging")
            self.cnc_controller.start_jog_mode("x", ~self.rdir_ref["x"])
        elif self.S5.rising_edge:
            self.logger.info("Start Y+ jogging")
            self.cnc_controller.start_jog_mode("y", self.rdir_ref["y"])
        elif self.S7.rising_edge:
            self.logger.info("Start Y- jogging")
            self.cnc_controller.start_jog_mode("y", ~self.rdir_ref["y"])
        elif self.S9.rising_edge:
            self.logger.info("Start Z+ jogging")
            self.cnc_controller.start_jog_mode("z", self.rdir_ref["z"])
        elif self.S11.rising_edge:
            self.logger.info("Start Z- jogging")
            self.cnc_controller.start_jog_mode("z", ~self.rdir_ref["z"])
        elif self.S2.rising_edge or self.S4.rising_edge:
            self.logger.info("Stop X+/- jogging")
            self.cnc_controller.stop_jog_mode("x")
        elif self.S6.rising_edge or self.S8.rising_edge:
            self.logger.info("Stop Y+/- jogging")
            self.cnc_controller.stop_jog_mode("y")
        elif self.S10.rising_edge or self.S12.rising_edge:
            self.logger.info("Stop Z+/- jogging")
            self.cnc_controller.stop_jog_mode("z")


class AutoMode:
    """
    Subroutine-style state machine for automatic trajectory execution.

    The subroutine first loads a JSON trajectory and then feeds its segments to
    :class:`XYZMotionController` whenever all axis controllers report ready.
    """

    def __init__(
        self,
        main: CNCPLC,
        *,
        trajectory_filepath: Path,
    ) -> None:
        """
        Create the auto-mode subroutine.

        Parameters
        ----------
        main:
            Parent PLC that owns the subroutine markers and start button.
        trajectory_filepath:
            JSON trajectory file to load when the subroutine starts.
        """
        self.main = main
        self.logger = main.logger
        self.trajectory_filepath = trajectory_filepath
        self.cnc_status: MotionStatus | None = None
        self.init_flag = True

        self.attach_controller(main.cnc_controller)

        self.TrajectoryLoaded = MemoryVariable()
        self.TrajectoryFinished = MemoryVariable()

        self.S0 = main.add_marker("AutoMode.S0")
        self.S1 = main.add_marker("AutoMode.S1")
        self.S2 = main.add_marker("AutoMode.S2")

    def attach_controller(self, controller: XYZMotionController) -> None:
        """
        Attach a CNC controller to the subroutine.

        Parameters
        ----------
        controller:
            Controller used for trajectory execution.
        """
        # noinspection PyAttributeOutsideInit
        self.cnc_controller = controller

    def call(self) -> None:
        """
        Execute one scan of the auto-mode subroutine.
        """
        self.cnc_status = self.cnc_controller.get_motion_status()

        if self.init_flag:
            self.init_flag = False
            self.S0.activate()

        self._sequence_control()
        self._execute_actions()

    def reset(self) -> None:
        """
        Deactivate all auto-mode steps and arm the subroutine for reuse.
        """
        self.S0.deactivate()
        self.S1.deactivate()
        self.S2.deactivate()
        self.TrajectoryLoaded.deactivate()
        self.TrajectoryFinished.deactivate()
        self.cnc_status = None
        self.init_flag = True

    def _sequence_control(self) -> None:
        """
        Execute the auto-mode transitions.
        """
        if self.cnc_status is None:
            return

        if self.S0.active and self.main.StartButton.rising_edge:
            self.S0.deactivate()
            self.S1.activate()
        elif (
            self.S1.active
            and self.TrajectoryLoaded.active
            and self.cnc_status.all_ready
        ):
            self.S1.deactivate()
            self.S2.activate()

    def _execute_actions(self) -> None:
        """
        Load the trajectory and execute it one segment at a time.
        """
        if self.cnc_status is None:
            return

        if self.S1.rising_edge:
            self.logger.info("Loading trajectory into memory")
            if not self.trajectory_filepath.exists():
                self.main.request_emergency(
                    "Trajectory file missing. Run create_trajectory.py first."
                )
            num_segments = self.cnc_controller.load_trajectory(
                filepath=str(self.trajectory_filepath)
            )
            if num_segments > 0:
                self.TrajectoryLoaded.activate()
            else:
                self.main.request_emergency("Loading failed.")

        elif self.S2.active:
            self.TrajectoryLoaded.deactivate()
            if self.S2.rising_edge:
                self.logger.info("Executing trajectory on the machine.")
            if self.cnc_status.all_ready:
                self.cnc_controller.move()
                self.cnc_status = self.cnc_controller.get_motion_status()
                # noinspection PyUnresolvedReferences
                if self.cnc_status.all_finished:
                    self.TrajectoryFinished.activate()


def main() -> None:
    """
    Run the CNC PLC demo.
    """
    cnc_plc = CNCPLC()
    cnc_plc.run()


if __name__ == "__main__":
    main()
