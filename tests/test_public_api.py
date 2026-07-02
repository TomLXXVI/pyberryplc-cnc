def test_cnc_public_api_exposes_xyz_motion_controller():
    """
    Expose the XYZ motion controller from the CNC package.

    This test documents the intended import path for CNC applications:
    ``from pyberryplc_cnc import XYZMotionController``.
    """
    from pyberryplc_cnc import (
        MotorStatus,
        MotionStatus,
        XYZMotionController,
    )
    from pyberryplc_cnc.controller import (
        MotorStatus as CanonicalMotorStatus,
        MotionStatus as CanonicalMotionStatus,
        XYZMotionController as CanonicalXYZMotionController,
    )

    assert MotorStatus is CanonicalMotorStatus
    assert MotionStatus is CanonicalMotionStatus
    assert XYZMotionController is CanonicalXYZMotionController


def test_stepper_public_api_does_not_expose_xyz_motion_controller():
    """
    Keep CNC-specific controllers out of the generic stepper API.

    Applications that need coordinated XYZ trajectories should import the
    controller from ``pyberryplc_cnc`` instead of ``pyberryplc_stepper``.
    """
    import pyberryplc_stepper
    import pyberryplc_stepper.controller as stepper_controller

    assert not hasattr(pyberryplc_stepper, "MotorStatus")
    assert not hasattr(pyberryplc_stepper, "MotionStatus")
    assert not hasattr(pyberryplc_stepper, "XYZMotionController")
    assert not hasattr(stepper_controller, "MotorStatus")
    assert not hasattr(stepper_controller, "MotionStatus")
    assert not hasattr(stepper_controller, "XYZMotionController")
