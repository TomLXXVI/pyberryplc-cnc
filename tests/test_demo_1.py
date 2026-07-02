import importlib.util
import json
from pathlib import Path


DEMO_DIR = Path(__file__).resolve().parents[1] / "demos" / "demo_1"


def _load_create_trajectory_module():
    """
    Load the demo trajectory generator as a module.

    Returns
    -------
    module
        Imported ``create_trajectory.py`` module.
    """
    module_path = DEMO_DIR / "create_trajectory.py"
    spec = importlib.util.spec_from_file_location(
        "pyberryplc_cnc_demo_1_create_trajectory",
        module_path,
    )
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_demo_trajectory_generator_writes_controller_json(tmp_path):
    """
    Generate a small controller trajectory without starting the demo PLC.
    """
    create_trajectory = _load_create_trajectory_module()
    output_filepath = tmp_path / "stepper_trajectory.json"

    num_segments = create_trajectory.create_demo_trajectory(
        motor_config_filepath=DEMO_DIR / "motor_config.toml",
        output_filepath=output_filepath,
        vertices=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        feed_rate=2.0,
        dt_blends=0.0,
        axes=("x",),
        include_stationary_axes=False,
    )

    data = json.loads(output_filepath.read_text(encoding="utf-8"))

    assert num_segments == 1
    assert len(data) == 1
    assert set(data[0]) == {"x"}
    delays, direction = data[0]["x"]
    assert delays
    assert direction in {"clockwise", "counterclockwise"}
