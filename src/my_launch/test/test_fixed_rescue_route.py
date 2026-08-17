from pathlib import Path

from my_launch.fixed_rescue_route import (
    DEFAULT_START_DELAY_S,
    FIXED_RESCUE_WAYPOINTS,
    flattened_waypoints,
)


def test_fixed_rescue_route_contract():
    assert DEFAULT_START_DELAY_S == 8.0
    assert len(FIXED_RESCUE_WAYPOINTS) == 22
    assert FIXED_RESCUE_WAYPOINTS[:4] == (
        (0.0, 0.0, 0.0, 0.0),
        (300.0, 0.0, 0.0, 0.0),
        (300.0, 0.0, 100.0, 0.0),
        (300.0, 0.0, 100.0, -90.0),
    )
    assert FIXED_RESCUE_WAYPOINTS[-2:] == (
        (0.0, 40.0, 100.0, 99.694444),
        (0.0, 40.0, 0.0, 99.694444),
    )
    assert len(flattened_waypoints()) == 88


def test_standalone_launch_uses_local_stack_only():
    source = (
        Path(__file__).parents[1] / "launch" / "fixed_rescue_route.launch.py"
    ).read_text(encoding="utf-8")

    assert '"preload_waypoints": True' in source
    assert '"waypoints": flattened_waypoints()' in source
    for package in (
        "my_carto_pkg",
        "ground_chassis_pkg",
        "uart_to_stm32",
        "pid_control_pkg",
        "activity_control_pkg",
    ):
        assert package in source
    for external in ("xmachine_bridge", "yolo_detector", "rescue_drop_sequencer"):
        assert external not in source
