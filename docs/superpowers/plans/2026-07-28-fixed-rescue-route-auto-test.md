# Fixed Rescue Route Auto Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fly-car-only ROS 2 launch that waits eight seconds and then executes the current fixed rescue ground, flight, return, and landing route without AI Agent, Web, car-side, or UDP dependencies.

**Architecture:** Store the fixed local-map waypoints in a small importable Python module owned by `my_launch`, then pass its flattened route directly to the existing `route_test_node` with `preload_waypoints=True`. The new launch starts Cartographer immediately and starts the existing ground chassis, UART, position PID, and route executor after a configurable delay.

**Tech Stack:** Python 3, ROS 2 Humble launch/launch_ros, pytest, existing `activity_control_pkg/route_test_node`

---

## File Structure

- Create `src/my_launch/my_launch/fixed_rescue_route.py`: canonical local-map waypoint tuple, default delay, and flattening helper.
- Create `src/my_launch/test/test_fixed_rescue_route.py`: route-value and launch-contract regression tests runnable without a ROS installation.
- Create `src/my_launch/launch/fixed_rescue_route.launch.py`: standalone full-stack launch using only fly-car-local components.

### Task 1: Lock the Fixed Route in a Tested Python Module

**Files:**
- Create: `src/my_launch/my_launch/fixed_rescue_route.py`
- Create: `src/my_launch/test/test_fixed_rescue_route.py`

- [ ] **Step 1: Write the failing route regression tests**

Create `src/my_launch/test/test_fixed_rescue_route.py`:

```python
from my_launch.fixed_rescue_route import (
    DEFAULT_START_DELAY_S,
    FIXED_RESCUE_WAYPOINTS,
    flattened_waypoints,
)


EXPECTED_WAYPOINTS = (
    (0.0, 0.0, 0.0, 0.0),
    (300.0, 0.0, 0.0, 0.0),
    (300.0, 0.0, 100.0, 0.0),
    (300.0, 0.0, 100.0, -90.0),
    (300.0, -300.0, 100.0, -90.0),
    (300.0, -375.0, 100.0, -90.0),
    (300.0, -375.0, 100.0, 180.0),
    (200.0, -375.0, 100.0, 180.0),
    (200.0, -375.0, 100.0, 90.0),
    (200.0, -250.0, 100.0, 90.0),
    (200.0, -250.0, 100.0, 180.0),
    (100.0, -250.0, 100.0, 180.0),
    (100.0, -250.0, 100.0, -90.0),
    (100.0, -375.0, 100.0, -90.0),
    (100.0, -375.0, 100.0, 180.0),
    (41.0, -375.0, 100.0, 180.0),
    (41.0, -375.0, 100.0, 90.0),
    (41.0, -238.0, 100.0, 90.0),
    (41.0, -200.0, 100.0, 90.0),
    (41.0, -200.0, 100.0, 99.694444),
    (0.0, 40.0, 100.0, 99.694444),
    (0.0, 40.0, 0.0, 99.694444),
)


def test_fixed_route_matches_current_voice_rescue_route():
    assert FIXED_RESCUE_WAYPOINTS == EXPECTED_WAYPOINTS
    assert DEFAULT_START_DELAY_S == 8.0


def test_flattened_route_keeps_climb_turn_return_and_landing_points():
    flat = flattened_waypoints()

    assert len(flat) == len(EXPECTED_WAYPOINTS) * 4
    assert flat[:8] == [0.0, 0.0, 0.0, 0.0, 300.0, 0.0, 0.0, 0.0]
    assert flat[8:16] == [
        300.0, 0.0, 100.0, 0.0,
        300.0, 0.0, 100.0, -90.0,
    ]
    assert flat[-8:] == [
        0.0, 40.0, 100.0, 99.694444,
        0.0, 40.0, 0.0, 99.694444,
    ]
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest \
  src/my_launch/test/test_fixed_rescue_route.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'my_launch.fixed_rescue_route'`.

- [ ] **Step 3: Add the minimal route module**

Create `src/my_launch/my_launch/fixed_rescue_route.py`:

```python
"""Fixed fly-car-local route used by the standalone rescue flight test."""

DEFAULT_START_DELAY_S = 8.0

FIXED_RESCUE_WAYPOINTS = (
    (0.0, 0.0, 0.0, 0.0),
    (300.0, 0.0, 0.0, 0.0),
    (300.0, 0.0, 100.0, 0.0),
    (300.0, 0.0, 100.0, -90.0),
    (300.0, -300.0, 100.0, -90.0),
    (300.0, -375.0, 100.0, -90.0),
    (300.0, -375.0, 100.0, 180.0),
    (200.0, -375.0, 100.0, 180.0),
    (200.0, -375.0, 100.0, 90.0),
    (200.0, -250.0, 100.0, 90.0),
    (200.0, -250.0, 100.0, 180.0),
    (100.0, -250.0, 100.0, 180.0),
    (100.0, -250.0, 100.0, -90.0),
    (100.0, -375.0, 100.0, -90.0),
    (100.0, -375.0, 100.0, 180.0),
    (41.0, -375.0, 100.0, 180.0),
    (41.0, -375.0, 100.0, 90.0),
    (41.0, -238.0, 100.0, 90.0),
    (41.0, -200.0, 100.0, 90.0),
    (41.0, -200.0, 100.0, 99.694444),
    (0.0, 40.0, 100.0, 99.694444),
    (0.0, 40.0, 0.0, 99.694444),
)


def flattened_waypoints() -> list[float]:
    """Return the route layout consumed by route_test_node."""
    return [value for waypoint in FIXED_RESCUE_WAYPOINTS for value in waypoint]
```

- [ ] **Step 4: Run only the two route tests**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest \
  src/my_launch/test/test_fixed_rescue_route.py \
  -k "matches_current or keeps_climb" -q
```

Expected: `2 passed`.

- [ ] **Step 5: Commit the route contract**

```bash
git add \
  src/my_launch/my_launch/fixed_rescue_route.py \
  src/my_launch/test/test_fixed_rescue_route.py
git commit -m "test: lock standalone rescue flight route"
```

### Task 2: Add the Standalone Auto-Start Launch

**Files:**
- Create: `src/my_launch/launch/fixed_rescue_route.launch.py`
- Test: `src/my_launch/test/test_fixed_rescue_route.py`

- [ ] **Step 1: Add the launch contract test**

Append this test to `src/my_launch/test/test_fixed_rescue_route.py` and add the
`Path` import at the top:

```python
from pathlib import Path


def test_launch_source_is_standalone_and_uses_the_route_module():
    launch_path = (
        Path(__file__).parents[1] / "launch" / "fixed_rescue_route.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")

    assert '"preload_waypoints": True' in source
    assert '"waypoints": flattened_waypoints()' in source
    assert '"start_delay_s"' in source
    assert 'LaunchConfiguration("start_delay_s")' in source
    for required_package in (
        "my_carto_pkg",
        "ground_chassis_pkg",
        "uart_to_stm32",
        "pid_control_pkg",
        "activity_control_pkg",
    ):
        assert required_package in source
    for forbidden_component in (
        "xmachine_bridge",
        "yolo_detector",
        "terminal",
        "rescue_drop_sequencer",
    ):
        assert forbidden_component not in source
```

- [ ] **Step 2: Run the launch test and verify RED**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest \
  src/my_launch/test/test_fixed_rescue_route.py::test_launch_source_is_standalone_and_uses_the_route_module \
  -q
```

Expected: FAIL with `FileNotFoundError` for `fixed_rescue_route.launch.py`.

- [ ] **Step 3: Add the minimal standalone launch**

Create `src/my_launch/launch/fixed_rescue_route.launch.py`:

```python
"""Run the fixed rescue route on the fly car without AI Agent or car-side nodes.

The full route is preloaded after a configurable delay. The transmitter flight
switch remains the final hardware takeoff gate.

Usage:
  ros2 launch my_launch fixed_rescue_route.launch.py
  ros2 launch my_launch fixed_rescue_route.launch.py start_delay_s:=12.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_launch.fixed_rescue_route import (
    DEFAULT_START_DELAY_S,
    flattened_waypoints,
)


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    start_delay = LaunchConfiguration("start_delay_s")
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include(
        "ground_chassis_pkg", "ground_chassis.launch.py"
    )
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include(
        "pid_control_pkg", "position_pid_controller.launch.py"
    )
    route_executor = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="fixed_rescue_route_executor",
        output="screen",
        parameters=[{
            "preload_waypoints": True,
            "waypoints": flattened_waypoints(),
            "position_tolerance_cm": 12.0,
            "lookahead_count": 0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_delay_s",
            default_value=str(DEFAULT_START_DELAY_S),
            description="Seconds to wait for Cartographer/TF before motion starts",
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        LogInfo(msg=[
            "Fixed rescue route test: motion starts after ",
            start_delay,
            "s; transmitter flight switch remains the takeoff gate.",
        ]),
        fly_carto,
        TimerAction(
            period=start_delay,
            actions=[ground_chassis, uart, position_pid, route_executor],
        ),
    ])
```

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest \
  src/my_launch/test/test_fixed_rescue_route.py -q
```

Expected: `3 passed`.

- [ ] **Step 5: Check Python syntax**

Run:

```bash
python3 -m py_compile \
  src/my_launch/my_launch/fixed_rescue_route.py \
  src/my_launch/launch/fixed_rescue_route.launch.py \
  src/my_launch/test/test_fixed_rescue_route.py
```

Expected: exit code 0 with no output.

- [ ] **Step 6: Run the package Python tests**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest src/my_launch/test -q
```

Expected: all `my_launch` tests pass.

- [ ] **Step 7: Verify installation discovery without building**

Run:

```bash
python3 -c "from glob import glob; assert \
'src/my_launch/launch/fixed_rescue_route.launch.py' in \
glob('src/my_launch/launch/*.launch.py')"
```

Expected: exit code 0. `setup.py` already installs every `launch/*.launch.py`,
so no packaging edit is required.

- [ ] **Step 8: Commit the standalone launch**

```bash
git add \
  src/my_launch/launch/fixed_rescue_route.launch.py \
  src/my_launch/test/test_fixed_rescue_route.py
git commit -m "feat: add standalone fixed rescue route launch"
```

### Task 3: Final Review and Handoff

**Files:**
- Verify: `src/my_launch/my_launch/fixed_rescue_route.py`
- Verify: `src/my_launch/launch/fixed_rescue_route.launch.py`
- Verify: `src/my_launch/test/test_fixed_rescue_route.py`

- [ ] **Step 1: Review only task-owned changes**

Run:

```bash
git diff 60d5d5f..HEAD -- \
  src/my_launch/my_launch/fixed_rescue_route.py \
  src/my_launch/launch/fixed_rescue_route.launch.py \
  src/my_launch/test/test_fixed_rescue_route.py
```

Expected: only the route module, standalone launch, and regression test appear.

- [ ] **Step 2: Re-run final local verification**

Run:

```bash
PYTHONPATH=src/my_launch python3 -m pytest \
  src/my_launch/test/test_fixed_rescue_route.py -q
python3 -m py_compile \
  src/my_launch/my_launch/fixed_rescue_route.py \
  src/my_launch/launch/fixed_rescue_route.launch.py \
  src/my_launch/test/test_fixed_rescue_route.py
```

Expected: `3 passed`, followed by a successful syntax check with no output.

- [ ] **Step 3: Record the development-board commands**

After syncing to the fly-car board:

```bash
cd ~/kian_flycar/fly_car
colcon build --symlink-install --packages-select my_launch
source install/setup.bash
ros2 launch my_launch fixed_rescue_route.launch.py
```

For a longer TF warm-up:

```bash
ros2 launch my_launch fixed_rescue_route.launch.py start_delay_s:=12.0
```

Expected: the ground segment starts after the delay; the route then waits on
the transmitter-controlled flight gate before physical takeoff and continues
through patrol, return, and landing when the gate is enabled.
