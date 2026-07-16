# Fixed L Ground Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a one-shot ROS 2 Humble tool that sends the fly-car's fixed ground L route, captures synchronized control data, and writes per-segment tuning metrics for later analysis.

**Architecture:** One Python script contains a ROS-independent analysis core plus a ROS adapter defined only after runtime imports, so unit tests run on the development machine without ROS. The adapter publishes the formal `/wildlife/waypoints` route, samples TF and control topics at 20 Hz, applies optional hot parameters, stops at the fixed endpoint, and writes a self-contained run directory.

**Tech Stack:** Python 3.10+, `unittest`, ROS 2 Humble `rclpy`, `tf2_ros`, standard ROS messages, standard-library CSV/JSON/statistics/subprocess.

---

## File map

- Create `fly_car/scripts/l_path_tuning.py`: pure geometry/statistics functions, CLI parsing, ROS node, safety checks, route publication, sampling, output.
- Create `fly_car/scripts/test_l_path_tuning.py`: ROS-free unit tests for parameter parsing, segmentation, geometry, sign-flip counting, endpoint stability, and summaries.
- Modify `fly_car/scripts/test_all.py`: include both new Python files in syntax checks and run the ROS-free unit test during `--where local`.

### Task 1: Pure fixed-L model and RED tests

**Files:**
- Create: `fly_car/scripts/test_l_path_tuning.py`
- Create: `fly_car/scripts/l_path_tuning.py`

- [ ] **Step 1: Write failing tests for the route contract and geometry**

Tests must import `l_path_tuning` by adding the script directory to `sys.path`, then assert:

```python
class FixedLGeometryTests(unittest.TestCase):
    def test_route_is_exact_fixed_ground_l(self):
        self.assertEqual(
            tuning.ROUTE_FLOATS,
            [0.0, 0.0, 0.0, 0.0,
             50.0, 0.0, 0.0, 74.1,
             150.0, 350.0, 0.0, 74.1],
        )

    def test_cross_track_for_second_leg(self):
        self.assertAlmostEqual(
            tuning.cross_track_cm("LEG_2", 60.0, 35.0), 0.0, places=6)

    def test_heading_error_wraps(self):
        self.assertAlmostEqual(tuning.angle_error_deg(-179.0, 179.0), 2.0)
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
python3 -m unittest fly_car/scripts/test_l_path_tuning.py -v
```

Expected: FAIL because `l_path_tuning` or its constants/functions do not exist.

- [ ] **Step 3: Implement the minimal pure model**

Define these exact public elements without importing ROS:

```python
ROUTE_POINTS = (
    Waypoint(0.0, 0.0, 0.0, 0.0),
    Waypoint(50.0, 0.0, 0.0, 74.1),
    Waypoint(150.0, 350.0, 0.0, 74.1),
)
ROUTE_FLOATS = [value for point in ROUTE_POINTS for value in point]

def normalize_deg(angle: float) -> float:
    return math.degrees(math.atan2(math.sin(math.radians(angle)),
                                   math.cos(math.radians(angle))))

def angle_error_deg(target: float, actual: float) -> float:
    return normalize_deg(target - actual)

def cross_track_cm(segment: str, x_cm: float, y_cm: float) -> float:
    if segment == "LEG_1":
        x0, y0, x1, y1 = 0.0, 0.0, 50.0, 0.0
    elif segment == "LEG_2":
        x0, y0, x1, y1 = 50.0, 0.0, 150.0, 350.0
    else:
        raise ValueError(f"unsupported straight segment: {segment}")
    dx, dy = x1 - x0, y1 - y0
    return (dx * (y_cm - y0) - dy * (x_cm - x0)) / math.hypot(dx, dy)
```

`cross_track_cm` accepts only `LEG_1` and `LEG_2`, raising `ValueError` otherwise.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run the same unittest command. Expected: all geometry tests PASS.

### Task 2: Parameter parsing, segmentation, and metrics

**Files:**
- Modify: `fly_car/scripts/test_l_path_tuning.py`
- Modify: `fly_car/scripts/l_path_tuning.py`

- [ ] **Step 1: Add RED tests for overrides and state classification**

Cover:

```python
self.assertEqual(parse_overrides(["straight_kp_w=0.22", "ki_w=0"]),
                 {"straight_kp_w": 0.22, "ki_w": 0.0})
with self.assertRaises(ValueError):
    parse_overrides(["bad"])

self.assertEqual(classify_target((50.0, 0.0, 0.0, 74.1), 10.0), "LEG_1")
self.assertEqual(classify_target((50.0, 0.0, 0.0, 74.1), 3.0), "TURN")
self.assertEqual(classify_target((150.0, 350.0, 0.0, 74.1), 100.0), "LEG_2")
```

Add samples proving `count_effective_sign_flips([0.1, 0.0, -0.1], 0.02) == 1` and sub-threshold values are ignored.

- [ ] **Step 2: Verify the new tests fail for missing functions**

Run the focused unittest command and confirm failures name the missing APIs.

- [ ] **Step 3: Implement pure parsing and statistics**

Add:

```python
def parse_overrides(items: Sequence[str]) -> dict[str, float]
def classify_target(target: Sequence[float] | None, distance_cm: float) -> str
def count_effective_sign_flips(values: Iterable[float], threshold: float = 0.02) -> int
def summarize_samples(samples: Sequence[Sample]) -> dict[str, object]
```

`summarize_samples` groups `LEG_1`, `TURN`, and `LEG_2`; straight legs include duration, sample count, mean/max speed, heading-error RMS/max/peak-to-peak, cross-track RMS/max, `w` flips, and max `|w|`. TURN includes duration, max overshoot beyond 74.1°, and final yaw error. Empty groups return `sample_count: 0` instead of raising.

- [ ] **Step 4: Verify all pure tests pass**

Run unittest; expected PASS with no ROS installation required.

- [ ] **Step 5: Commit the pure analysis slice**

```bash
git -C fly_car add scripts/l_path_tuning.py scripts/test_l_path_tuning.py
git -C fly_car commit -m "test: define fixed L tuning metrics"
```

### Task 3: ROS adapter and safe one-shot runner

**Files:**
- Modify: `fly_car/scripts/l_path_tuning.py`
- Modify: `fly_car/scripts/test_l_path_tuning.py`

- [ ] **Step 1: Add RED tests for endpoint stability and output naming**

Use supplied monotonic timestamps to verify endpoint completion only after continuously satisfying 12 cm / 5° for 0.5 seconds, and that leaving tolerance resets the timer. Test run-directory names are timestamped and collision-safe.

- [ ] **Step 2: Verify RED**

Run unittest and confirm missing `EndpointLatch`/`create_run_directory` failures.

- [ ] **Step 3: Implement the pure endpoint latch and filesystem helpers**

Add `EndpointLatch.update(now_s, distance_cm, yaw_error_deg) -> bool`, `create_run_directory(root, now)`, CSV field constants, and JSON-safe summary writing.

- [ ] **Step 4: Implement runtime ROS imports and `FixedLTuningNode`**

Inside `run_ros(args)`, import `rclpy`, `tf2_ros`, `geometry_msgs.msg.Twist`, `std_msgs.msg.Bool`, and `std_msgs.msg.Float32MultiArray`. Implement:

- Reliable/transient-local publisher `/wildlife/waypoints`.
- Reliable/transient-local subscriptions `/target_position`, `/ground_enable`, `/flight_enable`.
- Subscription `/cmd_vel`.
- TF lookup `map <- laser_link` at 20 Hz with 0.5-second freshness validation.
- Required-node check for `route_test_node`, `diff_drive_controller`, `chassis_mux`.
- Refusal to publish if `/flight_enable` is true.
- 20 Hz `Sample` collection and immediate CSV flushing.
- Final endpoint latch and 120-second timeout.
- On abort, publish `[current_x_cm, current_y_cm, 0.0, current_yaw_deg]` as a one-point hold route.

Wait for at least one `/wildlife/waypoints` subscriber before publishing. Publish the fixed route three times at 100 ms intervals to tolerate startup timing while retaining transient-local QoS.

- [ ] **Step 5: Implement parameter application and snapshot**

Before route publication, run `ros2 param set /diff_drive_controller NAME VALUE` for each validated `--set`. Any nonzero return aborts. Then run `ros2 param dump /diff_drive_controller`, preserve raw YAML as `params.yaml`, and write `params.json` containing baseline recommendations, requested overrides, command results, and raw dump text.

- [ ] **Step 6: Implement CLI and reports**

Arguments exactly match the spec. Write `samples.csv`, `params.yaml`, `params.json`, `summary.json`, and `summary.txt`. Print the run directory and concise segment table at exit.

- [ ] **Step 7: Run ROS-free tests and syntax checks**

```bash
python3 -m unittest fly_car/scripts/test_l_path_tuning.py -v
python3 -m py_compile fly_car/scripts/l_path_tuning.py fly_car/scripts/test_l_path_tuning.py
```

Expected: PASS; no `rclpy` import required during tests.

### Task 4: Integrate local verification

**Files:**
- Modify: `fly_car/scripts/test_all.py`

- [ ] **Step 1: Add a failing integration assertion**

Extend `PY_SYNTAX_FILES` with both scripts and add a local unittest entry for `scripts/test_l_path_tuning.py`. Run `python3 fly_car/scripts/test_all.py --where local`; expected failure before integration code is added.

- [ ] **Step 2: Implement the local test hook**

Call:

```python
run([sys.executable, "-m", "unittest", "scripts/test_l_path_tuning.py", "-v"],
    ROOT / "fly_car")
```

Record its result in the existing summary table.

- [ ] **Step 3: Run full local verification**

```bash
python3 fly_car/scripts/test_all.py --where local
```

Expected: fixed-L syntax and unit-test entries PASS. Report unrelated pre-existing failures separately; do not edit other subsystems.

- [ ] **Step 4: Review exact scope**

```bash
git -C fly_car diff --check -- scripts/l_path_tuning.py scripts/test_l_path_tuning.py scripts/test_all.py
git -C fly_car diff -- scripts/l_path_tuning.py scripts/test_l_path_tuning.py scripts/test_all.py
```

Confirm no changes to `chassis_mux.cpp`, `route_target_publisher.cpp`, or `car/`.

- [ ] **Step 5: Commit the ROS runner**

```bash
git -C fly_car add scripts/l_path_tuning.py scripts/test_l_path_tuning.py scripts/test_all.py
git -C fly_car commit -m "feat: capture fixed L ground tuning runs"
```

### Task 5: Board handoff

**Files:**
- No development-machine code changes.

- [ ] **Step 1: Provide board commands**

Document commands to sync, export domain 1, start `patrol_ground.launch.py`, run `--dry-run`, then run baseline overrides. Repeat the no-propeller warning.

- [ ] **Step 2: Define returned artifact**

Ask for the entire timestamped run directory, not only `summary.txt`, because tuning analysis needs raw `samples.csv` and the exact parameter snapshot.
