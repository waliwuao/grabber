# AGENTS.md — 2D LiDAR Target Tracking

## Setup & Build

- **Requires ROS 2 Jazzy** on Ubuntu 24.04. Source it before any build or run:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```
- **Always use `--symlink-install`** so Python edits take effect without a rebuild:
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```
- Build only the Python package (fast):
  ```bash
  colcon build --symlink-install --packages-select spear_locator
  ```
- `build/` and `install/` have `COLCON_IGNORE` markers — colcon won't recurse into them.

## Symlink `data → datasets`

Scripts reference `data/` paths. The `.gitignore` excludes both `data` and `datasets/` (untracked data). Create the symlink once:
```bash
ln -s datasets data
```

## Packages

| Package | Language | Build |
|---|---|---|
| `ldlidar_stl_ros2` | C++ (`ament_cmake`) | STL-27L vendor driver (locally patched for Jazzy/GCC 13) |
| `spear_locator` | Python (`ament_python`) | Target recognition, calibration, PID control |
| `ares_tool_interfaces` | C++ (`ament_cmake`) | ARES R2 Tool service definitions (copied from external repo) |

The C++ driver publishes `/scan` (`sensor_msgs/LaserScan`). All application logic lives in the Python package.

## Testing

```bash
pytest -q src/spear_locator/test    # 22 tests across 3 files
```

Tests are pure Python (no ROS 2 running required). They cover:
- `test_core.py`: scan filtering, clustering, calibration
- `test_temporal_recognition.py`: multi-frame voting, target scoring, compensations
- `test_position_pid.py`: PID logic

## Running (hardware)

- LiDAR serial port varies — check with `ls -l /dev/ttyUSB*`.
- **Never run rosbag play and the real driver simultaneously** — both publish `/scan`.
- Recognition requires `expected_count:=5` or `expected_count:=6` as a launch argument.
- PID defaults to disabled on launch (`enabled:=false`) — preview first, then engage.

## Key Architecture

- **Coordinate frame**: LiDAR frame. X = target row (left-right), Y = measurement axis. The LiDAR points into **-Y** (Y is negative in the working region).
- **ID assignment**: Targets sorted by X ascending (smallest X → ID 0).
- **X precision is the primary metric**; Y is computed but not a validation target.
- **Radial calibration coefficients** and **polar compensation** are hardcoded in `core.py`. They are hardware-specific — must be recalibrated if the mechanical setup changes.
- **PID is outer-loop only**; actual motors need their own inner velocity/current loop and independent timeout protection.
- No CI/CD exists. Linting is ROS 2 ament style (`ament_flake8`, `ament_pep257`) but not enforced by any pre-commit hook.

## Entry Points (Python console_scripts)

| Command | Module |
|---|---|
| `analyze_bag` | Offline bag analysis |
| `spear_recognition_node` | Multi-frame recognition pipeline |
| `spear_locator_node` | Single-frame clustering (legacy) |
| `spear_position_pid_node` | X-position outer-loop PID |
| `lateral_pid_node` | Trigger-based open-loop lateral control |
| `connector_approach_node` | Trigger-based prepare alignment + chassis forward (replaces PID) |
| `synthetic_scan_node` | Fake scan generator (no hardware needed) |
| `exam_answer_node` | Frozen blind-test answer publisher |

All are ROS 2 nodes launched via `ros2 launch spear_locator <name>.launch.py` — not run directly.

## Documentation

- `docs/project-memory.md` — exhaustive development history and experiment records
- `docs/processing.md` — recognition algorithm walkthrough
- `docs/calibration.md` — radial calibration process and frozen coefficients
- `docs/pid-control.md` — PID safety rules and parameters
- `docs/exam-result.md` — frozen blind-test answers (immutable)
- `docs/recognition-results.md` — accuracy benchmarks
- `README_CONTROL.md` — ARES R2 Tool connector control (separate workspace `app/ares_ws`)

## Connector Approach (replace PID)

`connector_approach_node` uses two independent triggers to replace the PID-controlled chassis alignment:

```bash
ros2 launch spear_locator connector_approach.launch.py

# Step 1: align X via prepare (adjusts connector position)
python3 scripts/trigger_prepare.py

# Step 2: chassis forward approach
python3 scripts/trigger_forward.py
```

- `trigger_prepare` calls `/ares_tool_node/tool_action {prepare}` with `length = direction_sign_x * X_error + prepare_offset_m`, blocking until MCU confirms completion.
- `trigger_forward` sends an open-loop timed forward pulse to `/t0x0101_` based on Y error.
- `forward_4s.py` is a standalone script that publishes a fixed 0.1 m/s forward pulse to `/t0x0101_` for 4 seconds (no recognition dependency).
- The `ares_tool_node` runs in the external `app/ares_ws` workspace (TreeAction repo) — both workspaces must be sourced.
- `ares_tool_interfaces` is included in this repo for build-time interface resolution.

- Prose is in Chinese.
