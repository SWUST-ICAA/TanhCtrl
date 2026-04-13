# TanhCtrl Structure Refactor Design

**Date:** 2026-04-13

**Goal:** Refactor `tanh_ctrl` to match the maintainability and code-ownership boundaries of `/home/nanwan/work_dir/px4adrc_ws/src/px4adrc` while keeping the package name, runtime behavior, message definition, and parameter compatibility intact.

## Scope

This refactor standardizes the package around the same layered shape used by `px4adrc`:

- pure data types
- pure math/common helpers
- pure controller implementation
- ROS2/PX4 node adapter
- minimal `main`
- focused tests

The package name remains `tanh_ctrl`. The refactor does not change control-law intent, message definitions, default topics, or the YAML parameter schema unless a small compatibility-preserving cleanup is required by the new structure.

## Current Problems

The package already has the expected top-level ROS2 directories, but the internal boundaries are weak:

- `src/tanh_ctrl.cpp` mixes controller state, geometry helpers, attitude reconstruction, control allocation, and numeric sanitation.
- `src/tanh_ctrl_node.cpp` mixes parameter declaration/loading, PX4 message translation, mission-state transitions, hold-reference generation, and control-loop execution.
- class and file naming do not clearly distinguish pure controller logic from ROS2 node logic.
- tests are stored under `tests/` instead of the reference package's `test/` layout and currently lock only one behavior.
- launch files contain duplicated source-tree config lookup logic.

The result is a package that works, but is harder to reason about than `px4adrc` because readers must keep multiple unrelated responsibilities in context at the same time.

## Design Constraints

The following items are explicitly preserved:

- package name: `tanh_ctrl`
- executable name: `tanh_ctrl_node`
- message file: `msg/FlatTrajectoryReference.msg`
- Python reference publisher: `scripts/flatness_reference_publisher.py`
- default topic semantics
- YAML parameter names and structure, unless a change is strictly internal and backward compatible
- mission behavior and control-output semantics

The following items are explicitly out of scope:

- renaming the package
- redesigning the control law
- changing the message schema
- introducing new flight behaviors
- broad parameter-system redesign

## Target Layout

The package keeps the same top-level ROS2 layout and refines its internal structure:

```text
tanh_ctrl/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── flatness_reference.yaml
│   └── tanh_ctrl.yaml
├── docs/
│   └── superpowers/
│       └── specs/
├── include/tanh_ctrl/
│   ├── common.hpp
│   ├── tanh_controller.hpp
│   ├── tanh_node.hpp
│   └── types.hpp
├── launch/
│   ├── demo.launch.py
│   └── tanh_ctrl.launch.py
├── msg/
│   └── FlatTrajectoryReference.msg
├── scripts/
│   └── flatness_reference_publisher.py
├── src/
│   ├── common.cpp
│   ├── tanh_controller.cpp
│   ├── tanh_node.cpp
│   └── tanh_node_main.cpp
└── test/
    ├── test_common.cpp
    ├── test_tanh_controller.cpp
    └── test_tanh_node_logic.cpp
```

## File Mapping

### Files Kept

- `include/tanh_ctrl/types.hpp`
- `config/flatness_reference.yaml`
- `config/tanh_ctrl.yaml`
- `msg/FlatTrajectoryReference.msg`
- `scripts/flatness_reference_publisher.py`
- `launch/tanh_ctrl.launch.py`
- `launch/demo.launch.py`

### Files Replaced

- replace `include/tanh_ctrl/tanh_ctrl.hpp` with `include/tanh_ctrl/tanh_controller.hpp`
- replace `include/tanh_ctrl/tanh_ctrl_node.hpp` with `include/tanh_ctrl/tanh_node.hpp`
- replace `src/tanh_ctrl.cpp` with `src/common.cpp` and `src/tanh_controller.cpp`
- replace `src/tanh_ctrl_node.cpp` with `src/tanh_node.cpp`
- replace `src/tanh_ctrl_node_main.cpp` with `src/tanh_node_main.cpp`
- replace `tests/tanh_ctrl_feedforward_frame_test.cpp` with `test/test_tanh_controller.cpp`

### New Files

- `include/tanh_ctrl/common.hpp`
- `src/common.cpp`
- `test/test_common.cpp`
- `test/test_tanh_node_logic.cpp`

## Component Responsibilities

### `types.hpp`

`types.hpp` becomes the package's pure data contract. It contains only reusable data structures and light inline helpers that are truly type-adjacent. It does not own geometry, allocation, or controller algorithms.

It continues to define:

- `VehicleState`
- `TrajectoryRef`
- `PositionGains`
- `AttitudeGains`
- `AllocationParams`
- `AttitudeReference`
- `ControlOutput`

Any helper left in this file must be trivial and clearly belong to type construction rather than package logic.

### `common.hpp/.cpp`

`common` owns reusable pure functions with no ROS dependencies and no controller state. This is the analog of `px4adrc/common.*` plus the reusable helper portion of the current `tanh_ctrl.cpp`.

Expected responsibilities:

- numeric sanitation helpers
- thrust-model inversion
- thrust-direction to attitude reconstruction
- feedforward frame rotation helpers
- tilt limiting
- motor allocation math
- first-order low-pass helper functions if they are generic enough to be reused cleanly

This layer must not depend on `rclcpp` or ROS messages.

### `tanh_controller.hpp/.cpp`

`TanhController` becomes the pure controller class. It owns:

- vehicle model parameters
- controller gains
- observer/filter states
- `compute(...)`
- position-loop logic
- attitude-loop logic
- filter reset and parameter setters

It may call into `common`, but it must not know about ROS2 node lifecycle, publishers, subscriptions, parameter APIs, or PX4 topic names.

### `tanh_node.hpp/.cpp`

`TanhNode` becomes the ROS2/PX4 adapter layer. It owns:

- parameter declaration and loading
- subscription and publisher creation
- PX4 message validation and translation
- mission-state transitions
- hold-reference generation
- reference freshness checks
- control-loop scheduling
- PX4 command/offboard/setpoint publication

This file is allowed to know ROS2 and PX4 message types, but not to reimplement controller math that belongs in `TanhController` or `common`.

### `tanh_node_main.cpp`

`tanh_node_main.cpp` is intentionally minimal: initialize ROS2, construct the node, spin, and shut down.

## Naming Strategy

The package name stays `tanh_ctrl`, but internal C++ identifiers are normalized to make role boundaries obvious:

- `tanh_ctrl` class becomes `TanhController`
- `tanh_ctrl_node` class becomes `TanhNode`
- helper functions move to snake_case free functions in `common.cpp` unless they are tightly coupled to controller private state

The executable remains `tanh_ctrl_node` so downstream launch usage stays stable.

## Build-System Changes

`CMakeLists.txt` will be reorganized to mirror the reference package:

- `tanh_ctrl_common` static/shared library for `src/common.cpp`
- `tanh_ctrl_core` library for `src/tanh_controller.cpp`
- `tanh_ctrl_node_lib` library for `src/tanh_node.cpp`
- `tanh_ctrl_node` executable for `src/tanh_node_main.cpp`

Link direction is one-way:

- `tanh_ctrl_core` depends on `tanh_ctrl_common`
- `tanh_ctrl_node_lib` depends on `tanh_ctrl_core` and ROS message typesupport
- `tanh_ctrl_node` depends on `tanh_ctrl_node_lib`

Tests link only the layer they are validating.

## Data Flow

Runtime data flow remains the same, but each layer has one job:

1. `TanhNode` receives PX4 odometry, status, IMU acceleration, and flatness reference messages.
2. `TanhNode` validates and translates them into `VehicleState` and `TrajectoryRef`.
3. `TanhNode` selects hold or external reference according to mission state.
4. `TanhNode` calls `TanhController::compute(...)`.
5. `TanhNode` publishes actuator commands, offboard heartbeats, thrust setpoints, and vehicle commands.

This preserves behavior while reducing the amount of unrelated logic inside each translation step.

## Launch and Config Strategy

Launch files will be reduced to the minimum structure used by `px4adrc`:

- compute config paths
- instantiate `Node(...)`
- pass `parameters=[...]`

The package currently requires a source-tree config lookup path so that editing `config/*.yaml` can take effect without rebuilding. That behavior remains, but the logic is minimized and centralized inside launch files rather than pushed into node code.

Planned launch behavior:

- `launch/tanh_ctrl.launch.py` starts only the controller node
- `launch/demo.launch.py` starts the controller node and the flatness reference publisher

No additional launch abstraction is introduced.

## Error Handling

The refactor preserves current operational safeguards:

- invalid odometry position or quaternion delays controller operation instead of using bad state
- invalid reference position clears the external reference
- malformed `model.inertia_diag` falls back to a safe default
- stale references return the node to hold behavior according to current mission logic
- unexpected offboard exit keeps the current shutdown behavior

Any helper extracted during refactor must preserve the same caller-visible behavior. Structural cleanup is not allowed to silently reinterpret frames, signs, or default values.

## Testing Strategy

The test layout is standardized to `test/` and expanded to lock the new boundaries.

### `test/test_common.cpp`

Validates pure helper behavior such as:

- thrust model inversion
- attitude reconstruction from thrust direction and yaw
- frame-rotation helper behavior
- allocation helper invariants that can be checked without ROS

### `test/test_tanh_controller.cpp`

Migrates the current feedforward frame tests and keeps them focused on controller semantics:

- torque feedforward is interpreted in the correct frame
- angular-velocity feedforward is interpreted in the correct frame

### `test/test_tanh_node_logic.cpp`

Validates ROS-free or low-dependency node logic such as:

- reference message validation
- conversion from message to internal reference
- hold-reference generation
- mission-state helper behavior if extracted into testable helpers

## Migration Plan

The implementation will proceed in small, behavior-preserving steps:

1. introduce new headers and source files without deleting old ones immediately
2. move pure helpers from `tanh_ctrl.cpp` into `common`
3. rename and migrate the controller class into `TanhController`
4. rename and migrate the node class into `TanhNode`
5. update `CMakeLists.txt` to build the new target graph
6. migrate tests from `tests/` to `test/` and add the missing focused tests
7. simplify launch files while preserving source-tree config behavior
8. remove superseded old files once the new structure builds and tests pass

This order keeps the package buildable throughout the refactor and limits the chance of mixing structural and behavioral regressions.

## Verification

The refactor is complete only if all of the following are true:

- the package builds with `colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- tests pass with `colcon test --packages-select tanh_ctrl`
- `colcon test-result --verbose` shows no failing test results for `tanh_ctrl`
- launch files remain short and readable
- the package still resolves controller config from the intended source-tree YAML during development
- no public runtime contract listed in the design constraints is broken

## Decision Summary

This design adopts the same architectural grain as `px4adrc` without renaming the package. The main objective is to make `tanh_ctrl` easier to understand and extend by enforcing a clean split between math helpers, controller state, ROS2/PX4 adaptation, and process entrypoints. The refactor is intentionally structural, not behavioral.
