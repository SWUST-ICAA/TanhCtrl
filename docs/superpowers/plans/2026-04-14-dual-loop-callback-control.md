# Dual-Loop Callback Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace timer-driven single-loop control with callback-driven dual-loop control and remove `control_rate_hz`.

**Architecture:** Split `TanhController` into independently callable outer-loop and inner-loop stages. Make `TanhNode` cache state from three PX4 topics and trigger outer/inner control from `vehicle_odometry` and `vehicle_angular_velocity` respectively.

**Tech Stack:** ROS2 `rclcpp`, `px4_msgs`, Eigen, `ament_cmake_gtest`

---

### Task 1: Add failing unit tests for split-loop controller support

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Create: `test/test_tanh_controller_split.cpp`

- [ ] **Step 1: Write the failing test**

Add tests that call `computePositionLoop()`, `computeAttitudeLoop()`, and the timestamp-driven `computeLoopDtFromSample()` helper before those APIs exist.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake -S /home/nanwan/work_dir/TanhCtrl_ws/src/TanhCtrl -B /tmp/tanh_ctrl_build && cmake --build /tmp/tanh_ctrl_build --target test_tanh_controller_split`
Expected: build failure due to missing split-loop APIs.

- [ ] **Step 3: Write minimal implementation**

Expose split-loop controller methods and the timestamp helper, then wire tests into `ament_cmake_gtest`.

- [ ] **Step 4: Run test to verify it passes**

Run: `ctest --test-dir /tmp/tanh_ctrl_build --output-on-failure`
Expected: the new unit tests pass.

### Task 2: Convert node scheduling from timer-driven to callback-driven

**Files:**
- Modify: `include/tanh_ctrl/tanh_node.hpp`
- Modify: `src/tanh_node.cpp`
- Modify: `config/tanh_ctrl.yaml`

- [ ] **Step 1: Remove timer-based control state**

Delete `control_rate_hz`, timer members, and timer setup. Add separate timestamp tracking and cached outer-loop command state.

- [ ] **Step 2: Switch subscriptions to the new PX4 state sources**

Subscribe to `vehicle_attitude` and `vehicle_angular_velocity`, and stop reading attitude/body-rates from odometry.

- [ ] **Step 3: Implement callback-driven outer and inner loops**

Run mission/reference logic and outer-loop updates from odometry callbacks. Run offboard streaming, automatic requests, inner-loop control, and actuator publication from angular-velocity callbacks.

- [ ] **Step 4: Verify node builds**

Run: `cmake --build /tmp/tanh_ctrl_build`
Expected: node library and executable build successfully.
