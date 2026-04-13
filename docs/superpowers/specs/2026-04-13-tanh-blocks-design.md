# TanhCtrl Blocks Design

**Date:** 2026-04-13

**Goal:** Add a lightweight `blocks` layer to `tanh_ctrl` that follows the design spirit of `/home/nanwan/work_dir/px4adrc_ws/src/px4adrc/adrc_blocks.*` without over-fragmenting the existing controller. The package keeps its current control-law structure, but controller-internal primitives such as low-pass filtering, rate estimation, and vectorized tanh feedback are factored into explicit reusable block states and free functions.

## Scope

This design adds a new internal layer between `common` and `TanhController`:

- `common` remains the stateless math and geometry helper layer
- `tanh_blocks` becomes the lightweight controller-primitive layer
- `TanhController` remains the place where the full position and attitude control laws are assembled

The package name stays `tanh_ctrl`. This design does not rename the package, change the message schema, rewrite the position or attitude control law, or change the current ROS2/PX4 node behavior.

## Why A Blocks Layer Is Needed

The recent refactor introduced clean file boundaries for:

- stateless math in `common`
- control-law orchestration in `TanhController`
- ROS2/PX4 adaptation in `TanhNode`

That structure is already better than the previous single-file implementation, but the controller still mixes two different responsibilities:

1. full control-law orchestration
2. low-level reusable control primitives such as filtering, differentiation, and tanh feedback shaping

`px4adrc` solves this by giving those lower-level primitives a dedicated `adrc_blocks` layer. The goal here is not to force `tanh_ctrl` into ADRC terminology, but to adopt the same separation idea:

- control primitives should be explicit and reusable
- control-law assembly should remain in the controller

## Design Constraint: Keep The Blocks Layer Small

The `blocks` layer is intentionally lightweight. It is not a second controller and it is not an abstract control framework.

It should not contain:

- the full position loop
- the full attitude loop
- frame semantics such as NED/FRD/reference-body interpretation
- allocation order or thrust/torque orchestration
- parameter loading or ROS2 concepts

It should contain only compact, reusable controller primitives.

This keeps the package close to `px4adrc/adrc_blocks.*`, which uses:

- small state structs
- free functions
- explicit update calls

and avoids the kind of heavy class hierarchy that would add abstraction without helping maintainability.

## Layer Responsibilities

### `common`

`common` stays responsible for stateless math and geometry:

- finite-value sanitation helpers
- thrust-model inversion
- tilt limiting
- desired-attitude reconstruction
- reference-body to current-body vector rotation
- motor allocation and normalized motor-control mapping

These functions remain reusable and stateless.

### `tanh_blocks`

`tanh_blocks` becomes the controller-primitive layer. It contains only:

- compact state structs
- free functions that update those states
- vectorized tanh feedback helpers

This layer is stateful where appropriate, but only at the primitive level.

### `TanhController`

`TanhController` remains the control-law assembly layer. It continues to own:

- position and attitude error definitions
- disturbance-observer state variables that are part of the actual control law
- gravity compensation
- tilt limiting decisions
- attitude-reference construction order
- thrust/torque/motor allocation sequencing
- reference-body feedforward semantics

In other words, `TanhController` becomes cleaner, but not thinner in responsibility.

## Blocks API Style

The API intentionally follows the `px4adrc/adrc_blocks` style:

- use a few small `struct`s
- use free functions for updates and resets
- avoid introducing many classes
- keep block state explicit in the controller

This means `tanh_blocks` should look like a small C++ control-primitives module rather than an object-heavy framework.

## Target Files

### New Files

- `include/tanh_ctrl/tanh_blocks.hpp`
- `src/tanh_blocks.cpp`
- `test/test_tanh_blocks.cpp`

### Modified Files

- `include/tanh_ctrl/tanh_controller.hpp`
- `src/tanh_controller.cpp`
- `CMakeLists.txt`

### Unchanged File Roles

- `include/tanh_ctrl/common.hpp`
- `src/common.cpp`
- `include/tanh_ctrl/tanh_node.hpp`
- `src/tanh_node.cpp`
- `src/tanh_node_main.cpp`

The node layer should not need structural changes beyond normal dependency wiring.

## Proposed `tanh_blocks.hpp`

The blocks layer should expose a minimal interface of small state structs and free functions:

```cpp
#pragma once

#include <Eigen/Dense>

#include <array>

namespace tanh_ctrl {

struct Vec3LowPass
{
  Eigen::Vector3d cutoff_hz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d state{Eigen::Vector3d::Zero()};
  std::array<bool, 3> initialized{{false, false, false}};
};

struct Vec3RateEstimator
{
  Eigen::Vector3d last_value{Eigen::Vector3d::Zero()};
  bool has_last_value{false};
  Vec3LowPass filter{};
};

Eigen::Vector3d tanh_feedback(
  const Eigen::Vector3d & error,
  const Eigen::Vector3d & slope,
  const Eigen::Vector3d & scale);

void reset_low_pass(Vec3LowPass & lpf);
Eigen::Vector3d update_low_pass(
  const Eigen::Vector3d & input,
  double dt,
  Vec3LowPass & lpf);

void reset_rate_estimator(Vec3RateEstimator & estimator);
Eigen::Vector3d update_rate_estimator(
  const Eigen::Vector3d & value,
  double dt,
  Vec3RateEstimator & estimator);

}  // namespace tanh_ctrl
```

## Why Only These Blocks

The blocks layer should stay deliberately narrow.

### `Vec3LowPass`

This block replaces the controller's repeated low-pass state bundles:

- linear-acceleration low-pass state
- velocity-disturbance low-pass state
- angular-velocity-disturbance low-pass state

Using one state struct for all three keeps the behavior explicit and consistent.

### `Vec3RateEstimator`

This block replaces the current combination of:

- `last_angular_velocity_body_`
- `has_last_angular_velocity_`
- angular-acceleration low-pass state

The current controller computes angular acceleration from angular-rate differences and then low-pass filters it. That behavior is exactly the kind of controller primitive that fits a `blocks` layer.

### `tanh_feedback(...)`

The current controller repeatedly computes:

`scale .* tanh(slope .* error)`

That is a clear reusable primitive and should be named explicitly in the blocks layer.

## What Should Not Become Blocks

The following should remain in `TanhController`:

- the position-loop equations
- the attitude-loop equations
- velocity and angular-velocity disturbance observer state variables
- the meaning of `TrajectoryRef::angular_velocity_body` and `TrajectoryRef::torque_body`
- the reference-body to current-body feedforward conversion sequence
- thrust vector assembly and normalization
- the order of control allocation and motor control generation

This is a deliberate design decision. The package needs a `blocks` layer, but not a controller split into dozens of pseudo-generic pieces.

## Controller Rewrite Plan

The controller should be simplified in the following specific way.

### Remove These Private Helper Functions

From `TanhController`, remove:

- `tanhVec(...)`
- `lowPassScalar(...)`
- both overloads of `lowPassVec3(...)`

Those become block-level responsibilities.

### Replace These State Bundles

Current controller state should be rewritten as follows:

- `linear_accel_lpf_cutoff_hz_`, `linear_accel_lpf_state_ned_`, `linear_accel_lpf_initialized_`
  becomes `Vec3LowPass linear_accel_lpf_`

- `velocity_disturbance_lpf_cutoff_hz_`, `velocity_disturbance_lpf_state_ned_`, `velocity_disturbance_lpf_initialized_`
  becomes `Vec3LowPass velocity_disturbance_lpf_`

- `angular_velocity_disturbance_lpf_cutoff_hz_`, `angular_velocity_disturbance_lpf_state_body_`, `angular_velocity_disturbance_lpf_initialized_`
  becomes `Vec3LowPass angular_velocity_disturbance_lpf_`

- `last_angular_velocity_body_`, `has_last_angular_velocity_`, `angular_accel_lpf_cutoff_hz_`, `angular_accel_lpf_state_body_`, `angular_accel_lpf_initialized_`
  becomes `Vec3RateEstimator angular_accel_estimator_`

### Keep These Controller States As-Is

These are not block states; they are controller-law states and should remain explicit:

- `velocity_error_hat_ned_`
- `angular_velocity_error_hat_body_`
- `first_run_` if still needed by controller reset/bootstrapping

### Update The Setters

`TanhController` setters should write into block state rather than separate scalar members:

- `setLinearAccelerationLowPassHz(...)` writes into `linear_accel_lpf_.cutoff_hz`
- `setVelocityDisturbanceLowPassHz(...)` writes into `velocity_disturbance_lpf_.cutoff_hz`
- `setAngularVelocityDisturbanceLowPassHz(...)` writes into `angular_velocity_disturbance_lpf_.cutoff_hz`
- `setAngularAccelerationLowPassHz(...)` writes into `angular_accel_estimator_.filter.cutoff_hz`

Reset should call:

- `reset_low_pass(linear_accel_lpf_)`
- `reset_low_pass(velocity_disturbance_lpf_)`
- `reset_low_pass(angular_velocity_disturbance_lpf_)`
- `reset_rate_estimator(angular_accel_estimator_)`

## Behavior Preservation Requirements

This design explicitly preserves the current controller semantics.

### Feedforward Frame Semantics Must Stay Identical

The current package interprets:

- `TrajectoryRef::angular_velocity_body`
- `TrajectoryRef::torque_body`

as vectors expressed in the reference-body frame, not in the current body frame.

That behavior is then converted inside the inner loop using:

- current body attitude
- desired/reference attitude
- `rotateReferenceBodyVectorToCurrentBody(...)`

The blocks refactor must not change this behavior.

### Filter Semantics Must Stay Identical

The blocks layer is a structural refactor, not a numerical redesign.

That means:

- non-positive cutoff still disables filtering
- invalid input handling still falls back to pass-through or zero exactly as it does today
- first-sample initialization behavior remains the same
- angular acceleration is still estimated by finite difference followed by low-pass filtering

## Testing Strategy

### New Test: `test/test_tanh_blocks.cpp`

This test should cover the blocks layer directly:

- `tanh_feedback(...)` returns expected per-axis output
- `reset_low_pass(...)` clears initialization flags and state
- `update_low_pass(...)` passes through when disabled
- `update_low_pass(...)` initializes correctly on first valid call
- `update_rate_estimator(...)` returns zero on first call and finite-difference output afterwards

These tests should be pure and not depend on ROS.

### Existing Controller Test Stays

`test/test_tanh_controller.cpp` remains the guardrail for controller semantics, especially:

- torque feedforward uses current-body frame after reference-to-current conversion
- angular-velocity feedforward uses current-body frame after reference-to-current conversion

If these tests regress after the blocks refactor, the refactor is incorrect even if the build passes.

### Node Tests Stay Focused

`test/test_tanh_node_logic.cpp` should not absorb blocks testing. The node layer should remain responsible only for node-side logic.

## Build-System Changes

`CMakeLists.txt` should be extended with one additional target:

- `tanh_ctrl_blocks`

Link direction should become:

- `tanh_ctrl_blocks` depends on Eigen only
- `tanh_ctrl_core` depends on `tanh_ctrl_common` and `tanh_ctrl_blocks`
- tests link the narrowest layer they need

This keeps the block layer explicit and independently testable.

## Verification

The blocks design is successfully implemented only if all of the following are true:

- `TanhController` no longer contains its own low-pass helper functions
- the low-level filter/differentiation/tanh primitives are visibly centralized in `tanh_blocks`
- feedforward frame behavior is unchanged
- `test_tanh_blocks`, `test_tanh_controller`, and the rest of the package test suite all pass
- the controller becomes easier to read because primitive maintenance details are removed from the control-law code path

## Decision Summary

This design adds a `blocks` layer in the same architectural spirit as `px4adrc/adrc_blocks`, but at a smaller and more conservative scope suited to `tanh_ctrl`. The package gains an explicit controller-primitives module without turning the control law into a fragmented framework. The end state is:

- `common` for stateless math
- `tanh_blocks` for low-level control primitives
- `TanhController` for control-law orchestration
- `TanhNode` for ROS2/PX4 adaptation

That gives `tanh_ctrl` the missing internal boundary the user asked for, while preserving the semantics that have already been validated in the current codebase.
