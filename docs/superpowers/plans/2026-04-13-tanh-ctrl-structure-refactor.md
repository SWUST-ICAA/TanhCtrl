# TanhCtrl Structure Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure `tanh_ctrl` into `common -> controller -> node -> main` layers that mirror `px4adrc` while keeping the package name, executable name, message definition, runtime behavior, and YAML compatibility intact.

**Architecture:** Extract pure math and frame-conversion helpers into `common`, move controller state and control-law execution into `TanhController`, move ROS2/PX4 adaptation into `TanhNode`, and reduce `main` and launch files to thin entrypoints. Migrate tests to `test/` and use focused targets so each layer can be verified in isolation before the final full-package build.

**Tech Stack:** ROS2 `ament_cmake`, `rclcpp`, `px4_msgs`, `rosidl`, `Eigen3`, `ament_cmake_gtest`, Python launch scripts.

**Execution Context:** Run `git` commands from `/home/nanwan/work_dir/TanhCtrl_ws/src/TanhCtrl`. Run `colcon` commands from `/home/nanwan/work_dir/TanhCtrl_ws`.

---

### Task 1: Establish The New Test/Build Scaffold And Common Layer

**Files:**
- Create: `include/tanh_ctrl/common.hpp`
- Create: `src/common.cpp`
- Create: `test/test_common.cpp`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

- [ ] **Step 1: Write the failing common-layer test**

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "tanh_ctrl/common.hpp"

namespace tanh_ctrl {
namespace {

TEST(Common, planar_axis_vec_repeats_planar_value)
{
  const Eigen::Vector3d gains = planar_axis_vec(2.5, 1.0);
  EXPECT_DOUBLE_EQ(gains.x(), 2.5);
  EXPECT_DOUBLE_EQ(gains.y(), 2.5);
  EXPECT_DOUBLE_EQ(gains.z(), 1.0);
}

TEST(Common, throttle_from_relative_thrust_is_identity_when_factor_is_zero)
{
  EXPECT_DOUBLE_EQ(throttle_from_relative_thrust(0.35, 0.0), 0.35);
}

TEST(Common, rotates_reference_body_vector_into_current_body_frame)
{
  const Eigen::Quaterniond current_body_to_ned(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond reference_body_to_ned = Eigen::Quaterniond::Identity();

  const Eigen::Vector3d rotated = rotate_reference_body_vector_to_current_body(
    current_body_to_ned, reference_body_to_ned, Eigen::Vector3d::UnitX());

  EXPECT_NEAR(rotated.x(), 0.0, 1e-9);
  EXPECT_NEAR(rotated.y(), -1.0, 1e-9);
  EXPECT_NEAR(rotated.z(), 0.0, 1e-9);
}

}  // namespace
}  // namespace tanh_ctrl
```

- [ ] **Step 2: Hook the new test into the build so it can fail for the right reason**

```cmake
find_package(ament_cmake_gtest REQUIRED)

add_library(tanh_ctrl_common src/common.cpp)
target_link_libraries(tanh_ctrl_common Eigen3::Eigen)
target_include_directories(tanh_ctrl_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  ament_add_gtest(test_common test/test_common.cpp)
  target_link_libraries(test_common tanh_ctrl_common)
  target_include_directories(test_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

install(TARGETS tanh_ctrl_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

- [ ] **Step 3: Run the build to verify the test fails before the implementation exists**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: FAIL with a compile error such as `tanh_ctrl/common.hpp: No such file or directory` or undefined common-layer symbols.

- [ ] **Step 4: Add the minimal `common` interface and implementation to satisfy the test**

```cpp
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

Eigen::Vector3d planar_axis_vec(double planar, double axial);
double sanitize_scalar(double value);
Eigen::Vector3d sanitize_vec3(const Eigen::Vector3d & value);
double throttle_from_relative_thrust(double relative_thrust, double thrust_model_factor);
Eigen::Quaterniond quaternion_from_thrust_direction_yaw_ned(
  const Eigen::Vector3d & thrust_direction_ned, double yaw_rad);
Eigen::Vector3d rotate_reference_body_vector_to_current_body(
  const Eigen::Quaterniond & current_body_to_ned,
  const Eigen::Quaterniond & reference_body_to_ned,
  const Eigen::Vector3d & reference_body_vector);
void apply_tilt_limit(Eigen::Vector3d * thrust_vector_over_mass_ned, double max_tilt_rad);
Eigen::Vector4d quad_x_allocate(
  const AllocationParams & params, double thrust_total, const Eigen::Vector3d & torque_body);
Eigen::Vector4d motor_forces_to_controls(
  const Eigen::Vector4d & forces, double motor_force_max, double thrust_model_factor);

}  // namespace tanh_ctrl
```

```cpp
#include "tanh_ctrl/common.hpp"

#include <algorithm>
#include <cmath>

namespace tanh_ctrl {

namespace {
constexpr double kMinPositiveZ = 1e-3;
constexpr double kSmallNorm = 1e-9;
constexpr double kMinCollectiveThrust = 1e-6;
}

Eigen::Vector3d planar_axis_vec(double planar, double axial)
{
  const double planar_value = std::isfinite(planar) ? planar : 0.0;
  const double axial_value = std::isfinite(axial) ? axial : planar_value;
  return Eigen::Vector3d(planar_value, planar_value, axial_value);
}

double sanitize_scalar(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

Eigen::Vector3d sanitize_vec3(const Eigen::Vector3d & value)
{
  return value.allFinite() ? value : Eigen::Vector3d::Zero();
}

double throttle_from_relative_thrust(double relative_thrust, double thrust_model_factor)
{
  const double rel_thrust = std::clamp(relative_thrust, 0.0, 1.0);
  const double factor = std::clamp(thrust_model_factor, 0.0, 1.0);
  if (factor <= 1e-9) {
    return rel_thrust;
  }

  const double a = factor;
  const double b = 1.0 - factor;
  const double tmp1 = b / (2.0 * a);
  const double tmp2 = (b * b) / (4.0 * a * a);
  return std::clamp(-tmp1 + std::sqrt(tmp2 + rel_thrust / a), 0.0, 1.0);
}

Eigen::Vector3d rotate_reference_body_vector_to_current_body(
  const Eigen::Quaterniond & current_body_to_ned,
  const Eigen::Quaterniond & reference_body_to_ned,
  const Eigen::Vector3d & reference_body_vector)
{
  if (!reference_body_vector.allFinite()) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Quaterniond reference_body_to_current_body =
    current_body_to_ned.conjugate() * reference_body_to_ned;
  return reference_body_to_current_body * reference_body_vector;
}

}  // namespace tanh_ctrl
```

- [ ] **Step 5: Rebuild and run the common-layer test**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select tanh_ctrl --ctest-args -R test_common --output-on-failure
colcon test-result --verbose
```

Expected: `test_common` PASS and no failing result entry for the package.

- [ ] **Step 6: Commit the scaffold and common layer**

```bash
git add CMakeLists.txt package.xml include/tanh_ctrl/common.hpp src/common.cpp test/test_common.cpp
git commit -m "refactor: add tanh_ctrl common layer scaffold"
```

### Task 2: Split The Pure Controller Into `TanhController`

**Files:**
- Create: `include/tanh_ctrl/tanh_controller.hpp`
- Create: `src/tanh_controller.cpp`
- Create: `test/test_tanh_controller.cpp`
- Modify: `include/tanh_ctrl/types.hpp`
- Modify: `CMakeLists.txt`
- Delete: `include/tanh_ctrl/tanh_ctrl.hpp`
- Delete: `src/tanh_ctrl.cpp`

- [ ] **Step 1: Write the failing controller test by migrating the existing feedforward-frame checks**

```cpp
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "tanh_ctrl/tanh_controller.hpp"

namespace tanh_ctrl {
namespace {

PositionGains zero_position_gains()
{
  PositionGains gains;
  gains.M_P.setZero();
  gains.K_P.setZero();
  gains.M_V.setZero();
  gains.K_V.setZero();
  gains.K_Acceleration.setZero();
  gains.P_V.setZero();
  gains.L_V.setOnes();
  return gains;
}

AttitudeGains zero_attitude_gains()
{
  AttitudeGains gains;
  gains.M_Angle.setZero();
  gains.K_Angle.setZero();
  gains.M_AngularVelocity.setZero();
  gains.K_AngularVelocity.setZero();
  gains.K_AngularAcceleration.setZero();
  gains.P_AngularVelocity.setZero();
  gains.L_AngularVelocity.setOnes();
  return gains;
}

TanhController make_controller()
{
  TanhController controller;
  controller.setMass(1.0);
  controller.setGravity(9.81);
  controller.setInertia(Eigen::Matrix3d::Identity());
  controller.setPositionGains(zero_position_gains());
  controller.setAttitudeGains(zero_attitude_gains());
  controller.setMotorForceMax(100.0);
  controller.setThrustModelFactor(0.0);
  controller.reset();
  return controller;
}

TEST(TanhController, torque_feedforward_uses_current_body_frame)
{
  TanhController controller = make_controller();

  TrajectoryRef ref;
  ref.valid = true;
  ref.torque_body = Eigen::Vector3d::UnitX();
  ref.has_torque_feedforward = true;

  VehicleState state;
  state.q_body_to_ned = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

  ControlOutput out;
  ASSERT_TRUE(controller.compute(state, ref, 0.01, &out));
  EXPECT_NEAR(out.torque_body.x(), 0.0, 1e-6);
  EXPECT_NEAR(out.torque_body.y(), -1.0, 1e-6);
  EXPECT_NEAR(out.torque_body.z(), 0.0, 1e-6);
}

}  // namespace
}  // namespace tanh_ctrl
```

- [ ] **Step 2: Register the new core library and controller test**

```cmake
add_library(tanh_ctrl_core src/tanh_controller.cpp)
target_link_libraries(tanh_ctrl_core Eigen3::Eigen tanh_ctrl_common)
target_include_directories(tanh_ctrl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  ament_add_gtest(test_tanh_controller test/test_tanh_controller.cpp)
  target_link_libraries(test_tanh_controller tanh_ctrl_core)
  target_include_directories(test_tanh_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

install(TARGETS tanh_ctrl_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: Run the build to confirm the new controller test fails before the class exists**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: FAIL with `tanh_ctrl/tanh_controller.hpp: No such file or directory` or `TanhController` not declared.

- [ ] **Step 4: Introduce `TanhController` and make `types.hpp` data-only**

```cpp
#pragma once

#include <Eigen/Dense>

#include <array>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

class TanhController
{
public:
  TanhController();

  void setMass(double mass);
  void setGravity(double gravity);
  void setPositionGains(const PositionGains & gains);
  void setAttitudeGains(const AttitudeGains & gains);
  void setInertia(const Eigen::Matrix3d & inertia);
  void setAllocationParams(const AllocationParams & alloc);
  void setMotorForceMax(double max_force);
  void setThrustModelFactor(double thrust_model_factor);
  void setMaxTilt(double max_tilt_rad);
  void setLinearAccelerationLowPassHz(const Eigen::Vector3d & cutoff_hz);
  void setAngularAccelerationLowPassHz(double cutoff_hz);
  void setVelocityDisturbanceLowPassHz(double cutoff_hz);
  void setAngularVelocityDisturbanceLowPassHz(double cutoff_hz);
  void reset();
  bool compute(const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out);

  const PositionGains & getPositionGains() const { return pos_gains_; }
  const AllocationParams & getAllocationParams() const { return alloc_; }

private:
  static double sanitizeCutoff(double cutoff_hz);
  static Eigen::Vector3d sanitizeCutoff(const Eigen::Vector3d & cutoff_hz);
  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, double cutoff_hz, double dt,
    Eigen::Vector3d * state, bool * initialized);
  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, const Eigen::Vector3d & cutoff_hz, double dt,
    Eigen::Vector3d * state, std::array<bool, 3> * initialized);
  static double lowPassScalar(
    double x, double cutoff_hz, double dt, double * state, bool * initialized);

  void computePosition(
    const VehicleState & state, const TrajectoryRef & ref, double dt,
    Eigen::Vector3d * thrust_vec_ned, double * thrust_norm);
  void computeAttitude(
    const VehicleState & state, const AttitudeReference & attitude_reference, double dt,
    Eigen::Vector3d * torque_body);

  double mass_{1.0};
  double gravity_{9.81};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  PositionGains pos_gains_{};
  AttitudeGains att_gains_{};
  AllocationParams alloc_{};
  double motor_force_max_{10.0};
  double thrust_model_factor_{1.0};
  double max_tilt_rad_{0.0};
  Eigen::Vector3d velocity_error_hat_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_error_hat_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_angular_velocity_body_{Eigen::Vector3d::Zero()};
  bool has_last_angular_velocity_{false};
  bool first_run_{true};
  Eigen::Vector3d linear_accel_lpf_cutoff_hz_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_accel_lpf_state_ned_{Eigen::Vector3d::Zero()};
  std::array<bool, 3> linear_accel_lpf_initialized_{{false, false, false}};
  double angular_accel_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d angular_accel_lpf_state_body_{Eigen::Vector3d::Zero()};
  bool angular_accel_lpf_initialized_{false};
  double velocity_disturbance_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d velocity_disturbance_lpf_state_ned_{Eigen::Vector3d::Zero()};
  bool velocity_disturbance_lpf_initialized_{false};
  double angular_velocity_disturbance_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d angular_velocity_disturbance_lpf_state_body_{Eigen::Vector3d::Zero()};
  bool angular_velocity_disturbance_lpf_initialized_{false};
};

}  // namespace tanh_ctrl
```

```cpp
#include "tanh_ctrl/tanh_controller.hpp"

#include <algorithm>
#include <cmath>

#include "tanh_ctrl/common.hpp"

namespace tanh_ctrl {

TanhController::TanhController() = default;

bool TanhController::compute(
  const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out)
{
  if (!out || !ref.valid) {
    return false;
  }

  Eigen::Vector3d thrust_vec_ned = Eigen::Vector3d::Zero();
  double thrust_norm = 0.0;
  computePosition(state, ref, dt, &thrust_vec_ned, &thrust_norm);

  AttitudeReference attitude_reference;
  attitude_reference.collective_thrust = thrust_norm;
  attitude_reference.thrust_direction_ned =
    thrust_norm > 1e-6 ? thrust_vec_ned / thrust_norm : Eigen::Vector3d::UnitZ();
  attitude_reference.attitude_body_to_ned =
    quaternion_from_thrust_direction_yaw_ned(attitude_reference.thrust_direction_ned, sanitize_scalar(ref.yaw));
  attitude_reference.angular_velocity_body =
    ref.has_angular_velocity_feedforward ? ref.angular_velocity_body : Eigen::Vector3d::Zero();
  attitude_reference.torque_body =
    ref.has_torque_feedforward ? ref.torque_body : Eigen::Vector3d::Zero();
  attitude_reference.has_angular_velocity_feedforward = ref.has_angular_velocity_feedforward;
  attitude_reference.has_torque_feedforward = ref.has_torque_feedforward;
  attitude_reference.valid = true;

  computeAttitude(state, attitude_reference, dt, &out->torque_body);
  out->thrust_total = thrust_norm;
  out->motor_forces = quad_x_allocate(alloc_, out->thrust_total, out->torque_body);
  out->motor_controls =
    motor_forces_to_controls(out->motor_forces, motor_force_max_, thrust_model_factor_);
  return true;
}

}  // namespace tanh_ctrl
```

```cpp
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tanh_ctrl {

struct VehicleState
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
};

struct TrajectoryRef
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  bool has_angular_velocity_feedforward{false};
  bool has_torque_feedforward{false};
  bool valid{false};
};

}  // namespace tanh_ctrl
```

- [ ] **Step 5: Rebuild and run the controller test**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select tanh_ctrl --ctest-args -R test_tanh_controller --output-on-failure
colcon test-result --verbose
```

Expected: `test_tanh_controller` PASS and the migrated frame-behavior assertions stay green.

- [ ] **Step 6: Commit the controller split**

```bash
git add CMakeLists.txt include/tanh_ctrl/types.hpp include/tanh_ctrl/tanh_controller.hpp src/tanh_controller.cpp test/test_tanh_controller.cpp
git rm include/tanh_ctrl/tanh_ctrl.hpp src/tanh_ctrl.cpp
git commit -m "refactor: split tanh controller core"
```

### Task 3: Split ROS2/PX4 Adaptation Into `TanhNode`

**Files:**
- Create: `include/tanh_ctrl/tanh_node.hpp`
- Create: `src/tanh_node.cpp`
- Create: `src/tanh_node_main.cpp`
- Create: `test/test_tanh_node_logic.cpp`
- Modify: `CMakeLists.txt`
- Delete: `include/tanh_ctrl/tanh_ctrl_node.hpp`
- Delete: `src/tanh_ctrl_node.cpp`
- Delete: `src/tanh_ctrl_node_main.cpp`

- [ ] **Step 1: Write the failing node-logic test around reference conversion and hold-reference creation**

```cpp
#include <gtest/gtest.h>

#include <limits>

#include "tanh_ctrl/tanh_node.hpp"

namespace tanh_ctrl {
namespace {

TEST(TanhNodeLogic, reference_message_requires_finite_position)
{
  msg::FlatTrajectoryReference ref_msg;
  ref_msg.position_ned.x = std::numeric_limits<float>::quiet_NaN();
  ref_msg.position_ned.y = 0.0f;
  ref_msg.position_ned.z = -2.0f;

  EXPECT_FALSE(reference_message_has_valid_position(ref_msg));
}

TEST(TanhNodeLogic, converts_reference_message_into_internal_reference)
{
  msg::FlatTrajectoryReference ref_msg;
  ref_msg.position_ned.x = 1.0f;
  ref_msg.position_ned.y = 2.0f;
  ref_msg.position_ned.z = -3.0f;
  ref_msg.velocity_ned.x = 0.1f;
  ref_msg.velocity_ned.y = 0.2f;
  ref_msg.velocity_ned.z = 0.3f;
  ref_msg.yaw = 0.4f;

  const TrajectoryRef ref = trajectory_reference_from_msg(ref_msg, 42);

  EXPECT_TRUE(ref.valid);
  EXPECT_DOUBLE_EQ(ref.position_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.y(), 2.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.z(), -3.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.x(), 0.1);
  EXPECT_DOUBLE_EQ(ref.yaw, 0.4);
}

TEST(TanhNodeLogic, hold_reference_keeps_xy_and_replaces_target_z)
{
  VehicleState state;
  state.position_ned = Eigen::Vector3d(4.0, -1.0, -0.5);

  const TrajectoryRef hold = make_hold_reference(state, -2.0, 0.25);

  EXPECT_TRUE(hold.valid);
  EXPECT_DOUBLE_EQ(hold.position_ned.x(), 4.0);
  EXPECT_DOUBLE_EQ(hold.position_ned.y(), -1.0);
  EXPECT_DOUBLE_EQ(hold.position_ned.z(), -2.0);
  EXPECT_DOUBLE_EQ(hold.yaw, 0.25);
}

}  // namespace
}  // namespace tanh_ctrl
```

- [ ] **Step 2: Register the node library, node executable, and node-logic test**

```cmake
add_library(tanh_ctrl_node_lib src/tanh_node.cpp)
target_link_libraries(tanh_ctrl_node_lib tanh_ctrl_core "${cpp_typesupport_target}")
ament_target_dependencies(tanh_ctrl_node_lib geometry_msgs rclcpp px4_msgs std_msgs)
target_include_directories(tanh_ctrl_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tanh_ctrl_node src/tanh_node_main.cpp)
target_link_libraries(tanh_ctrl_node tanh_ctrl_node_lib)
ament_target_dependencies(tanh_ctrl_node rclcpp)
target_include_directories(tanh_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  ament_add_gtest(test_tanh_node_logic test/test_tanh_node_logic.cpp)
  target_link_libraries(test_tanh_node_logic tanh_ctrl_node_lib)
  target_include_directories(test_tanh_node_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()
```

- [ ] **Step 3: Run the build to verify the node-logic test fails before the new node interface exists**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: FAIL with `tanh_ctrl/tanh_node.hpp: No such file or directory` or missing node-helper declarations.

- [ ] **Step 4: Introduce `TanhNode`, the helper functions, and the minimal main**

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tanh_ctrl/msg/flat_trajectory_reference.hpp"
#include "tanh_ctrl/tanh_controller.hpp"

namespace tanh_ctrl {

enum class MissionState
{
  WAIT_FOR_OFFBOARD,
  WAIT_FOR_ARMING,
  TAKEOFF,
  HOLD,
  TRACKING,
};

const char * to_string(MissionState state);
bool reference_message_has_valid_position(const msg::FlatTrajectoryReference & msg);
TrajectoryRef trajectory_reference_from_msg(
  const msg::FlatTrajectoryReference & msg, uint64_t timestamp_us);
TrajectoryRef make_hold_reference(const VehicleState & state, double target_z_ned, double yaw);

class TanhNode : public rclcpp::Node
{
public:
  explicit TanhNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParams();
  void createRosInterfaces();
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void accelCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void referenceCallback(const msg::FlatTrajectoryReference::SharedPtr msg);
  void controlLoop();
  void publishOffboardControlMode(uint64_t now_us);
  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);
  void publishStartTrackingSignal(bool enabled);

  TanhController controller_;
  VehicleState state_{};
  TrajectoryRef hold_ref_{};
  TrajectoryRef external_ref_{};
};

}  // namespace tanh_ctrl
```

```cpp
#include "tanh_ctrl/tanh_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "tanh_ctrl/common.hpp"

namespace tanh_ctrl {

bool reference_message_has_valid_position(const msg::FlatTrajectoryReference & msg)
{
  return std::isfinite(msg.position_ned.x) &&
         std::isfinite(msg.position_ned.y) &&
         std::isfinite(msg.position_ned.z);
}

TrajectoryRef trajectory_reference_from_msg(
  const msg::FlatTrajectoryReference & msg, uint64_t)
{
  TrajectoryRef ref;
  if (!reference_message_has_valid_position(msg)) {
    return ref;
  }

  ref.position_ned = Eigen::Vector3d(msg.position_ned.x, msg.position_ned.y, msg.position_ned.z);
  ref.velocity_ned = Eigen::Vector3d(
    sanitize_scalar(msg.velocity_ned.x),
    sanitize_scalar(msg.velocity_ned.y),
    sanitize_scalar(msg.velocity_ned.z));
  ref.acceleration_ned = Eigen::Vector3d(
    sanitize_scalar(msg.acceleration_ned.x),
    sanitize_scalar(msg.acceleration_ned.y),
    sanitize_scalar(msg.acceleration_ned.z));
  const bool has_body_rates =
    std::isfinite(msg.body_rates_frd.x) &&
    std::isfinite(msg.body_rates_frd.y) &&
    std::isfinite(msg.body_rates_frd.z);
  const bool has_body_torque =
    std::isfinite(msg.body_torque_frd.x) &&
    std::isfinite(msg.body_torque_frd.y) &&
    std::isfinite(msg.body_torque_frd.z);
  ref.angular_velocity_body = Eigen::Vector3d(
    sanitize_scalar(msg.body_rates_frd.x),
    sanitize_scalar(msg.body_rates_frd.y),
    sanitize_scalar(msg.body_rates_frd.z));
  ref.torque_body = Eigen::Vector3d(
    sanitize_scalar(msg.body_torque_frd.x),
    sanitize_scalar(msg.body_torque_frd.y),
    sanitize_scalar(msg.body_torque_frd.z));
  ref.yaw = sanitize_scalar(msg.yaw);
  ref.has_angular_velocity_feedforward = has_body_rates;
  ref.has_torque_feedforward = has_body_torque;
  ref.valid = true;
  return ref;
}

TrajectoryRef make_hold_reference(const VehicleState & state, double target_z_ned, double yaw)
{
  TrajectoryRef hold_ref;
  hold_ref.position_ned = Eigen::Vector3d(
    state.position_ned.x(), state.position_ned.y(), target_z_ned);
  hold_ref.yaw = yaw;
  hold_ref.valid = true;
  return hold_ref;
}

TanhNode::TanhNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tanh_ctrl", options)
{
  declareParameters();
  loadParams();
  createRosInterfaces();
}

}  // namespace tanh_ctrl
```

```cpp
#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/tanh_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanh_ctrl::TanhNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: Rebuild and run the node-logic test**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select tanh_ctrl --ctest-args -R test_tanh_node_logic --output-on-failure
colcon test-result --verbose
```

Expected: `test_tanh_node_logic` PASS and no reference-conversion or hold-reference regressions.

- [ ] **Step 6: Commit the node split**

```bash
git add CMakeLists.txt include/tanh_ctrl/tanh_node.hpp src/tanh_node.cpp src/tanh_node_main.cpp test/test_tanh_node_logic.cpp
git rm include/tanh_ctrl/tanh_ctrl_node.hpp src/tanh_ctrl_node.cpp src/tanh_ctrl_node_main.cpp
git commit -m "refactor: split tanh ros node adapter"
```

### Task 4: Simplify Launch Files And Normalize Source-Tree Config Lookup

**Files:**
- Modify: `launch/tanh_ctrl.launch.py`
- Modify: `launch/demo.launch.py`

- [ ] **Step 1: Rewrite `launch/tanh_ctrl.launch.py` as a minimal controller-only entrypoint**

```python
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _package_source_root() -> Path:
    launch_file = Path(__file__).resolve()
    package_root = launch_file.parents[1]
    if (package_root / "CMakeLists.txt").exists():
        return package_root

    install_root = next((parent for parent in launch_file.parents if parent.name == "install"), None)
    if install_root is None:
        return package_root

    candidate = install_root.parent / "src" / "TanhCtrl"
    if (candidate / "config" / "tanh_ctrl.yaml").exists():
        return candidate

    return package_root


def generate_launch_description():
    package_root = _package_source_root()
    controller_config = package_root / "config" / "tanh_ctrl.yaml"

    return LaunchDescription([
        Node(
            package="tanh_ctrl",
            executable="tanh_ctrl_node",
            name="tanh_ctrl",
            output="screen",
            parameters=[str(controller_config)],
        ),
    ])
```

- [ ] **Step 2: Rewrite `launch/demo.launch.py` as a minimal controller-plus-reference entrypoint**

```python
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _package_source_root() -> Path:
    launch_file = Path(__file__).resolve()
    package_root = launch_file.parents[1]
    if (package_root / "CMakeLists.txt").exists():
        return package_root

    install_root = next((parent for parent in launch_file.parents if parent.name == "install"), None)
    if install_root is None:
        return package_root

    candidate = install_root.parent / "src" / "TanhCtrl"
    if (candidate / "config" / "tanh_ctrl.yaml").exists():
        return candidate

    return package_root


def generate_launch_description():
    package_root = _package_source_root()
    controller_config = package_root / "config" / "tanh_ctrl.yaml"
    reference_config = package_root / "config" / "flatness_reference.yaml"

    return LaunchDescription([
        Node(
            package="tanh_ctrl",
            executable="tanh_ctrl_node",
            name="tanh_ctrl",
            output="screen",
            parameters=[str(controller_config)],
        ),
        Node(
            package="tanh_ctrl",
            executable="flatness_reference_publisher.py",
            name="flatness_reference_publisher",
            output="screen",
            parameters=[str(reference_config)],
        ),
    ])
```

- [ ] **Step 3: Run a syntax smoke check on the launch files**

Run:
```bash
python3 -m py_compile src/TanhCtrl/launch/tanh_ctrl.launch.py src/TanhCtrl/launch/demo.launch.py
```

Expected: no output and exit code `0`.

- [ ] **Step 4: Commit the launch cleanup**

```bash
git add launch/tanh_ctrl.launch.py launch/demo.launch.py
git commit -m "refactor: simplify tanh launch entrypoints"
```

### Task 5: Remove The Old Layout And Run Full Verification

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `install` rules that still mention replaced targets or old test paths
- Delete: `tests/tanh_ctrl_feedforward_frame_test.cpp`
- Delete: any stale references to `tanh_ctrl.hpp`, `tanh_ctrl_node.hpp`, `src/tanh_ctrl.cpp`, `src/tanh_ctrl_node.cpp`, or `tests/`

- [ ] **Step 1: Remove the old test path and stale target references**

```cmake
if(BUILD_TESTING)
  ament_add_gtest(test_common test/test_common.cpp)
  ament_add_gtest(test_tanh_controller test/test_tanh_controller.cpp)
  ament_add_gtest(test_tanh_node_logic test/test_tanh_node_logic.cpp)
endif()

install(TARGETS tanh_ctrl_common tanh_ctrl_core tanh_ctrl_node_lib tanh_ctrl_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

```bash
git rm tests/tanh_ctrl_feedforward_frame_test.cpp
```

- [ ] **Step 2: Verify no stale include or source references remain**

Run:
```bash
rg -n "tanh_ctrl.hpp|tanh_ctrl_node.hpp|src/tanh_ctrl.cpp|src/tanh_ctrl_node.cpp|tests/" CMakeLists.txt include src test launch
```

Expected: no matches.

- [ ] **Step 3: Run the full package build**

Run:
```bash
colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: PASS and `tanh_ctrl` finishes building with the new target graph.

- [ ] **Step 4: Run the full package test suite**

Run:
```bash
colcon test --packages-select tanh_ctrl
colcon test-result --verbose
```

Expected: PASS for `test_common`, `test_tanh_controller`, and `test_tanh_node_logic`; no remaining reference to the deleted `tests/` path.

- [ ] **Step 5: Commit the final cleanup**

```bash
git add CMakeLists.txt test
git commit -m "refactor: finish tanh_ctrl layout normalization"
```

- [ ] **Step 6: Record the exact verification summary in the final handoff**

```text
Verified:
- colcon build --packages-select tanh_ctrl --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
- colcon test --packages-select tanh_ctrl
- colcon test-result --verbose
```
