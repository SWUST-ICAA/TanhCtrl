#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/tanh_ctrl_node.hpp"

namespace {

class TanhCtrlNodeTopicsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_LOG_DIR", "/tmp", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TanhCtrlNodeTopicsTest, IgnoresOverridesForFixedPx4Topics)
{
  const std::vector<rclcpp::Parameter> overrides = {
    {"topics.reference", "/override/tanh_ctrl/reference"},
    {"topics.sensor_combined", "/override/out/sensor_combined"},
    {"topics.vehicle_odometry", "/override/out/vehicle_odometry"},
    {"topics.vehicle_status_v1", "/override/out/vehicle_status_v1"},
    {"topics.actuator_motors", "/override/in/actuator_motors"},
    {"topics.offboard_control_mode", "/override/in/offboard_control_mode"},
    {"topics.vehicle_command", "/override/in/vehicle_command"},
    {"topics.vehicle_thrust_setpoint", "/override/in/vehicle_thrust_setpoint"},
    {"publish_offboard_control_mode", false},
    {"publish_vehicle_thrust_setpoint", false},
    {"auto_offboard", false},
    {"auto_arm", true},
    {"offboard_warmup", 99},
    {"allocation.beta", 1.2345},
    {"model.force_max", 9.9},
    {"motor.force_max", 3.3},
    {"motor.output_map", std::vector<int64_t>{3, 2, 1, 0}},
  };

  const auto options = rclcpp::NodeOptions().parameter_overrides(overrides);
  auto controller = std::make_shared<tanh_ctrl::tanh_ctrl_node>(options);

  EXPECT_EQ(controller->sensorCombinedTopic(), "/fmu/out/sensor_combined");
  EXPECT_EQ(controller->vehicleOdometryTopic(), "/fmu/out/vehicle_odometry");
  EXPECT_EQ(controller->vehicleStatusV1Topic(), "/fmu/out/vehicle_status_v1");
  EXPECT_EQ(controller->referenceTopic(), "/override/tanh_ctrl/reference");
  EXPECT_EQ(controller->actuatorMotorsTopic(), "/fmu/in/actuator_motors");
  EXPECT_EQ(controller->offboardControlModeTopic(), "/fmu/in/offboard_control_mode");
  EXPECT_EQ(controller->vehicleCommandTopic(), "/fmu/in/vehicle_command");
  EXPECT_EQ(controller->vehicleThrustSetpointTopic(), "/fmu/in/vehicle_thrust_setpoint");
  EXPECT_TRUE(controller->publishOffboardControlModeEnabled());
  EXPECT_TRUE(controller->publishVehicleThrustSetpointEnabled());
  EXPECT_TRUE(controller->autoOffboardEnabled());
  EXPECT_FALSE(controller->autoArmEnabled());
  EXPECT_EQ(controller->offboardWarmupSetpointCount(), 10);
  EXPECT_DOUBLE_EQ(controller->allocationBeta(), 0.78539816339);
  EXPECT_DOUBLE_EQ(controller->motorForceMax(), 9.9);
  EXPECT_EQ(controller->motorOutputMap(), (std::array<int, 4>{1, 3, 0, 2}));

  EXPECT_FALSE(controller->has_parameter("topics.sensor_combined"));
  EXPECT_FALSE(controller->has_parameter("topics.vehicle_odometry"));
  EXPECT_FALSE(controller->has_parameter("topics.vehicle_status_v1"));
  EXPECT_FALSE(controller->has_parameter("topics.trajectory_setpoint"));
  EXPECT_FALSE(controller->has_parameter("topics.actuator_motors"));
  EXPECT_FALSE(controller->has_parameter("topics.offboard_control_mode"));
  EXPECT_FALSE(controller->has_parameter("topics.vehicle_command"));
  EXPECT_FALSE(controller->has_parameter("topics.vehicle_thrust_setpoint"));
  EXPECT_FALSE(controller->has_parameter("publish_offboard_control_mode"));
  EXPECT_FALSE(controller->has_parameter("publish_vehicle_thrust_setpoint"));
  EXPECT_FALSE(controller->has_parameter("auto_offboard"));
  EXPECT_FALSE(controller->has_parameter("auto_arm"));
  EXPECT_FALSE(controller->has_parameter("offboard_warmup"));
  EXPECT_FALSE(controller->has_parameter("allocation.beta"));
  EXPECT_FALSE(controller->has_parameter("motor.force_max"));
  EXPECT_FALSE(controller->has_parameter("motor.output_map"));
  EXPECT_FALSE(controller->has_parameter("position.M_P"));
  EXPECT_FALSE(controller->has_parameter("position.K_P"));
  EXPECT_FALSE(controller->has_parameter("position.M_V"));
  EXPECT_FALSE(controller->has_parameter("position.K_V"));
  EXPECT_FALSE(controller->has_parameter("position.K_Acceleration"));
  EXPECT_FALSE(controller->has_parameter("filters.linear_accel_cutoff_hz"));
  EXPECT_TRUE(controller->has_parameter("model.force_max"));
  EXPECT_TRUE(controller->has_parameter("topics.reference"));
}

}  // namespace
