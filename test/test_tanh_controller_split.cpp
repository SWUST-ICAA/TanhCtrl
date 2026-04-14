#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "tanh_ctrl/common.hpp"
#include "tanh_ctrl/tanh_controller.hpp"

namespace tanh_ctrl
{

namespace
{

void expectVecNear(
  const Eigen::VectorXd & actual, const Eigen::VectorXd & expected, double tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (Eigen::Index index = 0; index < actual.size(); ++index) {
    EXPECT_NEAR(actual(index), expected(index), tolerance);
  }
}

}  // namespace

TEST(ControlTiming, computesDtFromSampleTimestamp)
{
  uint64_t last_sample_us = 0;

  EXPECT_DOUBLE_EQ(computeLoopDtFromSample(1000, &last_sample_us), 1e-4);
  EXPECT_EQ(last_sample_us, 1000u);

  EXPECT_NEAR(computeLoopDtFromSample(21000, &last_sample_us), 0.02, 1e-12);
  EXPECT_EQ(last_sample_us, 21000u);

  EXPECT_DOUBLE_EQ(computeLoopDtFromSample(21000, &last_sample_us), 1e-4);
  EXPECT_EQ(last_sample_us, 21000u);

  EXPECT_DOUBLE_EQ(computeLoopDtFromSample(400000, &last_sample_us), 0.1);
  EXPECT_EQ(last_sample_us, 400000u);

  EXPECT_EQ(selectMessageTimestampUs(50, 100, 150), 50u);
  EXPECT_EQ(selectMessageTimestampUs(0, 100, 150), 100u);
  EXPECT_EQ(selectMessageTimestampUs(0, 0, 150), 150u);
}

TEST(ControlTiming, estimatesLinearAccelerationFromVelocitySamples)
{
  const Eigen::Vector3d current_velocity_ned(1.5, -0.5, 0.25);
  const Eigen::Vector3d previous_velocity_ned(0.9, -0.8, -0.15);

  const Eigen::Vector3d accel_ned =
    estimateLinearAccelerationNed(current_velocity_ned, previous_velocity_ned, 0.2);
  expectVecNear(accel_ned, Eigen::Vector3d(3.0, 1.5, 2.0), 1e-12);

  expectVecNear(
    estimateLinearAccelerationNed(current_velocity_ned, previous_velocity_ned, 0.0),
    Eigen::Vector3d::Zero(), 1e-12);
}

TEST(TanhControllerSplit, splitLoopsMatchMonolithicCompute)
{
  TanhController monolithic_controller;
  TanhController split_controller;

  VehicleState state{};
  state.position_ned = Eigen::Vector3d(1.2, -0.5, -2.1);
  state.velocity_ned = Eigen::Vector3d(0.3, -0.2, 0.1);
  state.linear_acceleration_ned = Eigen::Vector3d(0.2, -0.1, 9.7);
  state.q_body_to_ned =
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX());
  state.angular_velocity_body = Eigen::Vector3d(0.4, -0.3, 0.2);

  TrajectoryRef ref{};
  ref.position_ned = Eigen::Vector3d(0.8, -0.1, -2.5);
  ref.velocity_ned = Eigen::Vector3d(0.0, 0.1, -0.2);
  ref.acceleration_ned = Eigen::Vector3d(0.05, 0.02, 0.1);
  ref.angular_velocity_body = Eigen::Vector3d(0.03, -0.04, 0.05);
  ref.torque_body = Eigen::Vector3d(0.02, -0.01, 0.03);
  ref.yaw = 0.35;
  ref.has_angular_velocity_feedforward = true;
  ref.has_torque_feedforward = true;
  ref.valid = true;

  ControlOutput monolithic_output{};
  ASSERT_TRUE(monolithic_controller.compute(state, ref, 0.01, &monolithic_output));

  AttitudeReference attitude_reference{};
  ASSERT_TRUE(split_controller.computePositionLoop(state, ref, 0.01, &attitude_reference));
  ASSERT_TRUE(attitude_reference.valid);

  ControlOutput split_output{};
  ASSERT_TRUE(split_controller.computeAttitudeLoop(state, attitude_reference, 0.01, &split_output));

  EXPECT_NEAR(split_output.thrust_total, monolithic_output.thrust_total, 1e-9);
  expectVecNear(split_output.torque_body, monolithic_output.torque_body, 1e-9);
  expectVecNear(split_output.motor_forces, monolithic_output.motor_forces, 1e-9);
  expectVecNear(split_output.motor_controls, monolithic_output.motor_controls, 1e-9);
}

}  // namespace tanh_ctrl
