#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "tanh_ctrl/tanh_ctrl.hpp"

namespace {

tanh_ctrl::tanh_ctrl makeController()
{
  tanh_ctrl::tanh_ctrl controller;
  controller.setMass(2.0);
  controller.setGravity(9.81);
  controller.setInertia(0.02 * Eigen::Matrix3d::Identity());
  controller.setAllocationParams(tanh_ctrl::AllocationParams{0.25, M_PI_4, 0.016});
  controller.setMotorForceMax(10.0);
  controller.setMaxTilt(0.7);

  tanh_ctrl::PositionGains pos{};
  pos.M_P = Eigen::Vector3d::Constant(1.0);
  pos.K_P = Eigen::Vector3d::Constant(1.0);
  pos.M_V = Eigen::Vector3d::Constant(2.0);
  pos.K_V = Eigen::Vector3d::Constant(1.0);
  pos.K_Acceleration = Eigen::Vector3d::Zero();
  pos.P_V = Eigen::Vector3d::Zero();
  pos.L_V = Eigen::Vector3d::Ones();
  controller.setPositionGains(pos);

  tanh_ctrl::AttitudeGains att{};
  att.M_Angle = Eigen::Vector3d::Constant(2.0);
  att.K_Angle = Eigen::Vector3d::Constant(3.0);
  att.M_AngularVelocity = Eigen::Vector3d::Constant(1.5);
  att.K_AngularVelocity = Eigen::Vector3d::Constant(2.0);
  att.K_AngularAcceleration = Eigen::Vector3d::Zero();
  att.P_AngularVelocity = Eigen::Vector3d::Zero();
  att.L_AngularVelocity = Eigen::Vector3d::Ones();
  controller.setAttitudeGains(att);
  return controller;
}

tanh_ctrl::VehicleState makeHoverState()
{
  tanh_ctrl::VehicleState state;
  state.position_ned.setZero();
  state.velocity_ned.setZero();
  state.linear_acceleration_ned.setZero();
  state.q_body_to_ned = Eigen::Quaterniond::Identity();
  state.angular_velocity_body.setZero();
  return state;
}

tanh_ctrl::TrajectoryRef makeHoverReference()
{
  tanh_ctrl::TrajectoryRef ref;
  ref.position_ned.setZero();
  ref.velocity_ned.setZero();
  ref.acceleration_ned.setZero();
  ref.yaw = 0.0;
  ref.valid = true;
  return ref;
}

TEST(TanhCtrlCoreFeedforwardTest, UsesFeedforwardAngularVelocity)
{
  auto state = makeHoverState();
  auto ref = makeHoverReference();

  state.angular_velocity_body = Eigen::Vector3d(0.4, -0.3, 0.2);
  ref.angular_velocity_body = state.angular_velocity_body;
  ref.has_angular_velocity_feedforward = true;

  auto controller = makeController();
  tanh_ctrl::ControlOutput with_rate_ff;
  ASSERT_TRUE(controller.compute(state, ref, 0.01, &with_rate_ff));
  EXPECT_NEAR(with_rate_ff.torque_body.norm(), 0.0, 1e-6);

  ref.has_angular_velocity_feedforward = false;
  controller = makeController();
  tanh_ctrl::ControlOutput without_rate_ff;
  ASSERT_TRUE(controller.compute(state, ref, 0.01, &without_rate_ff));
  EXPECT_GT(without_rate_ff.torque_body.norm(), 1e-3);
}

TEST(TanhCtrlCoreFeedforwardTest, AddsFeedforwardBodyTorque)
{
  auto controller = makeController();
  auto state = makeHoverState();
  auto ref = makeHoverReference();

  ref.torque_body = Eigen::Vector3d(0.12, -0.08, 0.03);
  ref.has_torque_feedforward = true;

  tanh_ctrl::ControlOutput out;
  ASSERT_TRUE(controller.compute(state, ref, 0.01, &out));
  EXPECT_NEAR(out.torque_body.x(), ref.torque_body.x(), 1e-6);
  EXPECT_NEAR(out.torque_body.y(), ref.torque_body.y(), 1e-6);
  EXPECT_NEAR(out.torque_body.z(), ref.torque_body.z(), 1e-6);
}

}  // namespace
