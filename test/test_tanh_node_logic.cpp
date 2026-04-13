#include <gtest/gtest.h>

#include <limits>

#include "tanh_ctrl/tanh_node.hpp"

namespace
{

using tanh_ctrl::MissionState;

TEST(TanhNodeLogic, MissionStateNamesRemainStable)
{
  EXPECT_STREQ(tanh_ctrl::toString(MissionState::WAIT_FOR_OFFBOARD), "WAIT_FOR_OFFBOARD");
  EXPECT_STREQ(tanh_ctrl::toString(MissionState::WAIT_FOR_ARMING), "WAIT_FOR_ARMING");
  EXPECT_STREQ(tanh_ctrl::toString(MissionState::TAKEOFF), "TAKEOFF");
  EXPECT_STREQ(tanh_ctrl::toString(MissionState::HOLD), "HOLD");
  EXPECT_STREQ(tanh_ctrl::toString(MissionState::TRACKING), "TRACKING");
}

TEST(TanhNodeLogic, ReferenceMessageRequiresFinitePosition)
{
  tanh_ctrl::msg::FlatTrajectoryReference msg{};
  msg.position_ned.x = 1.0;
  msg.position_ned.y = std::numeric_limits<double>::quiet_NaN();
  msg.position_ned.z = -2.0;

  EXPECT_FALSE(tanh_ctrl::referenceMessageHasValidPosition(msg));
}

TEST(TanhNodeLogic, ReferenceMessageConversionKeepsCurrentFeedforwardSemantics)
{
  tanh_ctrl::msg::FlatTrajectoryReference msg{};
  msg.position_ned.x = 1.0;
  msg.position_ned.y = 2.0;
  msg.position_ned.z = -3.0;
  msg.velocity_ned.x = std::numeric_limits<double>::infinity();
  msg.velocity_ned.y = 4.0;
  msg.velocity_ned.z = -5.0;
  msg.acceleration_ned.x = 0.1;
  msg.acceleration_ned.y = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration_ned.z = 0.3;
  msg.body_rates_frd.x = 0.4;
  msg.body_rates_frd.y = std::numeric_limits<double>::quiet_NaN();
  msg.body_rates_frd.z = 0.6;
  msg.body_torque_frd.x = 0.7;
  msg.body_torque_frd.y = -0.2;
  msg.body_torque_frd.z = 0.3;
  msg.yaw = std::numeric_limits<float>::quiet_NaN();

  const tanh_ctrl::TrajectoryRef ref = tanh_ctrl::trajectoryReferenceFromMsg(msg, 42U);

  EXPECT_TRUE(ref.valid);
  EXPECT_DOUBLE_EQ(ref.position_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.y(), 2.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.z(), -3.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.x(), 0.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.y(), 4.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.z(), -5.0);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.x(), 0.1);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.y(), 0.0);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.z(), 0.3);
  EXPECT_DOUBLE_EQ(ref.angular_velocity_body.x(), 0.4);
  EXPECT_DOUBLE_EQ(ref.angular_velocity_body.y(), 0.0);
  EXPECT_DOUBLE_EQ(ref.angular_velocity_body.z(), 0.6);
  EXPECT_FALSE(ref.has_angular_velocity_feedforward);
  EXPECT_DOUBLE_EQ(ref.torque_body.x(), 0.7);
  EXPECT_DOUBLE_EQ(ref.torque_body.y(), -0.2);
  EXPECT_DOUBLE_EQ(ref.torque_body.z(), 0.3);
  EXPECT_TRUE(ref.has_torque_feedforward);
  EXPECT_DOUBLE_EQ(ref.yaw, 0.0);
}

TEST(TanhNodeLogic, HoldReferenceCapturesXYTargetAltitudeAndYaw)
{
  tanh_ctrl::VehicleState state{};
  state.position_ned = Eigen::Vector3d(3.0, -4.0, -1.5);

  const tanh_ctrl::TrajectoryRef hold_ref = tanh_ctrl::makeHoldReference(state, -2.5, 1.2);

  EXPECT_TRUE(hold_ref.valid);
  EXPECT_DOUBLE_EQ(hold_ref.position_ned.x(), 3.0);
  EXPECT_DOUBLE_EQ(hold_ref.position_ned.y(), -4.0);
  EXPECT_DOUBLE_EQ(hold_ref.position_ned.z(), -2.5);
  EXPECT_DOUBLE_EQ(hold_ref.yaw, 1.2);
  EXPECT_DOUBLE_EQ(hold_ref.velocity_ned.norm(), 0.0);
  EXPECT_DOUBLE_EQ(hold_ref.acceleration_ned.norm(), 0.0);
  EXPECT_FALSE(hold_ref.has_angular_velocity_feedforward);
  EXPECT_FALSE(hold_ref.has_torque_feedforward);
}

TEST(TanhNodeLogic, ReferenceFreshnessHonorsTimeoutRules)
{
  tanh_ctrl::TrajectoryRef ref{};
  ref.valid = true;

  EXPECT_TRUE(tanh_ctrl::hasFreshExternalReference(ref, 1000000U, 900000U, 0.0));
  EXPECT_TRUE(tanh_ctrl::hasFreshExternalReference(ref, 1000000U, 900000U, 0.2));
  EXPECT_FALSE(tanh_ctrl::hasFreshExternalReference(ref, 1000000U, 700000U, 0.2));
  EXPECT_FALSE(tanh_ctrl::hasFreshExternalReference(ref, 1000000U, 0U, 0.2));

  ref.valid = false;
  EXPECT_FALSE(tanh_ctrl::hasFreshExternalReference(ref, 1000000U, 900000U, 0.2));
}

TEST(TanhNodeLogic, MissionStateTransitionMatchesCurrentFlow)
{
  EXPECT_EQ(tanh_ctrl::nextMissionState(MissionState::WAIT_FOR_OFFBOARD, true, false, false, false),
            MissionState::WAIT_FOR_ARMING);
  EXPECT_EQ(tanh_ctrl::nextMissionState(MissionState::WAIT_FOR_ARMING, true, true, false, false),
            MissionState::TAKEOFF);
  EXPECT_EQ(tanh_ctrl::nextMissionState(MissionState::TAKEOFF, true, true, true, false),
            MissionState::HOLD);
  EXPECT_EQ(tanh_ctrl::nextMissionState(MissionState::HOLD, true, true, false, true),
            MissionState::TRACKING);
  EXPECT_EQ(tanh_ctrl::nextMissionState(MissionState::TRACKING, true, true, false, false),
            MissionState::HOLD);
}

TEST(TanhNodeLogic, StartTrackingSignalTriggersOnlyOnFirstTakeoffCompletion)
{
  EXPECT_TRUE(tanh_ctrl::shouldPublishStartTrackingSignal(MissionState::TAKEOFF, true, false));
  EXPECT_FALSE(tanh_ctrl::shouldPublishStartTrackingSignal(MissionState::TAKEOFF, true, true));
  EXPECT_FALSE(tanh_ctrl::shouldPublishStartTrackingSignal(MissionState::HOLD, true, false));
  EXPECT_FALSE(tanh_ctrl::shouldPublishStartTrackingSignal(MissionState::TAKEOFF, false, false));
}

} // namespace
