#include <gtest/gtest.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#define private public
#include "tanh_ctrl/tanh_node.hpp"
#undef private

namespace tanh_ctrl {

class TanhNodeMissionStateTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static std::shared_ptr<TanhNode> makeNode() {
    auto node = std::make_shared<TanhNode>();
    node->has_position_state_ = true;
    node->state_.position_ned = Eigen::Vector3d(1.0, -0.5, -1.0);
    node->current_yaw_ = 0.25;
    return node;
  }
};

TEST_F(TanhNodeMissionStateTest, transitionsFromOffboardToArmingThenTakeoff) {
  auto node = makeNode();

  node->is_offboard_ = true;
  node->updateMissionStateMachine(1'000'000);
  EXPECT_EQ(node->mission_state_, WAIT_FOR_ARMING);

  node->is_armed_ = true;
  node->updateMissionStateMachine(1'100'000);
  EXPECT_EQ(node->mission_state_, TAKEOFF);
  ASSERT_TRUE(node->has_hold_ref_);
  EXPECT_DOUBLE_EQ(node->hold_ref_.position_ned.z(), node->mission_takeoff_target_z_);
}

TEST_F(TanhNodeMissionStateTest, completesTakeoffAndSendsStartTrackingOnce) {
  auto node = makeNode();

  node->mission_state_ = TAKEOFF;
  node->state_.position_ned.z() = node->mission_takeoff_target_z_;
  node->mission_takeoff_hold_time_s_ = 0.5;

  node->updateMissionStateMachine(1'000'000);
  EXPECT_EQ(node->mission_state_, TAKEOFF);
  EXPECT_TRUE(node->takeoff_reached_);
  EXPECT_FALSE(node->start_tracking_sent_);

  node->updateMissionStateMachine(1'600'000);
  EXPECT_EQ(node->mission_state_, HOLD);
  EXPECT_TRUE(node->start_tracking_sent_);
}

TEST_F(TanhNodeMissionStateTest, transitionsBetweenHoldAndTrackingUsingReferenceFreshness) {
  auto node = makeNode();

  node->mission_state_ = HOLD;
  node->external_ref_.valid = true;
  node->last_reference_receive_us_ = 1'000'000;
  node->mission_reference_timeout_s_ = 0.3;

  node->updateMissionStateMachine(1'100'000);
  EXPECT_EQ(node->mission_state_, TRACKING);

  node->updateMissionStateMachine(1'400'001);
  EXPECT_EQ(node->mission_state_, HOLD);
}

TEST_F(TanhNodeMissionStateTest, preconditionsResetMissionProgressWhenOffboardOrArmIsLost) {
  auto node = makeNode();

  node->mission_state_ = TRACKING;
  node->start_tracking_sent_ = true;
  node->takeoff_reached_ = true;

  node->handleMissionPreconditions();
  EXPECT_EQ(node->mission_state_, WAIT_FOR_OFFBOARD);
  EXPECT_FALSE(node->start_tracking_sent_);
  EXPECT_FALSE(node->takeoff_reached_);

  node->mission_state_ = TAKEOFF;
  node->is_offboard_ = true;
  node->start_tracking_sent_ = true;
  node->takeoff_reached_ = true;

  node->handleMissionPreconditions();
  EXPECT_EQ(node->mission_state_, WAIT_FOR_ARMING);
  EXPECT_FALSE(node->start_tracking_sent_);
  EXPECT_FALSE(node->takeoff_reached_);
}

}  // namespace tanh_ctrl
