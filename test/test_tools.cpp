#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/tools.hpp"

namespace tanh_ctrl {

namespace {

void expectVecNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tolerance) {
  for (Eigen::Index index = 0; index < actual.size(); ++index) {
    EXPECT_NEAR(actual(index), expected(index), tolerance);
  }
}

}  // namespace

TEST(Tools, computesElapsedTimeAndRequestDeadline) {
  EXPECT_DOUBLE_EQ(elapsedSeconds(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(elapsedSeconds(10, 10), 0.0);
  EXPECT_NEAR(elapsedSeconds(2'500'000, 1'000'000), 1.5, 1e-12);

  EXPECT_TRUE(requestDue(2'500'000, 0, 0.1));
  EXPECT_FALSE(requestDue(2'500'000, 2'450'000, 0.1));
  EXPECT_TRUE(requestDue(2'500'000, 2'300'000, 0.1));
}

TEST(Tools, extractsYawFromQuaternion) {
  const Eigen::Quaterniond q = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());

  EXPECT_NEAR(quaternionToYaw(q), 0.4, 1e-12);
}

TEST(Tools, validatesAndConvertsArrayVectors) {
  const std::array<float, 3> finite_values{1.5f, -2.0f, 0.25f};
  const std::array<float, 3> partial_nan_values{
      1.5f, std::numeric_limits<float>::quiet_NaN(), 0.25f};
  const std::array<float, 4> finite_quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  const std::array<float, 4> invalid_quaternion{
      1.0f, 0.0f, std::numeric_limits<float>::infinity(), 0.0f};

  EXPECT_TRUE(isFiniteVec3(finite_values));
  EXPECT_FALSE(isFiniteVec3(partial_nan_values));
  EXPECT_TRUE(isFiniteQuat(finite_quaternion));
  EXPECT_FALSE(isFiniteQuat(invalid_quaternion));

  expectVecNear(eigenFromArray3OrZero(partial_nan_values), Eigen::Vector3d(1.5, 0.0, 0.25), 1e-12);
}

TEST(Tools, validatesAndConvertsMessageStyleVectors) {
  struct MsgVec3 {
    double x;
    double y;
    double z;
  };

  const MsgVec3 finite_values{1.0, -2.5, 0.75};
  const MsgVec3 partial_nan_values{1.0, std::numeric_limits<double>::quiet_NaN(), 0.75};

  EXPECT_TRUE(isFiniteXyz(finite_values));
  EXPECT_FALSE(isFiniteXyz(partial_nan_values));

  expectVecNear(eigenFromXyzOrZero(partial_nan_values), Eigen::Vector3d(1.0, 0.0, 0.75), 1e-12);
}

TEST(Tools, declaresAndLoadsAxisPairParameters) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("tools_test_node");
  declareAxisPair(*node, "gains.planar", 2.5, "gains.axial", 1.5);
  node->set_parameter(rclcpp::Parameter("gains.planar", 4.0));
  node->set_parameter(rclcpp::Parameter("gains.axial", 2.0));

  expectVecNear(loadAxisPairParam(*node, "gains.planar", "gains.axial"), Eigen::Vector3d(4.0, 4.0, 2.0), 1e-12);

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

}  // namespace tanh_ctrl
