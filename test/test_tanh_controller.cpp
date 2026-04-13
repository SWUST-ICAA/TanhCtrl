#include "tanh_ctrl/tanh_controller.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

namespace {

constexpr double kTol = 1e-6;

bool expectNear(
  const Eigen::Vector3d & actual, const Eigen::Vector3d & expected, const char * label)
{
  if ((actual - expected).cwiseAbs().maxCoeff() <= kTol) {
    return true;
  }

  std::cerr << label << " mismatch\n"
            << "  actual:   " << actual.transpose() << "\n"
            << "  expected: " << expected.transpose() << "\n";
  return false;
}

bool expectSmall(const Eigen::Vector3d & actual, const char * label)
{
  return expectNear(actual, Eigen::Vector3d::Zero(), label);
}

tanh_ctrl::PositionGains zeroPositionGains()
{
  tanh_ctrl::PositionGains gains;
  gains.M_P.setZero();
  gains.K_P.setZero();
  gains.M_V.setZero();
  gains.K_V.setZero();
  gains.K_Acceleration.setZero();
  gains.P_V.setZero();
  gains.L_V.setOnes();
  return gains;
}

tanh_ctrl::AttitudeGains zeroAttitudeGains()
{
  tanh_ctrl::AttitudeGains gains;
  gains.M_Angle.setZero();
  gains.K_Angle.setZero();
  gains.M_AngularVelocity.setZero();
  gains.K_AngularVelocity.setZero();
  gains.K_AngularAcceleration.setZero();
  gains.P_AngularVelocity.setZero();
  gains.L_AngularVelocity.setOnes();
  return gains;
}

tanh_ctrl::AttitudeGains angularVelocityTrackingGains()
{
  tanh_ctrl::AttitudeGains gains = zeroAttitudeGains();
  gains.M_AngularVelocity.setOnes();
  gains.K_AngularVelocity.setConstant(5.0);
  return gains;
}

tanh_ctrl::VehicleState makeState(
  const Eigen::Quaterniond & q_body_to_ned,
  const Eigen::Vector3d & angular_velocity_body = Eigen::Vector3d::Zero())
{
  tanh_ctrl::VehicleState state;
  state.q_body_to_ned = q_body_to_ned;
  state.angular_velocity_body = angular_velocity_body;
  return state;
}

tanh_ctrl::TrajectoryRef makeHoverReference()
{
  tanh_ctrl::TrajectoryRef ref;
  ref.valid = true;
  ref.yaw = 0.0;
  return ref;
}

tanh_ctrl::TanhController makeController(const tanh_ctrl::AttitudeGains & attitude_gains)
{
  tanh_ctrl::TanhController controller;
  controller.setMass(1.0);
  controller.setGravity(9.81);
  controller.setInertia(Eigen::Matrix3d::Identity());
  controller.setPositionGains(zeroPositionGains());
  controller.setAttitudeGains(attitude_gains);
  controller.setMotorForceMax(100.0);
  controller.setThrustModelFactor(0.0);
  controller.reset();
  return controller;
}

bool torqueFeedforwardUsesCurrentBodyFrame()
{
  tanh_ctrl::TanhController controller = makeController(zeroAttitudeGains());

  tanh_ctrl::TrajectoryRef ref = makeHoverReference();
  ref.torque_body = Eigen::Vector3d::UnitX();
  ref.has_torque_feedforward = true;

  const Eigen::Quaterniond current_body_to_ned(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const tanh_ctrl::VehicleState state = makeState(current_body_to_ned);

  tanh_ctrl::ControlOutput out;
  if (!controller.compute(state, ref, 0.01, &out)) {
    std::cerr << "controller.compute failed in torque feedforward test\n";
    return false;
  }

  return expectNear(out.torque_body, Eigen::Vector3d(0.0, -1.0, 0.0), "torque feedforward");
}

bool angularVelocityFeedforwardUsesCurrentBodyFrame()
{
  tanh_ctrl::TanhController controller = makeController(angularVelocityTrackingGains());

  tanh_ctrl::TrajectoryRef ref = makeHoverReference();
  ref.angular_velocity_body = Eigen::Vector3d(0.2, 0.0, 0.0);
  ref.has_angular_velocity_feedforward = true;

  const Eigen::Quaterniond current_body_to_ned(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const tanh_ctrl::VehicleState state =
    makeState(current_body_to_ned, Eigen::Vector3d(0.0, -0.2, 0.0));

  tanh_ctrl::ControlOutput out;
  if (!controller.compute(state, ref, 0.01, &out)) {
    std::cerr << "controller.compute failed in angular velocity feedforward test\n";
    return false;
  }

  return expectSmall(out.torque_body, "angular velocity feedforward");
}

}  // namespace

int main()
{
  bool ok = true;
  ok = torqueFeedforwardUsesCurrentBodyFrame() && ok;
  ok = angularVelocityFeedforwardUsesCurrentBodyFrame() && ok;
  return ok ? 0 : 1;
}
