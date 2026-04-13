#include "tanh_ctrl/common.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

namespace {

constexpr double kTol = 1e-6;

bool expectNear(double actual, double expected, const char * label)
{
  if (std::abs(actual - expected) <= kTol) {
    return true;
  }

  std::cerr << label << " mismatch\n"
            << "  actual:   " << actual << "\n"
            << "  expected: " << expected << "\n";
  return false;
}

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

bool expectNear(
  const Eigen::Vector4d & actual, const Eigen::Vector4d & expected, const char * label)
{
  if ((actual - expected).cwiseAbs().maxCoeff() <= kTol) {
    return true;
  }

  std::cerr << label << " mismatch\n"
            << "  actual:   " << actual.transpose() << "\n"
            << "  expected: " << expected.transpose() << "\n";
  return false;
}

bool throttleIsIdentityWhenFactorIsZero()
{
  return expectNear(
    tanh_ctrl::throttleFromRelativeThrust(0.35, 0.0), 0.35,
    "throttleFromRelativeThrust identity");
}

bool rotatesReferenceBodyVectorIntoCurrentBodyFrame()
{
  const Eigen::Quaterniond current_body_to_ned(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond reference_body_to_ned = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d reference_body_vector = Eigen::Vector3d::UnitX();

  const Eigen::Vector3d rotated = tanh_ctrl::rotateReferenceBodyVectorToCurrentBody(
    current_body_to_ned, reference_body_to_ned, reference_body_vector);

  return expectNear(
    rotated, Eigen::Vector3d(0.0, -1.0, 0.0),
    "rotateReferenceBodyVectorToCurrentBody");
}

bool reconstructsAttitudeReferenceFromThrustAndYaw()
{
  tanh_ctrl::TrajectoryRef ref;
  ref.valid = true;
  ref.yaw = 0.0;

  const tanh_ctrl::AttitudeReference attitude_reference =
    tanh_ctrl::computeAttitudeReference(Eigen::Vector3d(0.0, 0.0, 9.81), ref);

  const bool thrust_ok = expectNear(
    attitude_reference.thrust_direction_ned, Eigen::Vector3d::UnitZ(),
    "attitude thrust direction");
  const bool thrust_norm_ok = expectNear(
    attitude_reference.collective_thrust, 9.81, "attitude collective thrust");
  const bool valid_ok = attitude_reference.valid;
  if (!valid_ok) {
    std::cerr << "attitude reference should be valid\n";
  }
  return thrust_ok && thrust_norm_ok && valid_ok;
}

bool allocatesBalancedMotorForcesForPureCollectiveThrust()
{
  tanh_ctrl::AllocationParams params;
  params.l = 0.2;
  params.beta = M_PI_4;
  params.cq_ct = 0.01;

  const Eigen::Vector4d forces = tanh_ctrl::allocateMotorForces(
    params, 4.0, Eigen::Vector3d::Zero());

  return expectNear(
    forces, Eigen::Vector4d::Constant(1.0), "allocateMotorForces collective");
}

bool mapsMotorForcesToControls()
{
  const Eigen::Vector4d controls = tanh_ctrl::forcesToMotorControls(
    Eigen::Vector4d(0.0, 5.0, 10.0, 20.0), 10.0, 0.0);

  return expectNear(
    controls, Eigen::Vector4d(0.0, 0.5, 1.0, 1.0), "forcesToMotorControls");
}

}  // namespace

int main()
{
  bool ok = true;
  ok = throttleIsIdentityWhenFactorIsZero() && ok;
  ok = rotatesReferenceBodyVectorIntoCurrentBodyFrame() && ok;
  ok = reconstructsAttitudeReferenceFromThrustAndYaw() && ok;
  ok = allocatesBalancedMotorForcesForPureCollectiveThrust() && ok;
  ok = mapsMotorForcesToControls() && ok;
  return ok ? 0 : 1;
}
