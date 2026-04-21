#include "tanh_ctrl/common.hpp"

#include <algorithm>
#include <cmath>

namespace tanh_ctrl {

namespace {

constexpr double kMinPositiveZ = 1e-3;
constexpr double kSmallNorm = 1e-9;
constexpr double kMinCollectiveThrust = 1e-6;
constexpr double kMinLoopDt = 1e-4;
constexpr double kMaxLoopDt = 0.1;

}  // namespace

Eigen::Vector3d planarAxisVec(double planar, double axial) {
  return Eigen::Vector3d(planar, planar, axial);
}

uint64_t selectMessageTimestampUs(uint64_t timestamp_sample_us, uint64_t timestamp_us, uint64_t fallback_us) {
  if (timestamp_sample_us != 0) {
    return timestamp_sample_us;
  }
  if (timestamp_us != 0) {
    return timestamp_us;
  }
  return fallback_us;
}

double computeLoopDtFromSample(uint64_t sample_timestamp_us, uint64_t* last_sample_timestamp_us) {
  if (!last_sample_timestamp_us || sample_timestamp_us == 0) {
    return kMinLoopDt;
  }

  if (*last_sample_timestamp_us == 0 || sample_timestamp_us <= *last_sample_timestamp_us) {
    *last_sample_timestamp_us = sample_timestamp_us;
    return kMinLoopDt;
  }

  const double dt = static_cast<double>(sample_timestamp_us - *last_sample_timestamp_us) * 1e-6;
  *last_sample_timestamp_us = sample_timestamp_us;
  return std::clamp(dt, kMinLoopDt, kMaxLoopDt);
}

double throttleFromRelativeThrust(double relative_thrust, double thrust_model_factor) {
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

void applyTiltLimit(Eigen::Vector3d* thrust_vector_over_mass_ned, double max_tilt_rad) {
  if (!thrust_vector_over_mass_ned || max_tilt_rad <= 0.0) {
    return;
  }

  thrust_vector_over_mass_ned->z() = std::max(thrust_vector_over_mass_ned->z(), kMinPositiveZ);
  const double horizontal_norm = std::hypot(thrust_vector_over_mass_ned->x(), thrust_vector_over_mass_ned->y());
  const double max_horizontal = thrust_vector_over_mass_ned->z() * std::tan(max_tilt_rad);

  if (horizontal_norm <= max_horizontal || horizontal_norm <= kSmallNorm) {
    return;
  }

  const double scale = max_horizontal / horizontal_norm;
  thrust_vector_over_mass_ned->x() *= scale;
  thrust_vector_over_mass_ned->y() *= scale;
}

Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d& thrust_direction_ned, double yaw) {
  Eigen::Vector3d z_body_ned = thrust_direction_ned;
  if (z_body_ned.norm() <= kMinCollectiveThrust) {
    z_body_ned = Eigen::Vector3d::UnitZ();
  } else {
    z_body_ned.normalize();
  }

  Eigen::Vector3d x_course_ned(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d y_body_ned = z_body_ned.cross(x_course_ned);
  if (y_body_ned.norm() <= kMinCollectiveThrust) {
    x_course_ned = Eigen::Vector3d::UnitX();
    y_body_ned = z_body_ned.cross(x_course_ned);
  }
  y_body_ned.normalize();

  Eigen::Vector3d x_body_ned = y_body_ned.cross(z_body_ned);
  x_body_ned.normalize();

  Eigen::Matrix3d desired_rotation;
  desired_rotation.col(0) = x_body_ned;
  desired_rotation.col(1) = y_body_ned;
  desired_rotation.col(2) = z_body_ned;

  Eigen::Quaterniond desired_attitude(desired_rotation);
  desired_attitude.normalize();
  return desired_attitude;
}

Eigen::Vector3d rotateReferenceBodyVectorToCurrentBody(const Eigen::Quaterniond& current_body_to_ned, const Eigen::Quaterniond& reference_body_to_ned, const Eigen::Vector3d& reference_body_vector) {
  const Eigen::Quaterniond reference_body_to_current_body = current_body_to_ned.conjugate() * reference_body_to_ned;
  return reference_body_to_current_body * reference_body_vector;
}

AttitudeReference computeAttitudeReference(const Eigen::Vector3d& desired_thrust_vector_ned, const TrajectoryRef& ref) {
  AttitudeReference attitude_reference;

  const Eigen::Vector3d thrust_vector_ned = desired_thrust_vector_ned;
  const double collective_thrust = thrust_vector_ned.norm();
  if (collective_thrust <= kMinCollectiveThrust) {
    attitude_reference.collective_thrust = 0.0;
    attitude_reference.valid = false;
    return attitude_reference;
  }

  attitude_reference.collective_thrust = collective_thrust;
  attitude_reference.thrust_direction_ned = thrust_vector_ned / collective_thrust;
  attitude_reference.attitude_body_to_ned = computeDesiredAttitude(attitude_reference.thrust_direction_ned, ref.yaw);
  attitude_reference.angular_velocity_body = ref.angular_velocity_body;
  attitude_reference.has_angular_velocity_feedforward = ref.has_angular_velocity_feedforward;
  attitude_reference.angular_acceleration_body = ref.angular_acceleration_body;
  attitude_reference.has_angular_acceleration_feedforward = ref.has_angular_acceleration_feedforward;
  attitude_reference.torque_body = ref.torque_body;
  attitude_reference.has_torque_feedforward = ref.has_torque_feedforward;
  attitude_reference.valid = true;

  return attitude_reference;
}

Eigen::Vector4d allocateMotorForces(const AllocationParams& params, double thrust_total, const Eigen::Vector3d& torque_body) {
  const double sin_beta = std::sin(params.beta);
  const double cos_beta = std::cos(params.beta);

  Eigen::Matrix4d mixer;
  mixer << 1.0, 1.0, 1.0, 1.0, params.l * sin_beta, -params.l * sin_beta, -params.l * sin_beta, params.l * sin_beta, params.l * cos_beta, params.l * cos_beta, -params.l * cos_beta, -params.l * cos_beta, -params.cq_ct, params.cq_ct, -params.cq_ct, params.cq_ct;

  Eigen::Vector4d wrench;
  wrench << thrust_total, torque_body.x(), torque_body.y(), torque_body.z();

  return mixer.fullPivLu().solve(wrench);
}

Eigen::Vector4d forcesToMotorControls(const Eigen::Vector4d& forces, double motor_force_max, double thrust_model_factor) {
  const double safe_motor_force_max = std::max(kMinCollectiveThrust, motor_force_max);
  Eigen::Vector4d controls = Eigen::Vector4d::Zero();
  for (int index = 0; index < 4; ++index) {
    const double relative_thrust = std::clamp(forces(index) / safe_motor_force_max, 0.0, 1.0);
    controls(index) = throttleFromRelativeThrust(relative_thrust, thrust_model_factor);
  }
  return controls;
}

}  // namespace tanh_ctrl
