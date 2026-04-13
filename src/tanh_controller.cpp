#include "tanh_ctrl/tanh_controller.hpp"

#include <algorithm>
#include <cmath>

namespace tanh_ctrl
{

namespace
{

constexpr double kMinMass = 1e-3;
constexpr double kMinTiltRad = 1e-3;
constexpr double kMinDt = 1e-4;
constexpr double kMaxDt = 0.1;

double clampPositive(double value, double min_value)
{
  return std::max(min_value, value);
}

}  // namespace

TanhController::TanhController() = default;

void TanhController::setMass(double mass)
{
  mass_ = clampPositive(mass, kMinMass);
}

void TanhController::setGravity(double gravity)
{
  gravity_ = std::max(0.0, gravity);
}

void TanhController::setPositionGains(const PositionGains & gains)
{
  pos_gains_ = gains;
}

void TanhController::setAttitudeGains(const AttitudeGains & gains)
{
  att_gains_ = gains;
}

void TanhController::setInertia(const Eigen::Matrix3d & inertia)
{
  inertia_ = inertia;
}

void TanhController::setAllocationParams(const AllocationParams & alloc)
{
  alloc_ = alloc;
}

void TanhController::setMotorForceMax(double max_force)
{
  motor_force_max_ = clampPositive(max_force, kMinMass);
}

void TanhController::setThrustModelFactor(double thrust_model_factor)
{
  thrust_model_factor_ = std::clamp(thrust_model_factor, 0.0, 1.0);
}

void TanhController::setMaxTilt(double max_tilt_rad)
{
  if (!std::isfinite(max_tilt_rad) || max_tilt_rad <= 0.0) {
    max_tilt_rad_ = 0.0;
    return;
  }

  max_tilt_rad_ = std::clamp(max_tilt_rad, kMinTiltRad, M_PI_2 - kMinTiltRad);
}

void TanhController::setLinearAccelerationLowPassHz(const Eigen::Vector3d & cutoff_hz)
{
  linear_accel_lpf_cutoff_hz_ = sanitizeCutoff(cutoff_hz);
  linear_accel_lpf_initialized_ = {{false, false, false}};
}

void TanhController::setAngularAccelerationLowPassHz(double cutoff_hz)
{
  angular_accel_lpf_cutoff_hz_ = sanitizeCutoff(cutoff_hz);
  angular_accel_lpf_initialized_ = false;
}

void TanhController::setVelocityDisturbanceLowPassHz(double cutoff_hz)
{
  velocity_disturbance_lpf_cutoff_hz_ = sanitizeCutoff(cutoff_hz);
  velocity_disturbance_lpf_initialized_ = false;
}

void TanhController::setAngularVelocityDisturbanceLowPassHz(double cutoff_hz)
{
  angular_velocity_disturbance_lpf_cutoff_hz_ = sanitizeCutoff(cutoff_hz);
  angular_velocity_disturbance_lpf_initialized_ = false;
}

void TanhController::reset()
{
  velocity_error_hat_ned_.setZero();
  angular_velocity_error_hat_body_.setZero();
  last_angular_velocity_body_.setZero();
  has_last_angular_velocity_ = false;
  first_run_ = true;

  linear_accel_lpf_state_ned_.setZero();
  angular_accel_lpf_state_body_.setZero();
  linear_accel_lpf_initialized_ = {{false, false, false}};
  angular_accel_lpf_initialized_ = false;

  velocity_disturbance_lpf_state_ned_.setZero();
  angular_velocity_disturbance_lpf_state_body_.setZero();
  velocity_disturbance_lpf_initialized_ = false;
  angular_velocity_disturbance_lpf_initialized_ = false;
}

double TanhController::sanitizeCutoff(double cutoff_hz)
{
  return (std::isfinite(cutoff_hz) && cutoff_hz > 0.0) ? cutoff_hz : 0.0;
}

Eigen::Vector3d TanhController::sanitizeCutoff(const Eigen::Vector3d & cutoff_hz)
{
  return Eigen::Vector3d(
    sanitizeCutoff(cutoff_hz.x()), sanitizeCutoff(cutoff_hz.y()), sanitizeCutoff(cutoff_hz.z()));
}

Eigen::Vector3d TanhController::tanhVec(const Eigen::Vector3d & x)
{
  return x.array().tanh().matrix();
}

double TanhController::lowPassScalar(
  double x, double cutoff_hz, double dt, double * state, bool * initialized)
{
  if (!state || !initialized) {
    return x;
  }

  if (!(cutoff_hz > 0.0) || !std::isfinite(cutoff_hz) || !(dt > 0.0) || !std::isfinite(dt)) {
    return x;
  }

  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  const double alpha = dt / (tau + dt);

  if (!(*initialized)) {
    *state = x;
    *initialized = true;
    return x;
  }

  *state += alpha * (x - *state);
  return *state;
}

Eigen::Vector3d TanhController::lowPassVec3(
  const Eigen::Vector3d & x, double cutoff_hz, double dt,
  Eigen::Vector3d * state, bool * initialized)
{
  if (!state || !initialized) {
    return x;
  }

  Eigen::Vector3d result;
  result.x() = lowPassScalar(x.x(), cutoff_hz, dt, &state->x(), initialized);
  result.y() = lowPassScalar(x.y(), cutoff_hz, dt, &state->y(), initialized);
  result.z() = lowPassScalar(x.z(), cutoff_hz, dt, &state->z(), initialized);
  return result;
}

Eigen::Vector3d TanhController::lowPassVec3(
  const Eigen::Vector3d & x, const Eigen::Vector3d & cutoff_hz, double dt,
  Eigen::Vector3d * state, std::array<bool, 3> * initialized)
{
  if (!state || !initialized) {
    return x;
  }

  Eigen::Vector3d result;
  result.x() = lowPassScalar(x.x(), cutoff_hz.x(), dt, &state->x(), &(*initialized)[0]);
  result.y() = lowPassScalar(x.y(), cutoff_hz.y(), dt, &state->y(), &(*initialized)[1]);
  result.z() = lowPassScalar(x.z(), cutoff_hz.z(), dt, &state->z(), &(*initialized)[2]);
  return result;
}

bool TanhController::compute(
  const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out)
{
  if (!out || !ref.valid) {
    return false;
  }

  if (first_run_) {
    velocity_error_hat_ned_.setZero();
    angular_velocity_error_hat_body_.setZero();
    last_angular_velocity_body_ = state.angular_velocity_body;
    has_last_angular_velocity_ = true;
    first_run_ = false;
  }

  dt = std::clamp(dt, kMinDt, kMaxDt);

  Eigen::Vector3d thrust_vec_ned = Eigen::Vector3d::Zero();
  double thrust_norm = 0.0;
  computePosition(state, ref, dt, &thrust_vec_ned, &thrust_norm);

  const AttitudeReference attitude_reference = computeAttitudeReference(thrust_vec_ned, ref);

  Eigen::Vector3d torque_body = Eigen::Vector3d::Zero();
  computeAttitude(state, attitude_reference, dt, &torque_body);

  out->thrust_total = thrust_norm;
  out->torque_body = torque_body;
  out->motor_forces = allocateMotorForces(alloc_, thrust_norm, torque_body);
  out->motor_controls =
    forcesToMotorControls(out->motor_forces, motor_force_max_, thrust_model_factor_);
  return true;
}

void TanhController::computePosition(
  const VehicleState & state, const TrajectoryRef & ref, double dt,
  Eigen::Vector3d * thrust_vec_ned, double * thrust_norm)
{
  const Eigen::Vector3d position_error_ned = state.position_ned - ref.position_ned;

  Eigen::Vector3d linear_acceleration_ned = state.linear_acceleration_ned;
  if (!linear_acceleration_ned.allFinite()) {
    linear_acceleration_ned.setZero();
  }
  linear_acceleration_ned =
    lowPassVec3(linear_acceleration_ned, linear_accel_lpf_cutoff_hz_, dt,
      &linear_accel_lpf_state_ned_, &linear_accel_lpf_initialized_);

  const Eigen::Vector3d tanh_position_error =
    tanhVec(pos_gains_.K_P.cwiseProduct(position_error_ned));
  const Eigen::Vector3d velocity_error_ned =
    (state.velocity_ned - ref.velocity_ned) + pos_gains_.M_P.cwiseProduct(tanh_position_error);

  const Eigen::Vector3d velocity_estimation_error_ned =
    velocity_error_ned - velocity_error_hat_ned_;
  const Eigen::Vector3d velocity_disturbance_raw = pos_gains_.P_V.cwiseProduct(
    tanhVec(pos_gains_.L_V.cwiseProduct(velocity_estimation_error_ned)));
  const Eigen::Vector3d velocity_disturbance_filtered =
    lowPassVec3(velocity_disturbance_raw, velocity_disturbance_lpf_cutoff_hz_, dt,
      &velocity_disturbance_lpf_state_ned_, &velocity_disturbance_lpf_initialized_);

  const Eigen::Vector3d gravity_ned(0.0, 0.0, gravity_);
  const Eigen::Vector3d tanh_velocity_error =
    tanhVec(pos_gains_.K_V.cwiseProduct(velocity_error_ned));
  const Eigen::Vector3d velocity_feedback = pos_gains_.M_V.cwiseProduct(tanh_velocity_error);
  const Eigen::Vector3d acceleration_feedback =
    pos_gains_.K_Acceleration.cwiseProduct(linear_acceleration_ned);
  const auto thrustOverMassFromDisturbance = [&](const Eigen::Vector3d & disturbance) {
    return disturbance + gravity_ned + velocity_feedback + acceleration_feedback +
           ref.acceleration_ned;
  };

  Eigen::Vector3d thrust_over_mass_control =
    thrustOverMassFromDisturbance(velocity_disturbance_filtered);
  Eigen::Vector3d thrust_over_mass_observer =
    thrustOverMassFromDisturbance(velocity_disturbance_raw);

  applyTiltLimit(&thrust_over_mass_control, max_tilt_rad_);
  applyTiltLimit(&thrust_over_mass_observer, max_tilt_rad_);

  const Eigen::Vector3d desired_thrust_control = mass_ * thrust_over_mass_control;
  const Eigen::Vector3d desired_thrust_observer = mass_ * thrust_over_mass_observer;

  const Eigen::Vector3d velocity_error_hat_dot_ned =
    (-desired_thrust_observer / mass_) + gravity_ned + velocity_disturbance_raw;
  velocity_error_hat_ned_ += dt * velocity_error_hat_dot_ned;

  if (thrust_vec_ned) {
    *thrust_vec_ned = desired_thrust_control;
  }
  if (thrust_norm) {
    *thrust_norm = desired_thrust_control.norm();
  }
}

void TanhController::computeAttitude(
  const VehicleState & state, const AttitudeReference & attitude_reference, double dt,
  Eigen::Vector3d * torque_body)
{
  const Eigen::Quaterniond q = state.q_body_to_ned.normalized();
  const Eigen::Quaterniond q_d = attitude_reference.attitude_body_to_ned.normalized();

  // Trajectory feedforward is expressed in the reference-body frame and must be
  // rotated into the current body frame before it is used by the inner loop.
  const Eigen::Vector3d desired_angular_velocity_body =
    attitude_reference.has_angular_velocity_feedforward
      ? rotateReferenceBodyVectorToCurrentBody(
      q, q_d, attitude_reference.angular_velocity_body)
      : Eigen::Vector3d::Zero();
  const Eigen::Vector3d desired_angular_acceleration_body = Eigen::Vector3d::Zero();

  Eigen::Vector3d angular_acceleration_body = Eigen::Vector3d::Zero();
  if (has_last_angular_velocity_) {
    angular_acceleration_body = (state.angular_velocity_body - last_angular_velocity_body_) / dt;
  }
  if (!angular_acceleration_body.allFinite()) {
    angular_acceleration_body.setZero();
  }
  angular_acceleration_body =
    lowPassVec3(angular_acceleration_body, angular_accel_lpf_cutoff_hz_, dt,
      &angular_accel_lpf_state_body_, &angular_accel_lpf_initialized_);

  Eigen::Quaterniond q_error = q_d.conjugate() * q;
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }

  const Eigen::Vector3d attitude_error = q_error.vec();
  const Eigen::Vector3d tanh_attitude_error =
    tanhVec(att_gains_.K_Angle.cwiseProduct(attitude_error));
  const Eigen::Vector3d angular_velocity_error_body =
    (state.angular_velocity_body - desired_angular_velocity_body) +
    att_gains_.M_Angle.cwiseProduct(tanh_attitude_error);

  const Eigen::Vector3d angular_velocity_estimation_error_body =
    angular_velocity_error_body - angular_velocity_error_hat_body_;
  const Eigen::Vector3d angular_velocity_disturbance_raw = att_gains_.P_AngularVelocity.cwiseProduct(
    tanhVec(att_gains_.L_AngularVelocity.cwiseProduct(angular_velocity_estimation_error_body)));
  const Eigen::Vector3d angular_velocity_disturbance_filtered = lowPassVec3(
    angular_velocity_disturbance_raw, angular_velocity_disturbance_lpf_cutoff_hz_, dt,
    &angular_velocity_disturbance_lpf_state_body_, &angular_velocity_disturbance_lpf_initialized_);

  const Eigen::Vector3d omega_body = state.angular_velocity_body;
  const Eigen::Vector3d omega_cross_inertia_omega = omega_body.cross(inertia_ * omega_body);
  const Eigen::Matrix3d inertia_inv = inertia_.inverse();

  const Eigen::Vector3d tanh_angular_velocity_error =
    tanhVec(att_gains_.K_AngularVelocity.cwiseProduct(angular_velocity_error_body));
  const Eigen::Vector3d angular_velocity_control_term =
    att_gains_.M_AngularVelocity.cwiseProduct(tanh_angular_velocity_error);
  const Eigen::Vector3d angular_acceleration_error_body =
    angular_acceleration_body - desired_angular_acceleration_body;
  const Eigen::Vector3d feedforward_torque =
    attitude_reference.has_torque_feedforward
      ? rotateReferenceBodyVectorToCurrentBody(q, q_d, attitude_reference.torque_body)
      : Eigen::Vector3d::Zero();
  const Eigen::Vector3d angular_acceleration_feedback =
    inertia_ * att_gains_.K_AngularAcceleration.cwiseProduct(angular_acceleration_error_body);
  const auto desiredTorqueFromDisturbance =
    [&](const Eigen::Vector3d & angular_velocity_disturbance) {
      return feedforward_torque + omega_cross_inertia_omega -
             inertia_ * angular_velocity_disturbance - inertia_ * angular_velocity_control_term -
             angular_acceleration_feedback;
    };

  const Eigen::Vector3d desired_torque_control =
    desiredTorqueFromDisturbance(angular_velocity_disturbance_filtered);
  const Eigen::Vector3d desired_torque_observer =
    desiredTorqueFromDisturbance(angular_velocity_disturbance_raw);

  const Eigen::Vector3d angular_velocity_error_hat_dot_body =
    (-inertia_inv * omega_cross_inertia_omega) + inertia_inv * desired_torque_observer +
    angular_velocity_disturbance_raw;
  angular_velocity_error_hat_body_ += dt * angular_velocity_error_hat_dot_body;
  last_angular_velocity_body_ = state.angular_velocity_body;
  has_last_angular_velocity_ = true;

  if (torque_body) {
    *torque_body = desired_torque_control;
  }
}

}  // namespace tanh_ctrl
