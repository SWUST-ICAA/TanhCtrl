#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

/**
 * @brief Tanh feedback controller for quadrotor position and attitude stabilization.
 */
class tanh_ctrl
{
public:
  /**
   * @brief Construct the controller.
   */
  tanh_ctrl();

  /**
   * @brief Set vehicle mass.
   *
   * @param mass Vehicle mass [kg].
   */
  void setMass(double mass);

  /**
   * @brief Set gravity magnitude in the NED frame.
   *
   * @param gravity Gravity acceleration [m/s^2], positive down in NED.
   */
  void setGravity(double gravity);

  /**
   * @brief Set position-loop gains.
   *
   * @param gains Position-loop gains.
   */
  void setPositionGains(const PositionGains & gains);

  /**
   * @brief Set attitude-loop gains.
   *
   * @param gains Attitude-loop gains.
   */
  void setAttitudeGains(const AttitudeGains & gains);

  /**
   * @brief Set inertia matrix expressed in body FRD.
   *
   * @param inertia Inertia matrix [kg*m^2].
   */
  void setInertia(const Eigen::Matrix3d & inertia);

  /**
   * @brief Set allocation parameters.
   *
   * @param alloc Allocation geometry parameters.
   */
  void setAllocationParams(const AllocationParams & alloc);

  /**
   * @brief Set maximum single-motor thrust.
   *
   * @param max_force Maximum thrust [N].
   */
  void setMotorForceMax(double max_force);

  const AllocationParams & getAllocationParams() const { return alloc_; }

  /**
   * @brief Set the maximum tilt used by the outer loop.
   *
   * @param max_tilt_rad Maximum tilt angle [rad]. Non-positive disables the limit.
   */
  void setMaxTilt(double max_tilt_rad);

  /**
   * @brief Set low-pass cutoff for linear acceleration feedback.
   *
   * @param cutoff_hz Cutoff frequency [Hz]. Non-positive disables filtering.
   */
  void setLinearAccelerationLowPassHz(const Eigen::Vector3d & cutoff_hz);

  /**
   * @brief Set low-pass cutoff for angular acceleration estimation.
   *
   * @param cutoff_hz Cutoff frequency [Hz]. Non-positive disables filtering.
   */
  void setAngularAccelerationLowPassHz(double cutoff_hz);

  /**
   * @brief Set low-pass cutoff for the velocity disturbance estimate.
   *
   * @param cutoff_hz Cutoff frequency [Hz]. Non-positive disables filtering.
   */
  void setVelocityDisturbanceLowPassHz(double cutoff_hz);

  /**
   * @brief Set low-pass cutoff for the angular disturbance estimate.
   *
   * @param cutoff_hz Cutoff frequency [Hz]. Non-positive disables filtering.
   */
  void setAngularVelocityDisturbanceLowPassHz(double cutoff_hz);

  /**
   * @brief Reset observer and filter states.
   */
  void reset();

  /**
   * @brief Compute controller output.
   *
   * @param state Current vehicle state.
   * @param ref Current trajectory reference.
   * @param dt Sample period [s].
   * @param[out] out Controller output.
   * @return True on success. False when `ref` is invalid or `out` is null.
   */
  bool compute(const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out);

private:
  /**
   * @brief Clamp a scalar low-pass cutoff to a valid value.
   *
   * @param cutoff_hz Candidate cutoff frequency [Hz].
   * @return Positive finite cutoff, or zero when disabled.
   */
  static double sanitizeCutoff(double cutoff_hz);

  /**
   * @brief Clamp a per-axis low-pass cutoff vector to valid values.
   *
   * @param cutoff_hz Candidate cutoff frequencies [Hz].
   * @return Sanitized cutoff vector.
   */
  static Eigen::Vector3d sanitizeCutoff(const Eigen::Vector3d & cutoff_hz);

  /**
   * @brief Apply hyperbolic tangent element-wise.
   *
   * @param x Input vector.
   * @return Saturated vector.
   */
  static Eigen::Vector3d tanhVec(const Eigen::Vector3d & x);

  /**
   * @brief Apply a first-order low-pass filter element-wise.
   *
   * @param x Input vector.
   * @param cutoff_hz Cutoff frequency [Hz].
   * @param dt Sample period [s].
   * @param[in,out] state Filter state.
   * @param[in,out] initialized Filter initialization flag.
   * @return Filtered vector.
   */
  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, double cutoff_hz, double dt,
    Eigen::Vector3d * state, bool * initialized);

  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, const Eigen::Vector3d & cutoff_hz, double dt,
    Eigen::Vector3d * state, std::array<bool, 3> * initialized);

  static double lowPassScalar(
    double x, double cutoff_hz, double dt, double * state, bool * initialized);

  /**
   * @brief Run the position loop and velocity disturbance observer.
   *
   * @param state Current state.
   * @param ref Trajectory reference.
   * @param dt Sample period [s].
   * @param[out] thrust_vec_ned Desired thrust vector in NED [N].
   * @param[out] thrust_norm Desired collective thrust magnitude [N].
   */
  void computePosition(
    const VehicleState & state, const TrajectoryRef & ref, double dt,
    Eigen::Vector3d * thrust_vec_ned, double * thrust_norm);

  /**
   * @brief Run the attitude loop and angular disturbance observer.
   *
   * @param state Current state.
   * @param attitude_reference Desired attitude reconstructed from the outer loop.
   * @param dt Sample period [s].
   * @param[out] torque_body Desired body torque in FRD [N*m].
   */
  void computeAttitude(
    const VehicleState & state, const AttitudeReference & attitude_reference, double dt,
    Eigen::Vector3d * torque_body);

private:
  double mass_{1.0};
  double gravity_{9.81};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  PositionGains pos_gains_{};
  AttitudeGains att_gains_{};
  AllocationParams alloc_{};
  double motor_force_max_{10.0};
  double max_tilt_rad_{0.0};

  Eigen::Vector3d velocity_error_hat_ned_{Eigen::Vector3d::Zero()};  ///< Velocity observer state in NED.
  Eigen::Vector3d angular_velocity_error_hat_body_{Eigen::Vector3d::Zero()};  ///< Angular-rate observer state in FRD.
  Eigen::Vector3d last_angular_velocity_body_{Eigen::Vector3d::Zero()};
  bool has_last_angular_velocity_{false};
  bool first_run_{true};

  Eigen::Vector3d linear_accel_lpf_cutoff_hz_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_accel_lpf_state_ned_{Eigen::Vector3d::Zero()};
  std::array<bool, 3> linear_accel_lpf_initialized_{{false, false, false}};

  double angular_accel_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d angular_accel_lpf_state_body_{Eigen::Vector3d::Zero()};
  bool angular_accel_lpf_initialized_{false};

  double velocity_disturbance_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d velocity_disturbance_lpf_state_ned_{Eigen::Vector3d::Zero()};
  bool velocity_disturbance_lpf_initialized_{false};

  double angular_velocity_disturbance_lpf_cutoff_hz_{0.0};
  Eigen::Vector3d angular_velocity_disturbance_lpf_state_body_{Eigen::Vector3d::Zero()};
  bool angular_velocity_disturbance_lpf_initialized_{false};
};

}  // namespace tanh_ctrl
