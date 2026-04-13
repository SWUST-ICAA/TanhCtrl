#pragma once

#include <Eigen/Dense>

#include <array>

#include "tanh_ctrl/common.hpp"

namespace tanh_ctrl {

/**
 * @brief Tanh feedback controller for quadrotor position and attitude stabilization.
 */
class TanhController
{
public:
  TanhController();

  void setMass(double mass);
  void setGravity(double gravity);
  void setPositionGains(const PositionGains & gains);
  void setAttitudeGains(const AttitudeGains & gains);
  void setInertia(const Eigen::Matrix3d & inertia);
  void setAllocationParams(const AllocationParams & alloc);
  void setMotorForceMax(double max_force);
  void setThrustModelFactor(double thrust_model_factor);

  const PositionGains & getPositionGains() const { return pos_gains_; }
  const AllocationParams & getAllocationParams() const { return alloc_; }

  void setMaxTilt(double max_tilt_rad);
  void setLinearAccelerationLowPassHz(const Eigen::Vector3d & cutoff_hz);
  void setAngularAccelerationLowPassHz(double cutoff_hz);
  void setVelocityDisturbanceLowPassHz(double cutoff_hz);
  void setAngularVelocityDisturbanceLowPassHz(double cutoff_hz);

  void reset();

  bool compute(const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out);

private:
  static double sanitizeCutoff(double cutoff_hz);
  static Eigen::Vector3d sanitizeCutoff(const Eigen::Vector3d & cutoff_hz);
  static Eigen::Vector3d tanhVec(const Eigen::Vector3d & x);
  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, double cutoff_hz, double dt,
    Eigen::Vector3d * state, bool * initialized);
  static Eigen::Vector3d lowPassVec3(
    const Eigen::Vector3d & x, const Eigen::Vector3d & cutoff_hz, double dt,
    Eigen::Vector3d * state, std::array<bool, 3> * initialized);
  static double lowPassScalar(
    double x, double cutoff_hz, double dt, double * state, bool * initialized);

  void computePosition(
    const VehicleState & state, const TrajectoryRef & ref, double dt,
    Eigen::Vector3d * thrust_vec_ned, double * thrust_norm);
  void computeAttitude(
    const VehicleState & state, const AttitudeReference & attitude_reference, double dt,
    Eigen::Vector3d * torque_body);

  double mass_{1.0};
  double gravity_{9.81};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  PositionGains pos_gains_{};
  AttitudeGains att_gains_{};
  AllocationParams alloc_{};
  double motor_force_max_{10.0};
  double thrust_model_factor_{1.0};
  double max_tilt_rad_{0.0};

  Eigen::Vector3d velocity_error_hat_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_error_hat_body_{Eigen::Vector3d::Zero()};
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
