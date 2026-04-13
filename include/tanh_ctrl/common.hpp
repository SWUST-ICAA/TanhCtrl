#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

/**
 * @brief Build a 3-axis vector with shared planar gains and an independent yaw/vertical axis.
 */
Eigen::Vector3d planarAxisVec(double planar, double axial);

/**
 * @brief Invert a PX4 THR_MDL_FAC-style normalized thrust model.
 */
double throttleFromRelativeThrust(double relative_thrust, double thrust_model_factor);

/**
 * @brief Replace non-finite scalars with zero.
 */
double sanitizeScalar(double value);

/**
 * @brief Replace non-finite vectors with zero.
 */
Eigen::Vector3d sanitizeVector(const Eigen::Vector3d & value);

/**
 * @brief Limit horizontal thrust so the thrust vector respects a tilt bound.
 */
void applyTiltLimit(Eigen::Vector3d * thrust_vector_over_mass_ned, double max_tilt_rad);

/**
 * @brief Reconstruct the desired body attitude from thrust direction and yaw.
 */
Eigen::Quaterniond computeDesiredAttitude(
  const Eigen::Vector3d & thrust_direction_ned, double yaw);

/**
 * @brief Rotate a vector from the reference-body frame into the current body frame.
 */
Eigen::Vector3d rotateReferenceBodyVectorToCurrentBody(
  const Eigen::Quaterniond & current_body_to_ned,
  const Eigen::Quaterniond & reference_body_to_ned,
  const Eigen::Vector3d & reference_body_vector);

/**
 * @brief Build the inner-loop attitude reference from thrust and trajectory feedforward.
 */
AttitudeReference computeAttitudeReference(
  const Eigen::Vector3d & desired_thrust_vector_ned, const TrajectoryRef & ref);

/**
 * @brief Solve the quad-X allocation for four motor thrusts.
 */
Eigen::Vector4d allocateMotorForces(
  const AllocationParams & params, double thrust_total, const Eigen::Vector3d & torque_body);

/**
 * @brief Map motor forces to normalized actuator controls.
 */
Eigen::Vector4d forcesToMotorControls(
  const Eigen::Vector4d & forces, double motor_force_max, double thrust_model_factor);

}  // namespace tanh_ctrl
