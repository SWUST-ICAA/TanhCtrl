#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace tanh_ctrl {

/**
 * @brief Quadrotor state expressed in the controller frames.
 *
 * Inertial vectors use the PX4 world convention `NED`.
 * Body-frame vectors use the PX4 body convention `FRD`.
 */
struct VehicleState {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};             ///< Position in NED [m].
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};             ///< Velocity in NED [m/s].
  Eigen::Vector3d linear_acceleration_ned{Eigen::Vector3d::Zero()};  ///< Linear acceleration in NED [m/s^2].
  Eigen::Quaterniond q_body_to_ned{Eigen::Quaterniond::Identity()};  ///< Attitude from body FRD to NED.
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};    ///< Angular velocity in body FRD [rad/s].
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};  ///< Angular acceleration in body FRD [rad/s^2].
};

/**
 * @brief Trajectory reference for the outer-loop.
 */
struct TrajectoryRef {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};           ///< Desired position in NED [m].
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};           ///< Desired velocity in NED [m/s].
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};       ///< Desired acceleration in NED [m/s^2].
  Eigen::Vector3d jerk_ned{Eigen::Vector3d::Zero()};               ///< Desired jerk in NED [m/s^3].
  Eigen::Vector3d snap_ned{Eigen::Vector3d::Zero()};               ///< Desired snap in NED [m/s^4].
  double yaw{0.0};                                                 ///< Desired yaw angle [rad].
  double yaw_rate{0.0};                                            ///< Desired yaw rate [rad/s].
  double yaw_acceleration{0.0};                                    ///< Desired yaw acceleration [rad/s^2].
  bool has_flatness_feedforward{false};                            ///< True when jerk/snap/yaw derivative feedforward is valid.
  bool valid{false};                                               ///< True when the reference is valid.
};

/**
 * @brief Position-loop gains.
 */
struct PositionGains {
  Eigen::Vector3d M_P{Eigen::Vector3d::Ones()};             ///< Position tanh scale.
  Eigen::Vector3d K_P{Eigen::Vector3d::Ones()};             ///< Position tanh slope.
  Eigen::Vector3d M_V{Eigen::Vector3d::Ones()};             ///< Velocity tanh scale.
  Eigen::Vector3d K_V{Eigen::Vector3d::Ones()};             ///< Velocity tanh slope.
  Eigen::Vector3d K_Acceleration{Eigen::Vector3d::Zero()};  ///< Acceleration feedback gain.
  Eigen::Vector3d P_V{Eigen::Vector3d::Zero()};             ///< Velocity disturbance observer gain.
  Eigen::Vector3d L_V{Eigen::Vector3d::Ones()};             ///< Velocity disturbance observer slope.
};

/**
 * @brief Attitude-loop gains.
 */
struct AttitudeGains {
  Eigen::Vector3d M_Angle{Eigen::Vector3d::Ones()};                ///< Attitude tanh scale.
  Eigen::Vector3d K_Angle{Eigen::Vector3d::Ones()};                ///< Attitude tanh slope.
  Eigen::Vector3d M_AngularVelocity{Eigen::Vector3d::Ones()};      ///< Angular-rate tanh scale.
  Eigen::Vector3d K_AngularVelocity{Eigen::Vector3d::Ones()};      ///< Angular-rate tanh slope.
  Eigen::Vector3d K_AngularAcceleration{Eigen::Vector3d::Zero()};  ///< Angular-acceleration gain.
  Eigen::Vector3d P_AngularVelocity{Eigen::Vector3d::Zero()};      ///< Angular disturbance observer gain.
  Eigen::Vector3d L_AngularVelocity{Eigen::Vector3d::Ones()};      ///< Angular disturbance observer slope.
};

/**
 * @brief Control allocation parameters for the quadrotor mixer matrix.
 */
struct AllocationParams {
  double l{0.2};        ///< Arm length [m].
  double beta{M_PI_4};  ///< Arm angle w.r.t. body x-axis [rad].
  double cq_ct{0.01};   ///< Rotor drag-to-thrust ratio c_q / c_t.
};

/**
 * @brief Desired attitude reconstructed from the outer-loop thrust vector.
 */
struct AttitudeReference {
  Eigen::Quaterniond attitude_body_to_ned{Eigen::Quaterniond::Identity()};  ///< Desired attitude from body FRD to NED.
  Eigen::Vector3d jerk_ned{Eigen::Vector3d::Zero()};                        ///< Desired jerk in NED [m/s^3].
  Eigen::Vector3d snap_ned{Eigen::Vector3d::Zero()};                        ///< Desired snap in NED [m/s^4].
  Eigen::Vector3d thrust_direction_ned{Eigen::Vector3d::UnitZ()};           ///< Desired body z-axis in NED.
  double collective_thrust{0.0};                                            ///< Desired collective thrust magnitude [N].
  double yaw_rate{0.0};                                                     ///< Desired yaw rate [rad/s].
  double yaw_acceleration{0.0};                                             ///< Desired yaw acceleration [rad/s^2].
  bool has_flatness_feedforward{false};                                     ///< True when jerk/snap/yaw derivative feedforward is valid.
  bool valid{false};                                                        ///< True when the reference is valid.
};

/**
 * @brief Controller output after allocation.
 */
struct ControlOutput {
  double thrust_total{0.0};                                 ///< Total thrust [N].
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};     ///< Desired body torque in FRD [N*m].
  Eigen::Vector4d motor_forces{Eigen::Vector4d::Zero()};    ///< Per-motor thrust [N].
  Eigen::Vector4d motor_controls{Eigen::Vector4d::Zero()};  ///< Normalized motor commands in [0, 1].
};

}  // namespace tanh_ctrl
