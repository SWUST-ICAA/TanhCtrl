#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include "tanh_ctrl/msg/flat_trajectory_reference.hpp"
#include "tanh_ctrl/tanh_ctrl.hpp"

namespace tanh_ctrl
{

/**
 * @brief High-level mission phase managed by the controller node.
 */
enum class MissionState
{
  WAIT_FOR_OFFBOARD,
  WAIT_FOR_ARMING,
  TAKEOFF,
  HOLD,
  TRACKING,
};

/**
 * @brief Convert mission state to a readable string.
 *
 * @param state Mission state.
 * @return Human-readable state name.
 */
inline const char *toString(MissionState state)
{
  switch (state)
  {
  case MissionState::WAIT_FOR_OFFBOARD:
    return "WAIT_FOR_OFFBOARD";
  case MissionState::WAIT_FOR_ARMING:
    return "WAIT_FOR_ARMING";
  case MissionState::TAKEOFF:
    return "TAKEOFF";
  case MissionState::HOLD:
    return "HOLD";
  case MissionState::TRACKING:
    return "TRACKING";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief ROS2 PX4 adapter node for tanh_ctrl.
 *
 * The node subscribes to PX4 state topics and trajectory references, then
 * publishes direct-actuator commands to PX4.
 */
class tanh_ctrl_node : public rclcpp::Node
{
public:
  /**
   * @brief Construct the ROS2 node.
   *
   * @param options ROS2 node options.
   */
  explicit tanh_ctrl_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &sensorCombinedTopic() const
  {
    return topic_sensor_combined_;
  }
  const std::string &vehicleOdometryTopic() const
  {
    return topic_vehicle_odometry_;
  }
  const std::string &vehicleStatusV1Topic() const
  {
    return topic_vehicle_status_v1_;
  }
  const std::string &referenceTopic() const
  {
    return topic_reference_;
  }
  const std::string &actuatorMotorsTopic() const
  {
    return topic_actuator_motors_;
  }
  const std::string &offboardControlModeTopic() const
  {
    return topic_offboard_control_mode_;
  }
  const std::string &vehicleCommandTopic() const
  {
    return topic_vehicle_command_;
  }
  const std::string &vehicleThrustSetpointTopic() const
  {
    return topic_vehicle_thrust_setpoint_;
  }
  bool publishOffboardControlModeEnabled() const
  {
    return publish_offboard_control_mode_;
  }
  bool publishVehicleThrustSetpointEnabled() const
  {
    return publish_vehicle_thrust_setpoint_;
  }
  bool autoOffboardEnabled() const
  {
    return enable_auto_offboard_;
  }
  bool autoArmEnabled() const
  {
    return enable_auto_arm_;
  }
  int offboardWarmupSetpointCount() const
  {
    return offboard_setpoint_warmup_;
  }
  double motorForceMax() const
  {
    return motor_force_max_;
  }
  double thrustModelFactor() const
  {
    return thrust_model_factor_;
  }
  const PositionGains &positionGains() const
  {
    return controller_.getPositionGains();
  }
  std::array<int, 4> motorOutputMap() const
  {
    return motor_output_map_;
  }
  double allocationBeta() const
  {
    return controller_.getAllocationParams().beta;
  }

private:
  /**
   * @brief Declare all configurable ROS parameters.
   */
  void declareParameters();

  /**
   * @brief Create subscriptions, publishers, and timers.
   */
  void createRosInterfaces();

  /**
   * @brief Handle PX4 odometry updates.
   *
   * @param msg PX4 vehicle odometry message.
   */
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  /**
   * @brief Handle PX4 IMU acceleration updates.
   *
   * @param msg PX4 sensor-combined message.
   */
  void accelCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);

  /**
   * @brief Handle PX4 vehicle status updates.
   *
   * @param msg PX4 vehicle status message.
   */
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  /**
   * @brief Handle custom trajectory reference updates.
   *
   * @param msg Flatness-based streamed reference message.
   */
  void referenceCallback(const msg::FlatTrajectoryReference::SharedPtr msg);

  /**
   * @brief Run the periodic control loop.
   */
  void controlLoop();

  /**
   * @brief Publish a PX4 vehicle command.
   *
   * @param command MAVLink/PX4 command ID.
   * @param param1 Command parameter 1.
   * @param param2 Command parameter 2.
   * @param param3 Command parameter 3.
   */
  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);

  /**
   * @brief Publish the unified start-tracking signal.
   *
   * @param enabled Signal value to publish.
   */
  void publishStartTrackingSignal(bool enabled);

  /**
   * @brief Load ROS parameters and configure the controller.
   */
  void loadParams();

  /**
   * @brief Load topic names and behavior flags.
   */
  void loadGeneralParams();

  /**
   * @brief Load physical-model parameters into the controller.
   */
  void loadModelParams();

  /**
   * @brief Load outer-loop gains and mission references.
   */
  void loadPositionParams();

  /**
   * @brief Load attitude-loop gains.
   */
  void loadAttitudeParams();

  /**
   * @brief Load observer and acceleration filter settings.
   */
  void loadFilterParams();

  /**
   * @brief Load control-allocation geometry.
   */
  void loadAllocationParams();

  /**
   * @brief Load and validate the motor output permutation.
   */
  void loadMotorOutputMap();

  /**
   * @brief Compute the control-loop timestep from the current time.
   *
   * @param now_us Current ROS time in microseconds.
   * @return Clamped control-loop period [s].
   */
  double computeControlDt(uint64_t now_us);

  /**
   * @brief Publish PX4 offboard-control-mode heartbeat.
   *
   * @param now_us Current ROS time in microseconds.
   */
  void publishOffboardControlMode(uint64_t now_us);

  /**
   * @brief Set the internal hold reference using the current state.
   *
   * @param target_z_ned Desired hold altitude in NED [m].
   */
  void updateHoldReference(double target_z_ned);

  /**
   * @brief Refresh the hold reference at the current altitude.
   */
  void updateCurrentHoldReference();

  /**
   * @brief Reset mission progress and enter a wait/hold-style state.
   *
   * @param next_state State entered after resetting progress.
   * @param reason Transition reason for logging.
   */
  void transitionToWaitState(MissionState next_state, const char *reason);

  /**
   * @brief Reset the takeoff completion timers.
   */
  void resetTakeoffProgress();

  /**
   * @brief Check whether takeoff altitude has been held long enough.
   *
   * @param now_us Current ROS time in microseconds.
   * @return True when the takeoff hold condition is satisfied.
   */
  bool takeoffHoldComplete(uint64_t now_us);

  /**
   * @brief Publish the start-tracking signal only once.
   */
  void publishStartTrackingOnce();

  /**
   * @brief Check whether the external trajectory reference is still fresh.
   *
   * @param now_us Current ROS time in microseconds.
   * @return True when the latest trajectory message is recent enough.
   */
  bool hasFreshExternalReference(uint64_t now_us) const;

  /**
   * @brief Transition to another mission state and log the change.
   *
   * @param next_state New mission state.
   * @param reason Transition reason for logging.
   */
  void setMissionState(MissionState next_state, const char *reason);

  /**
   * @brief Reset mission progress flags when reverting to hold/wait states.
   */
  void resetMissionProgress();

  /**
   * @brief Apply mission-state transitions caused by arming/offboard changes.
   */
  void handleMissionPreconditions();

  /**
   * @brief Send automatic offboard/arm requests when enabled.
   *
   * @param now_us Current ROS time in microseconds.
   */
  void maybeSendAutomaticRequests(uint64_t now_us);

  /**
   * @brief Advance the high-level mission state machine.
   *
   * @param now_us Current ROS time in microseconds.
   */
  void updateMissionStateMachine(uint64_t now_us);

  /**
   * @brief Select the reference currently consumed by the controller.
   *
   * @param now_us Current ROS time in microseconds.
   * @return Active trajectory reference, or nullptr when none is valid.
   */
  const TrajectoryRef *selectActiveReference(uint64_t now_us) const;

  /**
   * @brief Publish direct actuator motor commands.
   *
   * @param out Controller output.
   * @param now_us Current ROS time in microseconds.
   */
  void publishMotorCommands(const ControlOutput &out, uint64_t now_us);

  /**
   * @brief Publish PX4 thrust setpoint heartbeat used by the land detector.
   *
   * @param out Controller output.
   * @param now_us Current ROS time in microseconds.
   */
  void publishThrustSetpoint(const ControlOutput &out, uint64_t now_us);

private:
  // ROS interfaces
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr accel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::FlatTrajectoryReference>::SharedPtr reference_sub_;

  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_tracking_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Cached state and references
  VehicleState state_{};
  bool has_state_{false};
  TrajectoryRef external_ref_{};
  bool has_external_ref_{false};
  TrajectoryRef hold_ref_{};
  bool has_hold_ref_{false};
  uint64_t last_control_us_{0};
  uint64_t last_reference_receive_us_{0};
  bool is_armed_{false};
  bool is_offboard_{false};
  bool offboard_ever_engaged_{false};
  bool exit_requested_{false};
  uint8_t last_nav_state_{0};
  double current_yaw_{0.0};
  MissionState mission_state_{MissionState::WAIT_FOR_OFFBOARD};
  bool takeoff_reached_{false};
  uint64_t takeoff_reached_since_us_{0};
  bool start_tracking_sent_{false};
  uint64_t last_offboard_request_us_{0};
  uint64_t last_arm_request_us_{0};

  // Controller core
  tanh_ctrl controller_{};

  // Topics and behavior parameters
  std::string topic_sensor_combined_;
  std::string topic_vehicle_odometry_;
  std::string topic_reference_;
  std::string topic_actuator_motors_;
  std::string topic_offboard_control_mode_;
  std::string topic_vehicle_command_;
  std::string topic_vehicle_thrust_setpoint_;
  std::string topic_start_tracking_;
  std::string topic_vehicle_status_v1_;

  double control_rate_hz_{100.0};
  bool publish_offboard_control_mode_{true};
  bool publish_vehicle_thrust_setpoint_{true};
  double motor_force_max_{8.54858};
  double thrust_model_factor_{1.0};
  double gravity_ned_{9.81};

  // Output permutation: out[i] = internal[motor_output_map_[i]]
  std::array<int, 4> motor_output_map_{{1, 3, 0, 2}};

  bool enable_auto_offboard_{true};
  bool enable_auto_arm_{false};
  int offboard_setpoint_warmup_{10};
  int offboard_counter_{0};
  double mission_takeoff_target_z_{-2.0};
  double mission_takeoff_z_threshold_{0.2};
  double mission_takeoff_hold_time_s_{2.0};
  double mission_reference_timeout_s_{0.3};
  double mission_request_interval_s_{1.0};
};

} // namespace tanh_ctrl
