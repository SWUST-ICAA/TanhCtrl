#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <string>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include "tanh_ctrl/msg/flat_trajectory_reference.hpp"
#include "tanh_ctrl/tanh_controller.hpp"

namespace tanh_ctrl
{

enum class MissionState
{
  WAIT_FOR_OFFBOARD,
  WAIT_FOR_ARMING,
  TAKEOFF,
  HOLD,
  TRACKING,
};

const char * toString(MissionState state);

MissionState nextMissionState(MissionState current, bool is_offboard, bool is_armed,
                              bool takeoff_complete, bool reference_fresh);

bool shouldPublishStartTrackingSignal(MissionState state, bool takeoff_complete,
                                      bool already_sent);

bool referenceMessageHasValidPosition(const msg::FlatTrajectoryReference & msg);

TrajectoryRef trajectoryReferenceFromMsg(const msg::FlatTrajectoryReference & msg,
                                         uint64_t timestamp_us);

TrajectoryRef makeHoldReference(const VehicleState & state, double target_z_ned, double yaw);

bool hasFreshExternalReference(const TrajectoryRef & external_ref, uint64_t now_us,
                               uint64_t last_reference_receive_us, double timeout_s);

class TanhNode : public rclcpp::Node
{
public:
  explicit TanhNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const std::string & sensorCombinedTopic() const { return topic_sensor_combined_; }
  const std::string & vehicleOdometryTopic() const { return topic_vehicle_odometry_; }
  const std::string & vehicleStatusV1Topic() const { return topic_vehicle_status_v1_; }
  const std::string & referenceTopic() const { return topic_reference_; }
  const std::string & actuatorMotorsTopic() const { return topic_actuator_motors_; }
  const std::string & offboardControlModeTopic() const { return topic_offboard_control_mode_; }
  const std::string & vehicleCommandTopic() const { return topic_vehicle_command_; }
  const std::string & vehicleThrustSetpointTopic() const
  {
    return topic_vehicle_thrust_setpoint_;
  }
  bool publishOffboardControlModeEnabled() const { return publish_offboard_control_mode_; }
  bool publishVehicleThrustSetpointEnabled() const
  {
    return publish_vehicle_thrust_setpoint_;
  }
  bool autoOffboardEnabled() const { return enable_auto_offboard_; }
  bool autoArmEnabled() const { return enable_auto_arm_; }
  int offboardWarmupSetpointCount() const { return offboard_setpoint_warmup_; }
  double motorForceMax() const { return motor_force_max_; }
  double thrustModelFactor() const { return thrust_model_factor_; }
  const PositionGains & positionGains() const { return controller_.getPositionGains(); }
  std::array<int, 4> motorOutputMap() const { return motor_output_map_; }
  double allocationBeta() const { return controller_.getAllocationParams().beta; }

private:
  void declareParameters();
  void loadParams();
  void loadGeneralParams();
  void loadModelParams();
  void loadPositionParams();
  void loadAttitudeParams();
  void loadFilterParams();
  void loadAllocationParams();
  void loadMotorOutputMap();
  void createRosInterfaces();

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void accelCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void referenceCallback(const msg::FlatTrajectoryReference::SharedPtr msg);
  void controlLoop();

  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);
  void publishStartTrackingSignal(bool enabled);
  void publishOffboardControlMode(uint64_t now_us);
  void publishMotorCommands(const ControlOutput & out, uint64_t now_us);
  void publishThrustSetpoint(const ControlOutput & out, uint64_t now_us);

  double computeControlDt(uint64_t now_us);
  void updateHoldReference(double target_z_ned);
  void updateCurrentHoldReference();
  void resetMissionProgress();
  void transitionToWaitState(MissionState next_state, const char * reason);
  void publishStartTrackingOnce();
  void resetTakeoffProgress();
  bool takeoffHoldComplete(uint64_t now_us);
  void handleMissionPreconditions();
  void maybeSendAutomaticRequests(uint64_t now_us);
  void updateMissionStateMachine(uint64_t now_us);
  const TrajectoryRef * selectActiveReference(uint64_t now_us) const;
  void setMissionState(MissionState next_state, const char * reason);

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

  VehicleState state_{};
  bool has_state_{false};
  TrajectoryRef external_ref_{};
  TrajectoryRef hold_ref_{};
  bool has_external_ref_{false};
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

  TanhController controller_{};

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
