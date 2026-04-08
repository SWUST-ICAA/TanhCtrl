#include "tanh_ctrl/tanh_ctrl_node.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace tanh_ctrl {

namespace {

uint64_t nowMicros(rclcpp::Clock & clock)
{
  return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
}

double elapsedSeconds(uint64_t now_us, uint64_t previous_us)
{
  if (previous_us == 0 || now_us <= previous_us) {
    return 0.0;
  }
  return static_cast<double>(now_us - previous_us) * 1e-6;
}

double quaternionToYaw(const Eigen::Quaterniond & q_body_to_ned)
{
  const Eigen::Quaterniond q = q_body_to_ned.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

template<typename Array3T>
bool isFiniteVec3(const Array3T & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

constexpr char kSensorCombinedTopic[] = "/fmu/out/sensor_combined";
constexpr char kVehicleOdometryTopic[] = "/fmu/out/vehicle_odometry";
constexpr char kVehicleStatusV1Topic[] = "/fmu/out/vehicle_status_v1";
constexpr char kActuatorMotorsTopic[] = "/fmu/in/actuator_motors";
constexpr char kOffboardControlModeTopic[] = "/fmu/in/offboard_control_mode";
constexpr char kVehicleCommandTopic[] = "/fmu/in/vehicle_command";
constexpr char kVehicleThrustSetpointTopic[] = "/fmu/in/vehicle_thrust_setpoint";
constexpr bool kPublishOffboardControlMode = true;
constexpr bool kPublishVehicleThrustSetpoint = true;
constexpr bool kAutoOffboard = true;
constexpr bool kAutoArm = false;
constexpr int kOffboardWarmup = 10;
constexpr double kAllocationBeta = 0.78539816339;
constexpr std::array<int, 4> kMotorOutputMap{{1, 3, 0, 2}};

}  // namespace

tanh_ctrl_node::tanh_ctrl_node(const rclcpp::NodeOptions & options)
: Node("tanh_ctrl", options)
{
  declareParameters();
  loadParams();
  createRosInterfaces();
}

void tanh_ctrl_node::declareParameters()
{
  this->declare_parameter<std::string>(
    "topics.trajectory_setpoint", "/tanh_ctrl/trajectory_setpoint");
  this->declare_parameter<std::string>("topics.start_tracking", "/mission/start_tracking");

  this->declare_parameter<double>("control_rate_hz", 100.0);
  this->declare_parameter<double>("mission.takeoff_target_z", -2.0);
  this->declare_parameter<double>("mission.takeoff_z_threshold", 0.2);
  this->declare_parameter<double>("mission.takeoff_hold_time_s", 2.0);
  this->declare_parameter<double>("mission.reference_timeout_s", 0.3);
  this->declare_parameter<double>("mission.request_interval_s", 1.0);

  this->declare_parameter<double>("model.mass", 2.0643076923076915);
  this->declare_parameter<double>("model.gravity", 9.81);
  this->declare_parameter<double>("model.force_max", 8.54858);
  this->declare_parameter<std::vector<double>>(
    "model.inertia_diag", {0.02384669, 0.02394962, 0.04399995});

  this->declare_parameter<std::vector<double>>("position.M_P", {1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("position.K_P", {1.5, 1.5, 1.5});
  this->declare_parameter<std::vector<double>>("position.M_V", {3.0, 3.0, 3.0});
  this->declare_parameter<std::vector<double>>("position.K_V", {1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("position.K_Acceleration", {0.0, 0.0, 0.0});
  this->declare_parameter<double>("position.horizontal.M_P", 2.5);
  this->declare_parameter<double>("position.vertical.M_P", 2.0);
  this->declare_parameter<double>("position.horizontal.K_P", 1.0);
  this->declare_parameter<double>("position.vertical.K_P", 1.0);
  this->declare_parameter<double>("position.horizontal.M_V", 8.0);
  this->declare_parameter<double>("position.vertical.M_V", 6.5);
  this->declare_parameter<double>("position.horizontal.K_V", 0.5);
  this->declare_parameter<double>("position.vertical.K_V", 0.5);
  this->declare_parameter<double>("position.horizontal.K_Acceleration", 1.1);
  this->declare_parameter<double>("position.vertical.K_Acceleration", 1.0);
  this->declare_parameter<double>("position.max_tilt_deg", 35.0);
  this->declare_parameter<std::vector<double>>("position.observer.P_V", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("position.observer.L_V", {5.0, 5.0, 5.0});

  this->declare_parameter<double>("attitude.tilt.M_Angle", 3.0);
  this->declare_parameter<double>("attitude.yaw.M_Angle", 3.0);
  this->declare_parameter<double>("attitude.tilt.K_Angle", 4.0);
  this->declare_parameter<double>("attitude.yaw.K_Angle", 4.0);
  this->declare_parameter<double>("attitude.tilt.M_AngularVelocity", 20.0);
  this->declare_parameter<double>("attitude.yaw.M_AngularVelocity", 15.0);
  this->declare_parameter<double>("attitude.tilt.K_AngularVelocity", 2.0);
  this->declare_parameter<double>("attitude.yaw.K_AngularVelocity", 2.0);
  this->declare_parameter<double>("attitude.tilt.K_AngularAcceleration", 0.0);
  this->declare_parameter<double>("attitude.yaw.K_AngularAcceleration", 0.0);
  this->declare_parameter<double>("attitude.tilt.observer.P_AngularVelocity", 0.0);
  this->declare_parameter<double>("attitude.yaw.observer.P_AngularVelocity", 0.0);
  this->declare_parameter<double>("attitude.tilt.observer.L_AngularVelocity", 5.0);
  this->declare_parameter<double>("attitude.yaw.observer.L_AngularVelocity", 5.0);

  this->declare_parameter<double>("filters.linear_accel_cutoff_hz", 0.0);
  this->declare_parameter<double>(
    "filters.linear.horizontal_cutoff_hz", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<double>(
    "filters.linear.vertical_cutoff_hz", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<double>("filters.angular_accel_cutoff_hz", 0.0);
  this->declare_parameter<double>("filters.velocity_disturbance_cutoff_hz", 0.0);
  this->declare_parameter<double>(
    "filters.angular_velocity_disturbance_cutoff_hz", 0.0);

  this->declare_parameter<double>("allocation.l", 0.246073);
  this->declare_parameter<double>("allocation.cq_ct", 0.016);
}

void tanh_ctrl_node::createRosInterfaces()
{
  const auto qos_px4_out = rclcpp::SensorDataQoS();
  const auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    topic_vehicle_odometry_,
    qos_px4_out,
    std::bind(&tanh_ctrl_node::odomCallback, this, std::placeholders::_1));
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_vehicle_status_v1_,
    qos_px4_out,
    std::bind(&tanh_ctrl_node::vehicleStatusCallback, this, std::placeholders::_1));

  accel_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
    topic_sensor_combined_,
    qos_px4_out,
    std::bind(&tanh_ctrl_node::accelCallback, this, std::placeholders::_1));
  setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    topic_trajectory_setpoint_,
    qos_default,
    std::bind(&tanh_ctrl_node::setpointCallback, this, std::placeholders::_1));

  motors_pub_ =
    this->create_publisher<px4_msgs::msg::ActuatorMotors>(topic_actuator_motors_, qos_default);
  offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    topic_offboard_control_mode_, qos_default);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    topic_vehicle_command_, qos_default);
  thrust_sp_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
    topic_vehicle_thrust_setpoint_, qos_default);
  start_tracking_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    topic_start_tracking_, rclcpp::QoS(1).reliable().transient_local());
  publishStartTrackingSignal(false);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&tanh_ctrl_node::controlLoop, this));
}

void tanh_ctrl_node::loadParams()
{
  loadGeneralParams();
  loadModelParams();
  loadPositionParams();
  loadAttitudeParams();
  loadFilterParams();
  loadAllocationParams();
  loadMotorOutputMap();
}

void tanh_ctrl_node::loadGeneralParams()
{
  topic_sensor_combined_ = kSensorCombinedTopic;
  topic_vehicle_odometry_ = kVehicleOdometryTopic;
  topic_vehicle_status_v1_ = kVehicleStatusV1Topic;
  topic_trajectory_setpoint_ = this->get_parameter("topics.trajectory_setpoint").as_string();
  topic_actuator_motors_ = kActuatorMotorsTopic;
  topic_offboard_control_mode_ = kOffboardControlModeTopic;
  topic_vehicle_command_ = kVehicleCommandTopic;
  topic_vehicle_thrust_setpoint_ = kVehicleThrustSetpointTopic;
  topic_start_tracking_ = this->get_parameter("topics.start_tracking").as_string();

  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  publish_offboard_control_mode_ = kPublishOffboardControlMode;
  publish_vehicle_thrust_setpoint_ = kPublishVehicleThrustSetpoint;
  enable_auto_offboard_ = kAutoOffboard;
  enable_auto_arm_ = kAutoArm;
  offboard_setpoint_warmup_ = kOffboardWarmup;

  mission_takeoff_target_z_ = this->get_parameter("mission.takeoff_target_z").as_double();
  mission_takeoff_z_threshold_ =
    std::max(0.0, this->get_parameter("mission.takeoff_z_threshold").as_double());
  mission_takeoff_hold_time_s_ =
    std::max(0.0, this->get_parameter("mission.takeoff_hold_time_s").as_double());
  mission_reference_timeout_s_ =
    std::max(0.0, this->get_parameter("mission.reference_timeout_s").as_double());
  mission_request_interval_s_ =
    std::max(0.1, this->get_parameter("mission.request_interval_s").as_double());
}

void tanh_ctrl_node::loadModelParams()
{
  controller_.setMass(this->get_parameter("model.mass").as_double());
  gravity_ned_ = this->get_parameter("model.gravity").as_double();
  controller_.setGravity(gravity_ned_);

  const auto inertia_diag = this->get_parameter("model.inertia_diag").as_double_array();
  if (inertia_diag.size() != 3) {
    RCLCPP_WARN(this->get_logger(), "model.inertia_diag长度不是3，使用单位阵");
    controller_.setInertia(Eigen::Matrix3d::Identity());
    return;
  }

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = inertia_diag[0];
  inertia(1, 1) = inertia_diag[1];
  inertia(2, 2) = inertia_diag[2];
  controller_.setInertia(inertia);
}

void tanh_ctrl_node::loadPositionParams()
{
  PositionGains gains;
  gains.M_P = planarAxisVec(
    this->get_parameter("position.horizontal.M_P").as_double(),
    this->get_parameter("position.vertical.M_P").as_double());
  gains.K_P = planarAxisVec(
    this->get_parameter("position.horizontal.K_P").as_double(),
    this->get_parameter("position.vertical.K_P").as_double());
  gains.M_V = planarAxisVec(
    this->get_parameter("position.horizontal.M_V").as_double(),
    this->get_parameter("position.vertical.M_V").as_double());
  gains.K_V = planarAxisVec(
    this->get_parameter("position.horizontal.K_V").as_double(),
    this->get_parameter("position.vertical.K_V").as_double());
  gains.K_Acceleration = planarAxisVec(
    this->get_parameter("position.horizontal.K_Acceleration").as_double(),
    this->get_parameter("position.vertical.K_Acceleration").as_double());
  gains.P_V = getVec3Param(*this, "position.observer.P_V");
  gains.L_V = getVec3Param(*this, "position.observer.L_V");
  controller_.setPositionGains(gains);

  const double max_tilt_deg = this->get_parameter("position.max_tilt_deg").as_double();
  controller_.setMaxTilt(max_tilt_deg * M_PI / 180.0);
}

void tanh_ctrl_node::loadAttitudeParams()
{
  AttitudeGains gains;
  gains.M_Angle = planarAxisVec(
    this->get_parameter("attitude.tilt.M_Angle").as_double(),
    this->get_parameter("attitude.yaw.M_Angle").as_double());
  gains.K_Angle = planarAxisVec(
    this->get_parameter("attitude.tilt.K_Angle").as_double(),
    this->get_parameter("attitude.yaw.K_Angle").as_double());
  gains.M_AngularVelocity = planarAxisVec(
    this->get_parameter("attitude.tilt.M_AngularVelocity").as_double(),
    this->get_parameter("attitude.yaw.M_AngularVelocity").as_double());
  gains.K_AngularVelocity = planarAxisVec(
    this->get_parameter("attitude.tilt.K_AngularVelocity").as_double(),
    this->get_parameter("attitude.yaw.K_AngularVelocity").as_double());
  gains.K_AngularAcceleration = planarAxisVec(
    this->get_parameter("attitude.tilt.K_AngularAcceleration").as_double(),
    this->get_parameter("attitude.yaw.K_AngularAcceleration").as_double());
  gains.P_AngularVelocity = planarAxisVec(
    this->get_parameter("attitude.tilt.observer.P_AngularVelocity").as_double(),
    this->get_parameter("attitude.yaw.observer.P_AngularVelocity").as_double());
  gains.L_AngularVelocity = planarAxisVec(
    this->get_parameter("attitude.tilt.observer.L_AngularVelocity").as_double(),
    this->get_parameter("attitude.yaw.observer.L_AngularVelocity").as_double());
  controller_.setAttitudeGains(gains);
}

void tanh_ctrl_node::loadFilterParams()
{
  const double linear_horizontal_cutoff =
    this->get_parameter("filters.linear.horizontal_cutoff_hz").as_double();
  const double linear_vertical_cutoff =
    this->get_parameter("filters.linear.vertical_cutoff_hz").as_double();
  const double legacy_linear_cutoff =
    this->get_parameter("filters.linear_accel_cutoff_hz").as_double();

  const double horizontal_cutoff =
    std::isfinite(linear_horizontal_cutoff) ?
    linear_horizontal_cutoff :
    (std::isfinite(legacy_linear_cutoff) ? legacy_linear_cutoff : 0.0);
  const double vertical_cutoff =
    std::isfinite(linear_vertical_cutoff) ?
    linear_vertical_cutoff :
    (std::isfinite(legacy_linear_cutoff) ? legacy_linear_cutoff : horizontal_cutoff);

  controller_.setLinearAccelerationLowPassHz(
    Eigen::Vector3d(horizontal_cutoff, horizontal_cutoff, vertical_cutoff));
  controller_.setAngularAccelerationLowPassHz(
    this->get_parameter("filters.angular_accel_cutoff_hz").as_double());
  controller_.setVelocityDisturbanceLowPassHz(
    this->get_parameter("filters.velocity_disturbance_cutoff_hz").as_double());
  controller_.setAngularVelocityDisturbanceLowPassHz(
    this->get_parameter("filters.angular_velocity_disturbance_cutoff_hz").as_double());
}

void tanh_ctrl_node::loadAllocationParams()
{
  AllocationParams params;
  params.l = this->get_parameter("allocation.l").as_double();
  params.beta = kAllocationBeta;
  params.cq_ct = this->get_parameter("allocation.cq_ct").as_double();
  controller_.setAllocationParams(params);

  motor_force_max_ = this->get_parameter("model.force_max").as_double();
  controller_.setMotorForceMax(motor_force_max_);
}

void tanh_ctrl_node::loadMotorOutputMap()
{
  motor_output_map_ = kMotorOutputMap;
}

Eigen::Vector3d tanh_ctrl_node::getVec3Param(rclcpp::Node & node, const std::string & name)
{
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != 3) {
    throw std::runtime_error("参数" + name + "长度必须为3");
  }

  return Eigen::Vector3d(values[0], values[1], values[2]);
}

double tanh_ctrl_node::computeControlDt(uint64_t now_us)
{
  double dt = 1.0 / std::max(1.0, control_rate_hz_);
  if (last_control_us_ != 0 && now_us > last_control_us_) {
    dt = static_cast<double>(now_us - last_control_us_) * 1e-6;
  }
  last_control_us_ = now_us;
  return std::clamp(dt, 1e-4, 0.1);
}

void tanh_ctrl_node::publishOffboardControlMode(uint64_t now_us)
{
  if (!publish_offboard_control_mode_ || !offboard_mode_pub_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode mode{};
  mode.timestamp = now_us;
  mode.position = false;
  mode.velocity = false;
  mode.acceleration = false;
  mode.attitude = false;
  mode.body_rate = false;
  mode.thrust_and_torque = false;
  mode.direct_actuator = true;
  offboard_mode_pub_->publish(mode);
}

void tanh_ctrl_node::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool position_ok = isFiniteVec3(msg->position);
  const bool quaternion_ok =
    std::isfinite(msg->q[0]) && std::isfinite(msg->q[1]) &&
    std::isfinite(msg->q[2]) && std::isfinite(msg->q[3]);
  if (!position_ok || !quaternion_ok) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "vehicle_odometry包含无效数据(position/q为NaN)，等待估计器就绪...");
    return;
  }

  if (last_odom_us_ != 0 && msg->timestamp > last_odom_us_) {
    const double odom_dt = static_cast<double>(msg->timestamp - last_odom_us_) * 1e-6;
    last_odom_dt_ = std::clamp(odom_dt, 1e-4, 0.1);
  }
  last_odom_us_ = msg->timestamp;

  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "vehicle_odometry.pose_frame不是NED(%u)，当前控制器按NED解释，可能导致不稳定",
      static_cast<unsigned>(msg->pose_frame));
  }

  Eigen::Quaterniond attitude(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  attitude.normalize();
  state_.q_body_to_ned = attitude;
  current_yaw_ = quaternionToYaw(attitude);

  state_.position_ned = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
  state_.angular_velocity_body = Eigen::Vector3d(
    std::isfinite(msg->angular_velocity[0]) ? msg->angular_velocity[0] : 0.0,
    std::isfinite(msg->angular_velocity[1]) ? msg->angular_velocity[1] : 0.0,
    std::isfinite(msg->angular_velocity[2]) ? msg->angular_velocity[2] : 0.0);

  const Eigen::Vector3d velocity_raw(
    std::isfinite(msg->velocity[0]) ? msg->velocity[0] : 0.0,
    std::isfinite(msg->velocity[1]) ? msg->velocity[1] : 0.0,
    std::isfinite(msg->velocity[2]) ? msg->velocity[2] : 0.0);

  if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
    state_.velocity_ned = velocity_raw;
  } else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
    state_.velocity_ned = attitude * velocity_raw;
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "vehicle_odometry.velocity_frame=%u未处理，当前直接按NED使用，可能导致不稳定",
      static_cast<unsigned>(msg->velocity_frame));
    state_.velocity_ned = velocity_raw;
  }

  has_state_ = true;
}

void tanh_ctrl_node::vehicleStatusCallback(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool was_offboard = is_offboard_;
  const uint8_t previous_nav_state = last_nav_state_;

  last_nav_state_ = msg->nav_state;
  is_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  is_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

  if (is_offboard_) {
    offboard_ever_engaged_ = true;
  }

  if (offboard_ever_engaged_ && was_offboard && !is_offboard_ && !exit_requested_) {
    exit_requested_ = true;
    RCLCPP_ERROR(
      this->get_logger(),
      "Detected offboard exit, nav_state changed from %u to %u. Controller will shut down.",
      static_cast<unsigned>(previous_nav_state),
      static_cast<unsigned>(msg->nav_state));
  }
}

void tanh_ctrl_node::accelCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
  if (!msg || !has_state_) {
    return;
  }

  const float ax = msg->accelerometer_m_s2[0];
  const float ay = msg->accelerometer_m_s2[1];
  const float az = msg->accelerometer_m_s2[2];
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
    return;
  }

  const Eigen::Vector3d accel_body(ax, ay, az);
  Eigen::Vector3d accel_ned = state_.q_body_to_ned * accel_body;
  if (!accel_ned.allFinite()) {
    state_.linear_acceleration_ned.setZero();
    return;
  }

  accel_ned.z() += gravity_ned_;
  state_.linear_acceleration_ned = accel_ned;
}

void tanh_ctrl_node::setpointCallback(
  const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!isFiniteVec3(msg->position)) {
    external_ref_.valid = false;
    has_external_ref_ = false;
    return;
  }

  external_ref_.position_ned =
    Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
  external_ref_.yaw = std::isfinite(msg->yaw) ? static_cast<double>(msg->yaw) : 0.0;
  external_ref_.velocity_ned = Eigen::Vector3d(
    std::isfinite(msg->velocity[0]) ? static_cast<double>(msg->velocity[0]) : 0.0,
    std::isfinite(msg->velocity[1]) ? static_cast<double>(msg->velocity[1]) : 0.0,
    std::isfinite(msg->velocity[2]) ? static_cast<double>(msg->velocity[2]) : 0.0);
  external_ref_.acceleration_ned = Eigen::Vector3d(
    std::isfinite(msg->acceleration[0]) ? static_cast<double>(msg->acceleration[0]) : 0.0,
    std::isfinite(msg->acceleration[1]) ? static_cast<double>(msg->acceleration[1]) : 0.0,
    std::isfinite(msg->acceleration[2]) ? static_cast<double>(msg->acceleration[2]) : 0.0);
  external_ref_.jerk_ned = Eigen::Vector3d(
    std::isfinite(msg->jerk[0]) ? static_cast<double>(msg->jerk[0]) : 0.0,
    std::isfinite(msg->jerk[1]) ? static_cast<double>(msg->jerk[1]) : 0.0,
    std::isfinite(msg->jerk[2]) ? static_cast<double>(msg->jerk[2]) : 0.0);
  external_ref_.yaw_rate =
    std::isfinite(msg->yawspeed) ? static_cast<double>(msg->yawspeed) : 0.0;

  const uint64_t reference_timestamp_us =
    msg->timestamp != 0 ? msg->timestamp : nowMicros(*this->get_clock());
  external_ref_.snap_ned.setZero();
  external_ref_.yaw_acceleration = 0.0;

  if (has_last_reference_derivatives_ && reference_timestamp_us > last_reference_timestamp_us_) {
    const double derivative_dt = std::clamp(
      static_cast<double>(reference_timestamp_us - last_reference_timestamp_us_) * 1e-6,
      1e-4,
      0.1);
    external_ref_.snap_ned =
      (external_ref_.jerk_ned - last_reference_jerk_ned_) / derivative_dt;
    external_ref_.yaw_acceleration =
      (external_ref_.yaw_rate - last_reference_yaw_rate_) / derivative_dt;
  }

  last_reference_timestamp_us_ = reference_timestamp_us;
  last_reference_receive_us_ = nowMicros(*this->get_clock());
  last_reference_jerk_ned_ = external_ref_.jerk_ned;
  last_reference_yaw_rate_ = external_ref_.yaw_rate;
  has_last_reference_derivatives_ = true;

  external_ref_.valid = true;
  has_external_ref_ = true;
}

void tanh_ctrl_node::publishVehicleCommand(
  uint32_t command, float param1, float param2, float param3)
{
  if (!vehicle_command_pub_) {
    return;
  }

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = nowMicros(*this->get_clock());
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.command = command;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  vehicle_command_pub_->publish(cmd);
}

void tanh_ctrl_node::publishStartTrackingSignal(bool enabled)
{
  if (!start_tracking_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = enabled;
  start_tracking_pub_->publish(msg);
}

void tanh_ctrl_node::updateHoldReference(double target_z_ned)
{
  if (!has_state_) {
    return;
  }

  hold_ref_.position_ned =
    Eigen::Vector3d(state_.position_ned.x(), state_.position_ned.y(), target_z_ned);
  hold_ref_.velocity_ned.setZero();
  hold_ref_.acceleration_ned.setZero();
  hold_ref_.jerk_ned.setZero();
  hold_ref_.snap_ned.setZero();
  hold_ref_.yaw = current_yaw_;
  hold_ref_.yaw_rate = 0.0;
  hold_ref_.yaw_acceleration = 0.0;
  hold_ref_.valid = true;
  has_hold_ref_ = true;
}

bool tanh_ctrl_node::hasFreshExternalReference(uint64_t now_us) const
{
  if (!has_external_ref_ || !external_ref_.valid) {
    return false;
  }
  if (mission_reference_timeout_s_ <= 0.0) {
    return true;
  }
  if (last_reference_receive_us_ == 0 || now_us <= last_reference_receive_us_) {
    return false;
  }

  return elapsedSeconds(now_us, last_reference_receive_us_) <= mission_reference_timeout_s_;
}

void tanh_ctrl_node::setMissionState(MissionState next_state, const char * reason)
{
  if (mission_state_ == next_state) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Mission state: %s -> %s (%s)",
    toString(mission_state_),
    toString(next_state),
    reason);
  mission_state_ = next_state;
}

void tanh_ctrl_node::resetMissionProgress()
{
  publishStartTrackingSignal(false);
  start_tracking_sent_ = false;
  takeoff_reached_ = false;
  takeoff_reached_since_us_ = 0;
}

void tanh_ctrl_node::handleMissionPreconditions()
{
  if (!is_offboard_ && mission_state_ != MissionState::WAIT_FOR_OFFBOARD) {
    resetMissionProgress();
    updateHoldReference(state_.position_ned.z());
    setMissionState(MissionState::WAIT_FOR_OFFBOARD, "offboard lost");
  }

  if (!is_armed_ &&
    mission_state_ != MissionState::WAIT_FOR_OFFBOARD &&
    mission_state_ != MissionState::WAIT_FOR_ARMING)
  {
    resetMissionProgress();
    updateHoldReference(state_.position_ned.z());
    setMissionState(MissionState::WAIT_FOR_ARMING, "vehicle disarmed");
  }
}

void tanh_ctrl_node::maybeSendAutomaticRequests(uint64_t now_us)
{
  if (has_state_ && offboard_counter_ < offboard_setpoint_warmup_) {
    ++offboard_counter_;
  }

  const bool warmup_done = offboard_counter_ >= offboard_setpoint_warmup_;
  if (!has_state_ || !warmup_done) {
    return;
  }

  if (enable_auto_offboard_ && !is_offboard_) {
    if (elapsedSeconds(now_us, last_offboard_request_us_) >= mission_request_interval_s_ ||
      last_offboard_request_us_ == 0)
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0f,
        6.0f,
        0.0f);
      last_offboard_request_us_ = now_us;
    }
  }

  if (enable_auto_arm_ && is_offboard_ && !is_armed_) {
    if (elapsedSeconds(now_us, last_arm_request_us_) >= mission_request_interval_s_ ||
      last_arm_request_us_ == 0)
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0f,
        0.0f,
        0.0f);
      last_arm_request_us_ = now_us;
    }
  }
}

void tanh_ctrl_node::updateMissionStateMachine(uint64_t now_us)
{
  switch (mission_state_) {
    case MissionState::WAIT_FOR_OFFBOARD:
      updateHoldReference(state_.position_ned.z());
      if (is_offboard_) {
        last_offboard_request_us_ = 0;
        setMissionState(MissionState::WAIT_FOR_ARMING, "offboard enabled");
      }
      break;

    case MissionState::WAIT_FOR_ARMING:
      updateHoldReference(state_.position_ned.z());
      if (!is_offboard_) {
        setMissionState(MissionState::WAIT_FOR_OFFBOARD, "waiting for offboard");
      } else if (is_armed_) {
        last_arm_request_us_ = 0;
        updateHoldReference(mission_takeoff_target_z_);
        takeoff_reached_ = false;
        takeoff_reached_since_us_ = 0;
        setMissionState(MissionState::TAKEOFF, "armed");
      }
      break;

    case MissionState::TAKEOFF: {
      updateHoldReference(mission_takeoff_target_z_);
      const double altitude_error =
        std::abs(state_.position_ned.z() - mission_takeoff_target_z_);
      if (altitude_error > mission_takeoff_z_threshold_) {
        takeoff_reached_ = false;
        takeoff_reached_since_us_ = 0;
        break;
      }

      if (!takeoff_reached_) {
        takeoff_reached_ = true;
        takeoff_reached_since_us_ = now_us;
      }

      if (elapsedSeconds(now_us, takeoff_reached_since_us_) >= mission_takeoff_hold_time_s_) {
        if (!start_tracking_sent_) {
          publishStartTrackingSignal(true);
          start_tracking_sent_ = true;
        }
        setMissionState(MissionState::HOLD, "takeoff complete");
      }
      break;
    }

    case MissionState::HOLD:
      if (hasFreshExternalReference(now_us)) {
        setMissionState(MissionState::TRACKING, "trajectory received");
      }
      break;

    case MissionState::TRACKING:
      if (!hasFreshExternalReference(now_us)) {
        updateHoldReference(state_.position_ned.z());
        setMissionState(MissionState::HOLD, "trajectory timeout");
      }
      break;
  }
}

const TrajectoryRef * tanh_ctrl_node::selectActiveReference(uint64_t now_us) const
{
  if (mission_state_ == MissionState::TRACKING && hasFreshExternalReference(now_us)) {
    return &external_ref_;
  }

  if (has_hold_ref_ && hold_ref_.valid) {
    return &hold_ref_;
  }

  return nullptr;
}

void tanh_ctrl_node::publishMotorCommands(const ControlOutput & out, uint64_t now_us)
{
  if (!motors_pub_) {
    return;
  }

  px4_msgs::msg::ActuatorMotors motors{};
  motors.timestamp = now_us;
  motors.timestamp_sample = now_us;
  motors.reversible_flags = 0;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  motors.control.fill(nan);
  for (int output_index = 0; output_index < 4; ++output_index) {
    const int internal_index = motor_output_map_[output_index];
    const double control = out.motor_controls(internal_index);
    motors.control[output_index] = static_cast<float>(
      std::isfinite(control) ? std::clamp(control, 0.0, 1.0) : 0.0);
  }

  motors_pub_->publish(motors);
}

void tanh_ctrl_node::publishThrustSetpoint(const ControlOutput & out, uint64_t now_us)
{
  if (!publish_vehicle_thrust_setpoint_ || !thrust_sp_pub_) {
    return;
  }

  px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
  thrust_sp.timestamp = now_us;
  thrust_sp.timestamp_sample = now_us;

  double throttle = 0.0;
  const double denominator = 4.0 * std::max(1e-6, motor_force_max_);
  if (std::isfinite(out.thrust_total)) {
    throttle = std::clamp(out.thrust_total / denominator, 0.0, 1.0);
  }

  thrust_sp.xyz = {0.0f, 0.0f, static_cast<float>(-throttle)};
  thrust_sp_pub_->publish(thrust_sp);
}

void tanh_ctrl_node::controlLoop()
{
  if (exit_requested_) {
    RCLCPP_ERROR(this->get_logger(), "Shutting down controller because offboard mode was exited.");
    rclcpp::shutdown();
    return;
  }

  const uint64_t now_us = nowMicros(*this->get_clock());
  const double dt = computeControlDt(now_us);

  publishOffboardControlMode(now_us);

  if (!has_state_) {
    return;
  }

  if (!has_hold_ref_) {
    updateHoldReference(state_.position_ned.z());
  }

  handleMissionPreconditions();
  maybeSendAutomaticRequests(now_us);
  updateMissionStateMachine(now_us);

  const TrajectoryRef * active_ref = selectActiveReference(now_us);
  if (!active_ref || !active_ref->valid) {
    return;
  }

  ControlOutput out;
  if (!controller_.compute(state_, *active_ref, dt, &out)) {
    return;
  }

  publishMotorCommands(out, now_us);
  publishThrustSetpoint(out, now_us);
}

}  // namespace tanh_ctrl
