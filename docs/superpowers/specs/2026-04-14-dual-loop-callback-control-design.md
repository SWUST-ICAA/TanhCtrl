# Dual-Loop Callback Control Design

**Goal**

Replace the fixed-rate timer-driven controller with callback-driven dual-loop control for PX4:
- Position loop updates on `/fmu/out/vehicle_odometry`
- Attitude loop updates on `/fmu/out/vehicle_angular_velocity`
- Attitude state is updated from `/fmu/out/vehicle_attitude`
- `control_rate_hz` is removed because loop frequency is defined by PX4 state updates

**Current Problem**

`TanhNode` currently runs a single `controlLoop()` from a wall timer. PX4 state subscriptions only update cached state, so control can be computed before a fresh state sample arrives or multiple times from the same stale sample.

**Approved Design**

1. Keep the mission/reference state machine in the node.
2. Split controller execution into two stages:
   - Outer loop: position/velocity/acceleration feedback computes desired thrust direction and collective thrust.
   - Inner loop: attitude/body-rate feedback computes torque and motor commands from the cached outer-loop command.
3. Trigger outer loop only from `vehicle_odometry`.
4. Trigger inner loop only from `vehicle_angular_velocity`.
5. Update the body attitude cache only from `vehicle_attitude`.
6. Keep `SensorCombined` only as an auxiliary acceleration source for the outer loop.
7. Keep `OffboardControlMode` streaming from the inner loop callback so offboard messages continue at the fast inner-loop rate.

**Data Flow**

1. `vehicle_attitude` callback updates `q_body_to_ned` and yaw cache.
2. `vehicle_odometry` callback updates position/velocity, advances mission logic, selects active reference, and computes/caches the latest outer-loop `AttitudeReference`.
3. `vehicle_angular_velocity` callback updates `angular_velocity_body`, publishes offboard mode, sends automatic mode/arm requests when warmup is complete, runs the inner loop from the cached outer-loop command, and publishes actuator/thrust commands.

**Behavior Changes**

- No timer-based control loop remains.
- No `control_rate_hz` parameter remains.
- Position and attitude loops now use different `dt` values derived from their own callback timestamps.
- The node will not publish actuator commands until both the fast inner-loop state and the latest outer-loop command are available.

**Testing**

- Add a unit test that verifies split outer-loop plus inner-loop execution matches the legacy monolithic controller output for the same state/reference/dt.
- Add a unit test for timestamp-driven `dt` calculation helper behavior.
