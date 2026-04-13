#include "tanh_ctrl/tanh_blocks.hpp"

#include <cmath>

namespace tanh_ctrl
{

namespace
{

double sanitize_cutoff(double cutoff_hz)
{
  return (std::isfinite(cutoff_hz) && cutoff_hz > 0.0) ? cutoff_hz : 0.0;
}

double update_scalar_low_pass(
  double input,
  double cutoff_hz,
  double dt,
  double * state,
  bool * initialized)
{
  if (!state || !initialized) {
    return input;
  }

  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return input;
  }

  cutoff_hz = sanitize_cutoff(cutoff_hz);
  if (!(cutoff_hz > 0.0)) {
    return input;
  }

  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  const double alpha = dt / (tau + dt);

  if (!(*initialized)) {
    *state = input;
    *initialized = true;
    return input;
  }

  *state += alpha * (input - *state);
  return *state;
}

}  // namespace

Eigen::Vector3d tanh_feedback(
  const Eigen::Vector3d & error,
  const Eigen::Vector3d & slope,
  const Eigen::Vector3d & scale)
{
  return scale.cwiseProduct((slope.cwiseProduct(error)).array().tanh().matrix());
}

void reset_low_pass(Vec3LowPass & lpf)
{
  lpf.state.setZero();
  lpf.initialized = {{false, false, false}};
}

Eigen::Vector3d update_low_pass(
  const Eigen::Vector3d & input,
  double dt,
  Vec3LowPass & lpf)
{
  Eigen::Vector3d output;
  output.x() = update_scalar_low_pass(
    input.x(), lpf.cutoff_hz.x(), dt, &lpf.state.x(), &lpf.initialized[0]);
  output.y() = update_scalar_low_pass(
    input.y(), lpf.cutoff_hz.y(), dt, &lpf.state.y(), &lpf.initialized[1]);
  output.z() = update_scalar_low_pass(
    input.z(), lpf.cutoff_hz.z(), dt, &lpf.state.z(), &lpf.initialized[2]);
  return output;
}

void reset_rate_estimator(Vec3RateEstimator & estimator)
{
  estimator.last_value.setZero();
  estimator.has_last_value = false;
  reset_low_pass(estimator.filter);
}

Eigen::Vector3d update_rate_estimator(
  const Eigen::Vector3d & value,
  double dt,
  Vec3RateEstimator & estimator)
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return Eigen::Vector3d::Zero();
  }

  if (!estimator.has_last_value) {
    estimator.last_value = value;
    estimator.has_last_value = true;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d rate = (value - estimator.last_value) / dt;
  estimator.last_value = value;

  if (!rate.allFinite()) {
    rate.setZero();
  }

  return update_low_pass(rate, dt, estimator.filter);
}

}  // namespace tanh_ctrl
