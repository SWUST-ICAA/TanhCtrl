#include "tanh_ctrl/tanh_blocks.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

namespace {

constexpr double kTol = 1e-6;

bool expectNear(
  const Eigen::Vector3d & actual, const Eigen::Vector3d & expected, const char * label)
{
  if ((actual - expected).cwiseAbs().maxCoeff() <= kTol) {
    return true;
  }

  std::cerr << label << " mismatch\n"
            << "  actual:   " << actual.transpose() << "\n"
            << "  expected: " << expected.transpose() << "\n";
  return false;
}

bool tanhFeedbackMatchesElementwiseDefinition()
{
  const Eigen::Vector3d error(0.5, -0.25, 1.0);
  const Eigen::Vector3d slope(2.0, 4.0, 1.5);
  const Eigen::Vector3d scale(1.0, 0.5, 2.0);

  const Eigen::Vector3d output = tanh_ctrl::tanh_feedback(error, slope, scale);
  const Eigen::Vector3d expected(
    1.0 * std::tanh(2.0 * 0.5),
    0.5 * std::tanh(4.0 * -0.25),
    2.0 * std::tanh(1.5 * 1.0));
  return expectNear(output, expected, "tanh_feedback");
}

bool lowPassPassesThroughWhenDisabled()
{
  tanh_ctrl::Vec3LowPass lpf;
  lpf.cutoff_hz = Eigen::Vector3d::Zero();

  const Eigen::Vector3d input(1.0, -2.0, 3.0);
  const Eigen::Vector3d output = tanh_ctrl::update_low_pass(input, 0.01, lpf);

  const bool output_ok = expectNear(output, input, "low-pass output when disabled");
  const bool state_ok = expectNear(lpf.state, Eigen::Vector3d::Zero(), "low-pass state when disabled");
  const bool init_ok = !lpf.initialized[0] && !lpf.initialized[1] && !lpf.initialized[2];
  if (!init_ok) {
    std::cerr << "low-pass initialized flags should remain false when disabled\n";
  }
  return output_ok && state_ok && init_ok;
}

bool lowPassInitializesStateOnFirstFilteredUpdate()
{
  tanh_ctrl::Vec3LowPass lpf;
  lpf.cutoff_hz = Eigen::Vector3d::Constant(10.0);

  const Eigen::Vector3d input(1.0, -2.0, 3.0);
  const Eigen::Vector3d output = tanh_ctrl::update_low_pass(input, 0.01, lpf);

  const bool output_ok = expectNear(output, input, "low-pass first output");
  const bool state_ok = expectNear(lpf.state, input, "low-pass first state");
  const bool init_ok = lpf.initialized[0] && lpf.initialized[1] && lpf.initialized[2];
  if (!init_ok) {
    std::cerr << "low-pass initialized flags should all become true after first filtered update\n";
  }
  return output_ok && state_ok && init_ok;
}

bool lowPassFiltersAfterInitialization()
{
  tanh_ctrl::Vec3LowPass lpf;
  lpf.cutoff_hz = Eigen::Vector3d::Constant(10.0);

  const Eigen::Vector3d first = tanh_ctrl::update_low_pass(Eigen::Vector3d::Zero(), 0.01, lpf);
  const Eigen::Vector3d second = tanh_ctrl::update_low_pass(Eigen::Vector3d::Ones(), 0.01, lpf);

  const double tau = 1.0 / (2.0 * M_PI * 10.0);
  const double alpha = 0.01 / (tau + 0.01);
  const Eigen::Vector3d expected = Eigen::Vector3d::Constant(alpha);

  return expectNear(first, Eigen::Vector3d::Zero(), "low-pass initial filtered output") &&
         expectNear(second, expected, "low-pass filtered output");
}

bool resetLowPassClearsStateButKeepsCutoff()
{
  tanh_ctrl::Vec3LowPass lpf;
  lpf.cutoff_hz = Eigen::Vector3d(5.0, 6.0, 7.0);
  lpf.state = Eigen::Vector3d(1.0, 2.0, 3.0);
  lpf.initialized = {{true, true, true}};

  tanh_ctrl::reset_low_pass(lpf);

  const bool state_ok = expectNear(lpf.state, Eigen::Vector3d::Zero(), "reset_low_pass state");
  const bool cutoff_ok = expectNear(lpf.cutoff_hz, Eigen::Vector3d(5.0, 6.0, 7.0), "reset_low_pass cutoff");
  const bool init_ok = !lpf.initialized[0] && !lpf.initialized[1] && !lpf.initialized[2];
  if (!init_ok) {
    std::cerr << "reset_low_pass should clear initialized flags\n";
  }
  return state_ok && cutoff_ok && init_ok;
}

bool rateEstimatorReturnsZeroUntilItHasHistory()
{
  tanh_ctrl::Vec3RateEstimator estimator;
  estimator.filter.cutoff_hz = Eigen::Vector3d::Zero();

  const Eigen::Vector3d first =
    tanh_ctrl::update_rate_estimator(Eigen::Vector3d(1.0, 2.0, 3.0), 0.01, estimator);
  const Eigen::Vector3d second =
    tanh_ctrl::update_rate_estimator(Eigen::Vector3d(1.2, 2.3, 3.4), 0.1, estimator);

  return expectNear(first, Eigen::Vector3d::Zero(), "rate estimator first output") &&
         expectNear(second, Eigen::Vector3d(2.0, 3.0, 4.0), "rate estimator second output");
}

bool rateEstimatorUsesLowPassFilterState()
{
  tanh_ctrl::Vec3RateEstimator estimator;
  estimator.filter.cutoff_hz = Eigen::Vector3d::Constant(10.0);

  (void)tanh_ctrl::update_rate_estimator(Eigen::Vector3d::Zero(), 0.01, estimator);
  const Eigen::Vector3d first =
    tanh_ctrl::update_rate_estimator(Eigen::Vector3d::Ones(), 0.1, estimator);
  const Eigen::Vector3d second =
    tanh_ctrl::update_rate_estimator(Eigen::Vector3d::Constant(2.0), 0.1, estimator);

  const bool first_ok = expectNear(first, Eigen::Vector3d::Constant(10.0), "rate estimator filtered first");
  const bool second_ok = expectNear(second, Eigen::Vector3d::Constant(10.0), "rate estimator filtered second");
  return first_ok && second_ok;
}

bool resetRateEstimatorClearsHistoryAndFilterState()
{
  tanh_ctrl::Vec3RateEstimator estimator;
  estimator.last_value = Eigen::Vector3d(1.0, 2.0, 3.0);
  estimator.has_last_value = true;
  estimator.filter.cutoff_hz = Eigen::Vector3d::Constant(8.0);
  estimator.filter.state = Eigen::Vector3d(4.0, 5.0, 6.0);
  estimator.filter.initialized = {{true, true, true}};

  tanh_ctrl::reset_rate_estimator(estimator);

  const bool last_ok = expectNear(estimator.last_value, Eigen::Vector3d::Zero(), "reset_rate_estimator last");
  const bool state_ok = expectNear(estimator.filter.state, Eigen::Vector3d::Zero(), "reset_rate_estimator filter state");
  const bool cutoff_ok =
    expectNear(estimator.filter.cutoff_hz, Eigen::Vector3d::Constant(8.0), "reset_rate_estimator cutoff");
  const bool history_ok = !estimator.has_last_value;
  const bool init_ok = !estimator.filter.initialized[0] &&
                       !estimator.filter.initialized[1] &&
                       !estimator.filter.initialized[2];
  if (!history_ok) {
    std::cerr << "reset_rate_estimator should clear history flag\n";
  }
  if (!init_ok) {
    std::cerr << "reset_rate_estimator should clear filter initialized flags\n";
  }
  return last_ok && state_ok && cutoff_ok && history_ok && init_ok;
}

}  // namespace

int main()
{
  bool ok = true;
  ok = tanhFeedbackMatchesElementwiseDefinition() && ok;
  ok = lowPassPassesThroughWhenDisabled() && ok;
  ok = lowPassInitializesStateOnFirstFilteredUpdate() && ok;
  ok = lowPassFiltersAfterInitialization() && ok;
  ok = resetLowPassClearsStateButKeepsCutoff() && ok;
  ok = rateEstimatorReturnsZeroUntilItHasHistory() && ok;
  ok = rateEstimatorUsesLowPassFilterState() && ok;
  ok = resetRateEstimatorClearsHistoryAndFilterState() && ok;
  return ok ? 0 : 1;
}
