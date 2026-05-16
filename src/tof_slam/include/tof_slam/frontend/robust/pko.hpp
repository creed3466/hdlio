// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// pko.hpp — Probabilistic Kernel Optimizer for adaptive robust estimation.
//
// Automatically tunes the Huber loss delta by minimizing Jensen-Shannon
// divergence between a GMM-fitted residual distribution and the Huber
// kernel-induced distribution.
//
// Reference: Choi & Kim, "Probabilistic Kernel Optimization for Robust
// State Estimation," IEEE RA-L vol.10 no.3, March 2025.

#ifndef TOF_SLAM_FRONTEND_ROBUST_PKO_HPP_
#define TOF_SLAM_FRONTEND_ROBUST_PKO_HPP_

#include <vector>

namespace tof_slam {
namespace core {

/// Configuration for the Probabilistic Kernel Optimizer.
struct PkoConfig {
  bool use_adaptive = true;
  double min_scale_factor = 0.01;
  double max_scale_factor = 10.0;
  int num_alpha_segments = 30;      // was 100: 30 log-spaced points sufficient
  double truncated_threshold = 10.0;
  int gmm_components = 3;
  int gmm_sample_size = 300;        // was 1000: subsample more aggressively
  int kmeans_max_iters = 20;        // was 100: K-means converges fast for K=3
  int em_max_iters = 20;            // was 100: EM converges fast for 1D GMM
  int jsd_segments = 30;            // was 100: JSD numerical integration segments
};

/// Adaptive Huber loss scale estimator via JSD minimization.
class Pko {
 public:
  explicit Pko(const PkoConfig& config = PkoConfig());
  ~Pko() = default;

  /// Compute optimal Huber delta from a vector of residuals.
  double compute_scale_factor(const std::vector<double>& residuals);

  /// Reset internal state (graduated non-convexity reference, GMM).
  void reset();

  const PkoConfig& config() const { return config_; }

 private:
  void initialize();

  void fit_gmm(const std::vector<double>& residuals);

  double gaussian_pdf(double x, double mean, double variance) const;

  double partition_function(double alpha) const;

  double huber_weight(double residual, double delta) const;

  double js_divergence(double alpha);

  PkoConfig config_;

  std::vector<double> alpha_candidates_;
  std::vector<double> partition_functions_;
  double alpha_star_ref_;
  bool initialized_ = false;

  std::vector<double> gmm_weights_;
  std::vector<double> gmm_means_;
  std::vector<double> gmm_variances_;
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ROBUST_PKO_HPP_
