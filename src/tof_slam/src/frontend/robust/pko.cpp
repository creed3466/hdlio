// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/robust/pko.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace tof_slam {
namespace core {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

Pko::Pko(const PkoConfig& config)
    : config_(config), alpha_star_ref_(config.max_scale_factor) {}

void Pko::reset() {
  alpha_star_ref_ = config_.max_scale_factor;
  gmm_weights_.clear();
  gmm_means_.clear();
  gmm_variances_.clear();
}

double Pko::compute_scale_factor(const std::vector<double>& residuals) {
  if (residuals.empty()) return 1.0;
  if (!config_.use_adaptive) return config_.min_scale_factor;

  if (!initialized_) {
    initialize();
    initialized_ = true;
  }

  fit_gmm(residuals);

  double best_alpha = config_.min_scale_factor;
  double best_cost = std::numeric_limits<double>::max();

  for (size_t i = 1; i < alpha_candidates_.size(); ++i) {
    const double alpha = alpha_candidates_[i];
    if (alpha > alpha_star_ref_) continue;

    const double jsd = js_divergence(alpha);
    if (jsd < best_cost) {
      best_cost = jsd;
      best_alpha = alpha;
    }
  }

  alpha_star_ref_ = best_alpha;
  return best_alpha;
}

void Pko::initialize() {
  const int n = config_.num_alpha_segments + 1;
  alpha_candidates_.resize(n);
  partition_functions_.resize(n);

  alpha_candidates_[0] = config_.min_scale_factor;
  partition_functions_[0] = partition_function(config_.min_scale_factor);

  for (int i = 1; i < n; ++i) {
    const double t = static_cast<double>(i) / config_.num_alpha_segments;
    const double log_scaled = (std::pow(100.0, t) - 1.0) / 99.0;
    const double alpha =
        config_.min_scale_factor +
        (config_.max_scale_factor - config_.min_scale_factor) * log_scaled;

    alpha_candidates_[i] = alpha;
    partition_functions_[i] = partition_function(alpha);
  }
}

void Pko::fit_gmm(const std::vector<double>& residuals) {
  if (residuals.empty()) return;

  const int n = static_cast<int>(residuals.size());
  const int sample_size = std::min(config_.gmm_sample_size, n);
  const int K = config_.gmm_components;

  // Random sub-sample.
  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);
  std::mt19937 gen(42);
  std::shuffle(indices.begin(), indices.end(), gen);

  std::vector<double> data(sample_size);
  for (int i = 0; i < sample_size; ++i) data[i] = residuals[indices[i]];

  // K-means initialization.
  gmm_means_.resize(K);
  gmm_means_[0] = 0.0;
  std::uniform_int_distribution<> dis(0, sample_size - 1);
  for (int k = 1; k < K; ++k) gmm_means_[k] = data[dis(gen)];

  std::vector<int> clusters(sample_size);
  for (int iter = 0; iter < config_.kmeans_max_iters; ++iter) {
    for (int i = 0; i < sample_size; ++i) {
      double min_d = std::numeric_limits<double>::max();
      int best = 0;
      for (int k = 0; k < K; ++k) {
        const double d = std::abs(data[i] - gmm_means_[k]);
        if (d < min_d) { min_d = d; best = k; }
      }
      clusters[i] = best;
    }

    std::vector<double> new_means(K, 0.0);
    std::vector<int> counts(K, 0);
    for (int i = 0; i < sample_size; ++i) {
      new_means[clusters[i]] += data[i];
      counts[clusters[i]]++;
    }
    new_means[0] = 0.0;
    for (int k = 1; k < K; ++k) {
      if (counts[k] > 0) new_means[k] /= counts[k];
    }
    if (new_means == gmm_means_) break;
    gmm_means_ = new_means;
  }

  // Initial variance from global data.
  const double data_mean =
      std::accumulate(data.begin(), data.end(), 0.0) / sample_size;
  double var = 0.0;
  for (double x : data) var += (x - data_mean) * (x - data_mean);
  var /= sample_size;
  gmm_variances_.assign(K, var);

  // Weights from cluster sizes.
  std::vector<int> cc(K, 0);
  for (int c : clusters) cc[c]++;
  gmm_weights_.resize(K);
  for (int k = 0; k < K; ++k)
    gmm_weights_[k] = static_cast<double>(cc[k]) / sample_size;

  // EM iterations.
  std::vector<std::vector<double>> resp(sample_size, std::vector<double>(K));
  for (int iter = 0; iter < config_.em_max_iters; ++iter) {
    // E-step.
    for (int i = 0; i < sample_size; ++i) {
      double sum_r = 0.0;
      for (int k = 0; k < K; ++k) {
        resp[i][k] = gmm_weights_[k] *
                     gaussian_pdf(data[i], gmm_means_[k], gmm_variances_[k]);
        sum_r += resp[i][k];
      }
      for (int k = 0; k < K; ++k) resp[i][k] /= (sum_r + 1e-10);
    }

    // M-step.
    std::vector<double> Nk(K, 0.0);
    for (int k = 0; k < K; ++k)
      for (int i = 0; i < sample_size; ++i) Nk[k] += resp[i][k];

    std::vector<double> nw(K), nm(K, 0.0), nv(K, 0.0);
    for (int k = 0; k < K; ++k) {
      nw[k] = Nk[k] / sample_size;
      if (k > 0) {
        for (int i = 0; i < sample_size; ++i)
          nm[k] += resp[i][k] * data[i];
        nm[k] /= (Nk[k] + 1e-10);
      }
      for (int i = 0; i < sample_size; ++i) {
        const double d = data[i] - nm[k];
        nv[k] += resp[i][k] * d * d;
      }
      nv[k] = std::max(nv[k] / (Nk[k] + 1e-10), 1e-6);
    }

    double change = 0.0;
    for (int k = 1; k < K; ++k)
      change += std::abs(nm[k] - gmm_means_[k]);

    gmm_weights_ = nw;
    gmm_means_ = nm;
    gmm_variances_ = nv;

    if (change < 1e-6) break;
  }
}

double Pko::gaussian_pdf(double x, double mean, double variance) const {
  if (variance <= 0.0) return 0.0;
  const double d = x - mean;
  return std::exp(-0.5 * d * d / variance) / std::sqrt(2.0 * kPi * variance);
}

double Pko::partition_function(double alpha) const {
  constexpr double step = 0.01;
  double integral = 0.0;
  for (double x = 0.0; x <= config_.truncated_threshold; x += step)
    integral += huber_weight(x, alpha) * step;
  return std::max(integral, 1e-10);
}

double Pko::huber_weight(double residual, double delta) const {
  const double a = std::abs(residual);
  return (a <= delta) ? 1.0 : delta / a;
}

double Pko::js_divergence(double alpha) {
  const int kSegments = config_.jsd_segments;
  const double dr = config_.truncated_threshold / kSegments;

  // Find cached partition function.
  double Z = 0.0;
  for (size_t j = 0; j < alpha_candidates_.size(); ++j) {
    if (std::abs(alpha_candidates_[j] - alpha) < 1e-10) {
      Z = partition_functions_[j];
      break;
    }
  }
  if (Z < 1e-10) Z = partition_function(alpha);

  double cost = 0.0;
  double cnt = 0.0;
  const int K = static_cast<int>(gmm_weights_.size());

  for (int i = 0; i < kSegments; ++i) {
    const double r = dr * (1 + i);
    double Pr = 0.0;
    for (int k = 0; k < K; ++k)
      Pr += gmm_weights_[k] * gaussian_pdf(r, gmm_means_[k], gmm_variances_[k]);
    Pr += 1e-10;

    const double Q = huber_weight(r, alpha) / (Z + 1e-10) + 1e-10;
    const double M = 0.5 * (Pr + Q);
    const double jsd = 0.5 * (Pr * std::log(Pr / M) + Q * std::log(Q / M));
    if (!std::isnan(jsd)) { cost += jsd; cnt += 1.0; }
  }

  return (cnt > 0.0) ? cost / cnt : std::numeric_limits<double>::max();
}

}  // namespace core
}  // namespace tof_slam
