#include "tof_slam/backend/pgo_convergence_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace tof_slam {

ceres::CallbackReturnType PgoConvergenceMonitor::operator()(
    const ceres::IterationSummary& summary) {
  PgoIterationRecord rec;
  rec.iteration           = summary.iteration;
  rec.cost                = summary.cost;
  rec.cost_change         = summary.cost_change;
  rec.gradient_max_norm   = summary.gradient_max_norm;
  rec.step_norm           = summary.step_norm;
  rec.relative_decrease   = summary.relative_decrease;
  rec.step_is_valid       = summary.step_is_valid;
  rec.step_is_successful  = summary.step_is_successful;
  records_.push_back(rec);
  return ceres::SOLVER_CONTINUE;
}

void PgoConvergenceMonitor::openCsv(const std::string& path) {
  csv_.open(path);
  if (!csv_.is_open()) return;
  csv_ << "opt_id,iteration,cost,cost_change,gradient_max_norm,"
          "step_norm,relative_decrease,step_is_valid,step_is_successful\n";
  csv_.flush();
  csv_open_ = true;
}

void PgoConvergenceMonitor::writeCsv(int opt_id) {
  if (csv_open_) {
    for (const auto& r : records_) {
      csv_ << opt_id << ","
           << r.iteration << ","
           << r.cost << ","
           << r.cost_change << ","
           << r.gradient_max_norm << ","
           << r.step_norm << ","
           << r.relative_decrease << ","
           << (r.step_is_valid ? 1 : 0) << ","
           << (r.step_is_successful ? 1 : 0) << "\n";
    }
    csv_.flush();
  }
  clear();
}

void PgoConvergenceMonitor::closeCsv() {
  if (csv_open_) {
    csv_.close();
    csv_open_ = false;
  }
}

}  // namespace tof_slam
