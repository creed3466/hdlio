#pragma once
#include <ceres/ceres.h>
#include <fstream>
#include <string>
#include <vector>

namespace tof_slam {

struct PgoIterationRecord {
  int iteration;
  double cost;
  double cost_change;
  double gradient_max_norm;
  double step_norm;
  double relative_decrease;
  bool step_is_valid;
  bool step_is_successful;
};

class PgoConvergenceMonitor : public ceres::IterationCallback {
public:
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override;
  const std::vector<PgoIterationRecord>& records() const { return records_; }
  void clear() { records_.clear(); }

  // CSV output
  void openCsv(const std::string& path);
  void writeCsv(int opt_id);  // writes current records_ then clears
  void closeCsv();

private:
  std::vector<PgoIterationRecord> records_;
  std::ofstream csv_;
  bool csv_open_{false};
};

}  // namespace tof_slam
