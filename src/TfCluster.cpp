// Standard dependencies
#include <algorithm>

// Internal dependencies
#include "skeleton_aligner/TfCluster.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfCluster::TfCluster(const double& t_weight) : weight_{t_weight} {}

hiros::hdt::TfCluster::TfCluster(const tf2::Transform& t_tf,
                                 const double& t_weight)
    : TfCluster(t_weight) {
  push_back(t_tf);
}

void hiros::hdt::TfCluster::push_back(const tf2::Transform& t_tf) {
  ++cluster_size_;
  last_tf_ = {std::chrono::system_clock::now(), t_tf};
  computeAvg();

  cumulative_weight_ += std::pow(weight_, cluster_size_);
}

void hiros::hdt::TfCluster::computeAvg() {
  if (empty()) {
    return;
  }

  if (cluster_size_ == 1) {
    avg_tf_ = last_tf_.transform;
    return;
  }

  avg_tf_ =
      utils::weightedAverage({avg_tf_, last_tf_.transform}, cumulative_weight_);
}
