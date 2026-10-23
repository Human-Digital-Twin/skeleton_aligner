// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeleton_aligner/TfCluster.h"
#include "skeleton_aligner/utils.h"

namespace hdt::skeleton {

TfCluster::TfCluster(const double& weight) : weight_{weight} {}

TfCluster::TfCluster(const tf2::Transform& tf, const double& weight)
    : TfCluster(weight) {
  push_back(tf);
}

void TfCluster::push_back(const tf2::Transform& tf) {
  ++cluster_size_;
  last_tf_.stamp_ = std::chrono::system_clock::now();
  last_tf_.setData(tf);
  computeAvg();

  cumulative_weight_ += std::pow(weight_, cluster_size_);
}

void TfCluster::computeAvg() {
  if (empty()) {
    return;
  }

  if (cluster_size_ == 1) {
    avg_tf_ = last_tf_;
    return;
  }

  avg_tf_ = utils::weightedAverage({avg_tf_, last_tf_}, cumulative_weight_);
}

}  // namespace hdt::skeleton
