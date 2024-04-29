// Internal dependencies
#include "skeleton_aligner/TfCluster.h"

#include "skeleton_aligner/utils.h"

hiros::hdt::TfCluster::TfCluster(const double& weight) : weight_{weight} {}

hiros::hdt::TfCluster::TfCluster(const tf2::Transform& tf, const double& weight)
    : TfCluster(weight) {
  push_back(tf);
}

void hiros::hdt::TfCluster::push_back(const tf2::Transform& tf) {
  ++cluster_size_;
  last_tf_.stamp_ = std::chrono::system_clock::now();
  last_tf_.setData(tf);
  computeAvg();

  cumulative_weight_ += std::pow(weight_, cluster_size_);
}

void hiros::hdt::TfCluster::computeAvg() {
  if (empty()) {
    return;
  }

  if (cluster_size_ == 1) {
    avg_tf_ = last_tf_;
    return;
  }

  avg_tf_ = utils::weightedAverage({avg_tf_, last_tf_}, cumulative_weight_);
}
