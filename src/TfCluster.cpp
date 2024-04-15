// Standard dependencies
#include <algorithm>

// Internal dependencies
#include "skeleton_aligner/TfCluster.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfCluster::TfCluster(const double& t_weight) : weight_{t_weight} {}
hiros::hdt::TfCluster::TfCluster(const double& t_weight,
                                 const tf2::Transform& t_tf)
    : TfCluster(t_weight) {
  push_back(t_tf);
}

void hiros::hdt::TfCluster::push_back(const tf2::Transform& t_tf) {
  cluster_.push_back({std::chrono::system_clock::now(), t_tf});
  computeAvg();
}

void hiros::hdt::TfCluster::sort() {
  std::sort(
      cluster_.begin(), cluster_.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.time < rhs.time; });
}

void hiros::hdt::TfCluster::merge(const hiros::hdt::TfCluster& t_other) {
  cluster_.insert(cluster_.end(),
                  std::make_move_iterator(t_other.cluster_.begin()),
                  std::make_move_iterator(t_other.cluster_.end()));
}

std::chrono::seconds hiros::hdt::TfCluster::age() const {
  return std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now() - cluster_.back().time);
}

void hiros::hdt::TfCluster::computeAvg() {
  if (cluster_.empty()) {
    return;
  }

  average_ = utils::weightedAverage(cluster_.back().transform, average_,
                                    cluster_.size() - 1, weight_);
}
