#ifndef skeleton_aligner_TfBuffer_h
#define skeleton_aligner_TfBuffer_h

// Standard dependencies
#include <cstddef>
#include <vector>

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Internal dependencies
#include "skeleton_aligner/TfCluster.h"

namespace hdt::skeleton {

class TfBuffer {
 public:
  explicit TfBuffer(
      const double& weight,
      const double& time_threshold = k_default_time_threshold,
      const double& clustering_threshold = k_default_clustering_threshold);

  size_t n_clusters() const { return clusters_.size(); }
  size_t size() const {
    return clusters_.empty() ? 0 : clusters_.front().size();
  }
  bool empty() const { return clusters_.empty() || clusters_.front().empty(); }
  void push_back(const tf2::Transform& tf);

  tf2::Transform avg() const {
    return clusters_.empty() ? tf2::Transform{} : clusters_.front().avg();
  }

 private:
  constexpr static const double k_default_time_threshold{10.};  // [s]
  constexpr static const double k_default_clustering_threshold{.5};

  void updateClusters(const tf2::Transform& tf);

  void timeBasedCleanup();
  void sortClusters();
  void distanceBasedCleanup();

  double weight_{};
  double time_threshold_{};
  double clustering_threshold_{};

  std::vector<TfCluster> clusters_{};
};

}  // namespace hdt::skeleton

#endif
