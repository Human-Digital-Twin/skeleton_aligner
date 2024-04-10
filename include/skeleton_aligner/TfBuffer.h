#ifndef hiros_skeleton_aligner_TfBuffer_h
#define hiros_skeleton_aligner_TfBuffer_h

// Standard dependencies
#include <chrono>
#include <deque>

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

namespace hiros {
namespace hdt {

class TfBuffer {
 public:
  TfBuffer(
      const double& t_weight,
      const double& t_weight_threshold = k_default_weight_threshold,
      const double& t_time_threshold = k_default_time_threshold,
      const double& t_clustering_threshold = k_default_clustering_threshold);

  size_t size() const {
    return clusters_.empty() ? 0. : clusters_.front().size();
  }
  bool empty() const { return clusters_.empty() || clusters_.front().empty(); }
  void push_back(const tf2::Transform& t_tf);

  tf2::Transform avg() const { return avg_; }

 private:
  constexpr static const double k_default_weight_threshold{1e-3};
  constexpr static const double k_default_time_threshold{600.};  // [s]
  constexpr static const double k_default_clustering_threshold{.5};

  struct StampedTransform {
    std::chrono::time_point<std::chrono::system_clock> time{};
    tf2::Transform transform{};
  };

  typedef std::deque<StampedTransform> StampedTransformDeque;

  void updateClusters(const StampedTransform& t_stf);
  void updateAvg();

  void cleanupClusters();
  void weightBasedCleanup();
  void timeBasedCleanup();
  void sortClusters();

  tf2::Transform computeAvg(const StampedTransformDeque& buffer) const;

  double weight_{};
  double weight_threshold_{};
  double time_threshold_{};
  double clustering_threshold_{};
  size_t max_cluster_size_{};

  tf2::Transform avg_{};
  std::vector<StampedTransformDeque> clusters_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
