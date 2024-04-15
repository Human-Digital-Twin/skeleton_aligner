#ifndef hiros_skeleton_aligner_TfCluster_h
#define hiros_skeleton_aligner_TfCluster_h

// Standard dependencies
#include <chrono>
#include <deque>

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

namespace hiros {
namespace hdt {

class TfCluster {
 public:
  TfCluster(const double& t_weight);
  TfCluster(const double& t_weight, const tf2::Transform& t_tf);

  bool empty() const { return cluster_.empty(); }
  size_t size() const { return cluster_.size(); }

  void push_back(const tf2::Transform& t_tf);
  void pop_back() { cluster_.pop_back(); }
  void pop_front() { cluster_.pop_front(); }
  void resize(const size_t& t_size) { cluster_.resize(t_size); }

  void merge(const TfCluster& t_other);

  std::chrono::seconds age() const;
  tf2::Transform avg() const { return average_; }

 private:
  struct StampedTransform {
    std::chrono::time_point<std::chrono::system_clock> time{};
    tf2::Transform transform{};
  };

  void computeFastAvg();
  void computeAvg();

  double weight_{1.};

  std::deque<StampedTransform> cluster_{};
  tf2::Transform average_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
