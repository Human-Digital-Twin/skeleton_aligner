#ifndef hiros_skeleton_aligner_TfCluster_h
#define hiros_skeleton_aligner_TfCluster_h

// ROS dependencies
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

namespace hiros {
namespace hdt {

class TfCluster {
 public:
  TfCluster(const double& t_weight = 1.);
  TfCluster(const tf2::Transform& t_tf, const double& t_weight = 1.);

  bool empty() const { return cluster_size_ == 0; }
  size_t size() const { return cluster_size_; }
  void push_back(const tf2::Transform& t_tf);

  double age() const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               std::chrono::system_clock::now() - last_tf_.stamp_)
        .count();
  }
  tf2::Transform avg() const { return avg_tf_; }

 private:
  void computeAvg();

  double weight_{};
  double cumulative_weight_{0.};

  size_t cluster_size_{0};
  tf2::Stamped<tf2::Transform> last_tf_{};
  tf2::Transform avg_tf_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
