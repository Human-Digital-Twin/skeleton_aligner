#ifndef hiros_skeleton_aligner_TfBuffer_h
#define hiros_skeleton_aligner_TfBuffer_h

// Standard dependencies
#include <deque>

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

namespace hiros {
namespace hdt {

const double k_default_threshold{1e-3};

class TfBuffer {
 public:
  TfBuffer(const double& t_weight,
           const double& t_threshold = k_default_threshold);

 private:
  void push_back(const tf2::Transform& t_tf);

  double weight_{};
  double threshold_{};
  unsigned int max_size_{};
  std::deque<tf2::Transform> buffer_{};
};

}  // namespace hdt
}  // namespace hiros

#endif