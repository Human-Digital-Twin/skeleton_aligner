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
  TfBuffer(const double& t_weight,
           const double& t_weight_threshold = k_default_weight_threshold,
           const double& t_time_threshold = k_default_time_threshold);

  size_t size() const { return buffer_.size(); }
  bool empty() const { return buffer_.empty(); }
  void push_back(const tf2::Transform& t_tf);

  tf2::Transform avg() const;

 private:
  constexpr static const double k_default_weight_threshold{1e-3};
  constexpr static const double k_default_time_threshold{60.};  // [s]

  struct StampedTransform {
    std::chrono::time_point<std::chrono::system_clock> time{};
    tf2::Transform transform{};
  };

  typedef std::deque<StampedTransform> StampedTransformDeque;

  double weight_{};
  double weight_threshold_{};
  double time_threshold_{};
  size_t max_size_{};

  StampedTransformDeque buffer_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
