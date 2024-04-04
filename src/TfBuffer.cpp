// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_threshold)
    : weight_{t_weight}, threshold_{t_threshold} {
  max_size_ =
      static_cast<unsigned int>(std::round(2 * log(threshold_) / log(weight_)));
}

void hiros::hdt::TfBuffer::push_back(const tf2::Transform& t_tf) {
  buffer_.push_back(t_tf);
  if (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}