// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_threshold)
    : weight_{t_weight}, threshold_{t_threshold} {
  max_size_ = static_cast<size_t>(
      std::round(2. * std::log(threshold_) / std::log(weight_)));
}

void hiros::hdt::TfBuffer::push_back(const tf2::Transform& t_tf) {
  buffer_.push_back(t_tf);
  if (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

tf2::Transform hiros::hdt::TfBuffer::avg() const {
  std::vector<tf2::Transform> identities{buffer_.size()};
  tf2::Transform identity{};
  identity.setIdentity();
  std::fill(identities.begin(), identities.end(), identity);
  std::vector<tf2::Transform> tfs{buffer_.cbegin(), buffer_.cend()};

  return utils::solveWeightedLeastSquares(identities, tfs, weight_);
}
