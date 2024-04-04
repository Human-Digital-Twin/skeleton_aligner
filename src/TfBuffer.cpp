// Standard dependencies
#include <cmath>

// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_threshold)
    : weight_{t_weight}, threshold_{t_threshold} {
  if (weight_ > 0. && weight_ < 1.) {
    max_size_ = static_cast<size_t>(
        std::round(2. * std::log(threshold_) / std::log(weight_)));
  } else {
    max_size_ = std::numeric_limits<size_t>::max();
    weight_ = 1.;
  }
}

void hiros::hdt::TfBuffer::push_back(const tf2::Transform& t_tf) {
  if (hiros::skeletons::utils::isNaN(t_tf.getOrigin()) ||
      hiros::skeletons::utils::isNaN(t_tf.getRotation())) {
    return;
  }

  buffer_.push_back(t_tf);
  if (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

tf2::Transform hiros::hdt::TfBuffer::avg() const {
  if (buffer_.empty()) {
    return {};
  }

  std::vector<tf2::Transform> identities{buffer_.size()};
  tf2::Transform identity{};
  identity.setIdentity();
  std::fill(identities.begin(), identities.end(), identity);
  std::vector<tf2::Transform> tfs{buffer_.cbegin(), buffer_.cend()};

  return utils::solveWeightedLeastSquares(identities, tfs, weight_);
}
