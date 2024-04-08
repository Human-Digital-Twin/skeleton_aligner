// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_weight_threshold,
                               const double& t_time_threshold)
    : weight_{t_weight},
      weight_threshold_{t_weight_threshold},
      time_threshold_{t_time_threshold} {
  if (time_threshold_ <= 0.) {
    time_threshold_ = std::numeric_limits<double>::max();
  }

  if (weight_threshold_ <= 0.) {
    weight_threshold_ = k_default_weight_threshold;
  }

  if (weight_ <= 0. || weight_ >= 1.) {
    weight_ = 1.;
    max_size_ = std::numeric_limits<size_t>::max();
  } else {
    max_size_ = static_cast<size_t>(
        std::round(2. * std::log(weight_threshold_) / std::log(weight_)));
  }
}

void hiros::hdt::TfBuffer::push_back(const tf2::Transform& t_tf) {
  if (hiros::skeletons::utils::isNaN(t_tf.getOrigin()) ||
      hiros::skeletons::utils::isNaN(t_tf.getRotation())) {
    return;
  }

  auto now{std::chrono::system_clock::now()};
  buffer_.push_back({std::chrono::system_clock::now(), t_tf});

  // Weight-based clean up
  if (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }

  // Time-based clean up
  while (buffer_.size() > max_size_ / 2. &&
         std::chrono::duration<double>(now - buffer_.front().time).count() >
             time_threshold_) {
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

  std::vector<tf2::Transform> tfs{};
  tfs.reserve(buffer_.size());
  for (const auto& stamped_tf : buffer_) {
    tfs.push_back(stamped_tf.transform);
  }

  return utils::solveWeightedLeastSquares(identities, tfs, weight_);
}
