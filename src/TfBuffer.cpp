// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_weight_threshold,
                               const double& t_time_threshold,
                               const double& t_clustering_threshold)
    : weight_{t_weight},
      weight_threshold_{t_weight_threshold},
      time_threshold_{t_time_threshold},
      clustering_threshold_{t_clustering_threshold} {
  if (clustering_threshold_ <= 0. || clustering_threshold_ >= 1.) {
    clustering_threshold_ = k_default_clustering_threshold;
  }

  if (time_threshold_ <= 0.) {
    time_threshold_ = std::numeric_limits<double>::max();
  }

  if (weight_threshold_ <= 0. || weight_threshold_ >= 1.) {
    weight_threshold_ = k_default_weight_threshold;
  }

  if (weight_ <= 0. || weight_ >= 1.) {
    weight_ = 1.;
    max_cluster_size_ = std::numeric_limits<size_t>::max();
  } else {
    max_cluster_size_ = static_cast<size_t>(
        std::round(2. * std::log(weight_threshold_) / std::log(weight_)));
  }
}

void hiros::hdt::TfBuffer::push_back(const tf2::Transform& t_tf) {
  if (hiros::skeletons::utils::isNaN(t_tf.getOrigin()) ||
      hiros::skeletons::utils::isNaN(t_tf.getRotation())) {
    return;
  }

  updateClusters(t_tf);
}

void hiros::hdt::TfBuffer::updateClusters(const tf2::Transform& t_tf) {
  // Create first cluster
  if (clusters_.empty()) {
    clusters_.push_back({weight_, t_tf});
    return;
  }

  // Calculate distances from the clusters
  std::vector<double> distances{};
  distances.reserve(clusters_.size());
  for (const auto& cluster : clusters_) {
    distances.push_back(utils::normalizedDistance(t_tf, cluster.avg()));
  }

  // Find the minimum distance and update clusters
  auto min_iterator{std::min_element(distances.begin(), distances.end())};
  auto min_distance{*min_iterator};
  auto min_index{
      static_cast<unsigned>(std::distance(distances.begin(), min_iterator))};

  if (min_distance < clustering_threshold_) {
    clusters_.at(min_index).push_back(t_tf);
  } else {
    // Create new cluster
    clusters_.push_back({weight_, t_tf});
  }

  cleanupClusters();
  sortClusters();
  mergeCloseClusters();
}

void hiros::hdt::TfBuffer::cleanupClusters() {
  weightBasedCleanup();
  timeBasedCleanup();
}

void hiros::hdt::TfBuffer::weightBasedCleanup() {
  for (auto& cluster : clusters_) {
    while (cluster.size() > max_cluster_size_) {
      cluster.pop_front();
    }
  }
}

void hiros::hdt::TfBuffer::timeBasedCleanup() {
  // Remove clusters where the newest element is older than time_threshold_
  clusters_.erase(std::remove_if(clusters_.begin(), clusters_.end(),
                                 [&](const auto& cluster) {
                                   return cluster.age().count() >
                                          time_threshold_;
                                 }),
                  clusters_.end());
}

void hiros::hdt::TfBuffer::sortClusters() {
  // This way, the buffer to use is always clusters_.front()
  std::sort(
      clusters_.begin(), clusters_.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.size() > rhs.size(); });
}

void hiros::hdt::TfBuffer::mergeCloseClusters() {
  if (clusters_.size() <= 1) {
    return;
  }

  // Merge the two largest clusters if their weighted centroids are closer
  // than clustering_threshold_
  if (utils::normalizedDistance(clusters_.front().avg(),
                                clusters_.at(1).avg()) <
      clustering_threshold_) {
    // Merge buffer and first cluster
    clusters_.front().merge(clusters_.at(1));

    // Resize the buffer to respect max_cluster_size_
    if (clusters_.front().size() > max_cluster_size_) {
      clusters_.front().resize(max_cluster_size_);
    }

    // Erase the merged cluster
    clusters_.erase(clusters_.begin() + 1);
  }
}
