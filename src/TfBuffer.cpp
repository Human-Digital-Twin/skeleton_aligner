// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

hiros::hdt::TfBuffer::TfBuffer(const double& t_weight,
                               const double& t_time_threshold,
                               const double& t_clustering_threshold)
    : weight_{t_weight},
      time_threshold_{t_time_threshold},
      clustering_threshold_{t_clustering_threshold} {
  if (clustering_threshold_ <= 0. || clustering_threshold_ >= 1.) {
    clustering_threshold_ = k_default_clustering_threshold;
  }

  if (time_threshold_ <= 0.) {
    time_threshold_ = std::numeric_limits<double>::max();
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
    clusters_.push_back({t_tf, weight_});
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
    clusters_.push_back({t_tf, weight_});
  }

  timeBasedCleanup();
  sortClusters();
  distanceBasedCleanup();
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

void hiros::hdt::TfBuffer::distanceBasedCleanup() {
  // Remove the smaller cluster where the normalized distance from the larger is
  // lower than clustering_threshold_
  if (clusters_.size() <= 1) {
    return;
  }

  if (utils::normalizedDistance(clusters_.front().avg(),
                                clusters_.at(1).avg()) <
      clustering_threshold_) {
    clusters_.erase(clusters_.begin() + 1);
  }
}
