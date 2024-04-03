// Standard dependencies
#include <numeric>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/utils.h"

std::vector<int> hiros::hdt::utils::kinectMarkerIds(
    const std::vector<MarkerPair>& t_marker_pairs) {
  std::vector<int> ids{};
  ids.reserve(t_marker_pairs.size());

  for (const auto& marker_pair : t_marker_pairs) {
    ids.push_back(marker_pair.kinect_id);
  }

  return ids;
}

std::vector<int> hiros::hdt::utils::xsensMarkerIds(
    const std::vector<MarkerPair>& t_marker_pairs) {
  std::vector<int> ids{};
  ids.reserve(t_marker_pairs.size());

  for (const auto& marker_pair : t_marker_pairs) {
    ids.push_back(marker_pair.xsens_id);
  }

  return ids;
}

bool hiros::hdt::utils::skeletonContains(
    const hiros::skeletons::types::Skeleton& t_skel,
    const std::vector<int>& t_marker_ids) {
  for (const auto& mk_id : t_marker_ids) {
    if (!t_skel.hasMarker(mk_id)) {
      return false;
    }
  }
  return true;
}

std::vector<hiros::skeletons::types::Point> hiros::hdt::utils::extractMarkers(
    const hiros::skeletons::types::Skeleton& t_skel,
    const std::vector<int>& t_marker_ids) {
  std::vector<hiros::skeletons::types::Point> res{};

  for (const auto& marker_id : t_marker_ids) {
    if (t_skel.hasMarker(marker_id)) {
      res.push_back(t_skel.getMarker(marker_id).center.pose.position);
    }
  }

  return res;
}

hiros::skeletons::types::Point hiros::hdt::utils::avg(
    const std::vector<hiros::skeletons::types::Point>& t_v) {
  return std::accumulate(t_v.begin(), t_v.end(),
                         hiros::skeletons::types::Point{0., 0., 0.}) /
         t_v.size();
}

bool hiros::hdt::utils::isNaN(const tf2::Transform& t_tf) {
  return hiros::skeletons::utils::isNaN(t_tf.getOrigin()) ||
         hiros::skeletons::utils::isNaN(t_tf.getRotation());
}

void hiros::hdt::utils::transform(hiros::skeletons::types::KinematicState& t_ks,
                                  const tf2::Transform& t_tf) {
  if (isNaN(t_tf)) {
    throw std::runtime_error("NaN transform");
  }

  if (!hiros::skeletons::utils::isNaN(t_ks.pose.position)) {
    t_ks.pose.position = t_tf * t_ks.pose.position;
  }
  if (!hiros::skeletons::utils::isNaN(t_ks.pose.orientation)) {
    t_ks.pose.orientation = t_tf * t_ks.pose.orientation;
  }
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Marker>& t_mks,
    const tf2::Transform& t_tf) {
  for (auto& mk : t_mks) {
    transform(mk.center, t_tf);
  }
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Link>& t_lks,
    const tf2::Transform& t_tf) {
  for (auto& lk : t_lks) {
    transform(lk.center, t_tf);
  }
}

void hiros::hdt::utils::transform(hiros::skeletons::types::Skeleton& t_skel,
                                  const tf2::Transform& t_tf) {
  transform(t_skel.markers, t_tf);
  transform(t_skel.links, t_tf);
  t_skel.bounding_box = hiros::skeletons::utils::computeBoundingBox(t_skel);
}
