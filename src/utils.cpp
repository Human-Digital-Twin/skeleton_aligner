// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_fusion/utils.h"

std::vector<int> hiros::hdt::utils::MarkerIds::toVec() const {
  return {pelvis, right_hip, left_hip};
}

bool hiros::hdt::utils::MarkerIds::arePresentIn(
    const hiros::skeletons::types::Skeleton& t_skel) const {
  for (const auto& mk_id : toVec()) {
    if (!t_skel.hasMarker(mk_id)) {
      return false;
    }
  }
  return true;
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
