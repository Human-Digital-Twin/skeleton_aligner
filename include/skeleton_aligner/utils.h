#ifndef hiros_skeleton_aligner_utils_h
#define hiros_skeleton_aligner_utils_h

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom external dependencies
#include "skeletons/types.h"

namespace hiros {
namespace hdt {
namespace utils {

struct MarkerPair {
  int kinect_id{};
  int xsens_id{};
};

std::vector<int> kinectMarkerIds(const std::vector<MarkerPair>& t_marker_pairs);
std::vector<int> xsensMarkerIds(const std::vector<MarkerPair>& t_marker_pairs);

bool skeletonContains(const hiros::skeletons::types::Skeleton& t_skel,
                      const std::vector<int>& t_marker_ids);

std::vector<hiros::skeletons::types::Point> extractMarkers(
    const hiros::skeletons::types::Skeleton& t_skel,
    const std::vector<int>& t_marker_ids);

hiros::skeletons::types::Point avg(
    const std::vector<hiros::skeletons::types::Point>& t_v);

bool isNaN(const tf2::Transform& t_tf);

void transform(hiros::skeletons::types::KinematicState& t_ks,
               const tf2::Transform& t_tf);
void transform(std::vector<hiros::skeletons::types::Marker>& t_mks,
               const tf2::Transform& t_tf);
void transform(std::vector<hiros::skeletons::types::Link>& t_lks,
               const tf2::Transform& t_tf);
void transform(hiros::skeletons::types::Skeleton& t_skel,
               const tf2::Transform& t_tf);

}  // namespace utils
}  // namespace hdt
}  // namespace hiros

#endif
