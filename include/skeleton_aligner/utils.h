#ifndef hiros_skeleton_aligner_utils_h
#define hiros_skeleton_aligner_utils_h

// Standard dependencies
#include <map>

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom external dependencies
#include "skeletons/types.h"

namespace hiros {
namespace hdt {
namespace utils {

struct MarkersMap {
  // map<input_topic, vector<ID>>
  std::map<std::string, std::vector<long>> translation{};
  std::map<std::string, std::vector<long>> rotation{};
};

bool skeletonContains(const hiros::skeletons::types::Skeleton& skel,
                      const std::vector<long>& marker_ids);

std::vector<hiros::skeletons::types::Point> extractMarkers(
    const hiros::skeletons::types::Skeleton& skel,
    const std::vector<long>& marker_ids);

hiros::skeletons::types::Point avg(
    const std::vector<hiros::skeletons::types::Point>& v);

bool isNaN(const tf2::Transform& t_tf);

double translationDistance(const tf2::Transform& tf1,
                           const tf2::Transform& tf2);
double rotationDistance(const tf2::Transform& tf1, const tf2::Transform& tf2);
double normalizedDistance(
    const tf2::Transform& t_tf1, const tf2::Transform& tf2,
    const double& max_translation_distance = 2. /* [m] */,
    const double& max_rotation_distance = 2. * M_PI /* [rad] */);

void transform(hiros::skeletons::types::KinematicState& ks,
               const tf2::Transform& tf);
void transform(std::vector<hiros::skeletons::types::Marker>& mks,
               const tf2::Transform& tf);
void transform(std::vector<hiros::skeletons::types::Link>& lks,
               const tf2::Transform& tf);
void transform(hiros::skeletons::types::Skeleton& skel,
               const tf2::Transform& tf);

tf2::Transform solveWeightedLeastSquares(const std::vector<tf2::Transform>& As,
                                         const std::vector<tf2::Transform>& bs,
                                         const double& weight = 1.);
tf2::Transform weightedAverage(const std::vector<tf2::Transform>& tfs,
                               const double& weight = 1.);

}  // namespace utils
}  // namespace hdt
}  // namespace hiros

#endif
