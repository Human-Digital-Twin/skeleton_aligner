#ifndef hiros_skeleton_fusion_utils_h
#define hiros_skeleton_fusion_utils_h

// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom external dependencies
#include "skeletons/types.h"

namespace hiros {
namespace hdt {
namespace utils {

struct MarkerIds {
  int pelvis{};
  int right_hip{};
  int left_hip{};

  std::vector<int> toVec() const;
  bool arePresentIn(const hiros::skeletons::types::Skeleton& t_skel) const;
};

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
