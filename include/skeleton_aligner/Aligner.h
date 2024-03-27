#ifndef hiros_skeleton_aligner_Aligner_h
#define hiros_skeleton_aligner_Aligner_h

// ROS dependencies
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// Custom external packages dependencies
#include "hiros_skeleton_msgs/msg/stamped_skeleton.hpp"
#include "skeletons/types.h"

// Internal dependencies
#include "skeleton_aligner/utils.h"

namespace hiros {
namespace hdt {

class Aligner : public rclcpp::Node {
 public:
  Aligner();
  ~Aligner();

 private:
  struct Parameters {
    utils::MarkerIds kinect_marker_ids{};
    utils::MarkerIds xsens_marker_ids{};

    std::string kinect_input_topic{};
    std::string xsens_input_topic{};
    std::string output_topic{};

    bool publish_tfs{false};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name, parameter);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRos();

  geometry_msgs::msg::TransformStamped ks2tf(
      const std::string& name,
      const hiros::skeletons::types::KinematicState& ks) const;

  void publishTfs();
  void publishAlignedSkeleton();

  void computeRotation();
  void computeTranslation();
  void computeTransform();

  void alignSkeleton();
  void clearSkeletons();
  void processSkeleton();

  void kinectCallback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg);
  void xsensCallback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  rclcpp::Subscription<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      kinect_skel_sub_{};
  rclcpp::Subscription<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      xsens_skel_sub_{};

  rclcpp::Publisher<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      aligned_skel_pub_{};

  Parameters params_{};

  tf2::Transform transform_{};

  hiros::skeletons::types::Skeleton kinect_skeleton_{};
  hiros::skeletons::types::Skeleton xsens_skeleton_{};
  hiros::skeletons::types::Skeleton aligned_skeleton_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
