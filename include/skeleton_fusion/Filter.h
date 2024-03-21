#ifndef skeleton_fusion_Filter_h
#define skeleton_fusion_Filter_h

// ROS dependencies
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// Custom external packages dependencies
#include "hiros_skeleton_msgs/msg/stamped_skeleton.hpp"
#include "skeletons/types.h"

namespace hiros {
namespace hdt {

class Filter : public rclcpp::Node {
 public:
  Filter();
  ~Filter();

 private:
  struct Parameters {
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
  void publishFusedSkeleton();

  void processSkeletons();

  void kinect_callback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg);
  void xsens_callback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  rclcpp::Subscription<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      kinect_skel_sub_{};
  rclcpp::Subscription<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      xsens_skel_sub_{};

  rclcpp::Publisher<hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr
      fused_skel_pub_{};

  Parameters params_{};

  hiros::skeletons::types::Skeleton kinect_skeleton_{};
  hiros::skeletons::types::Skeleton xsens_skeleton_{};
  hiros::skeletons::types::Skeleton fused_skeleton_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
