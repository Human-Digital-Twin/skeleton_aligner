#ifndef hiros_skeleton_aligner_Aligner_h
#define hiros_skeleton_aligner_Aligner_h

// ROS dependencies
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// Custom external packages dependencies
#include "hiros_skeleton_msgs/msg/stamped_skeleton.hpp"
#include "skeletons/types.h"

// Internal dependencies
#include "skeleton_aligner/TfBuffer.h"
#include "skeleton_aligner/utils.h"

namespace hiros {
namespace hdt {

class Aligner : public rclcpp::Node {
 public:
  Aligner();
  ~Aligner();

 private:
  struct Parameters {
    std::vector<std::string> input_topics{};
    double weight{};
    utils::MarkersMap marker_ids{};
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
  void getMarkersConfig();
  void setupRos();

  void updateTransform();

  void computeTransform();
  void publishTransform();
  void clearSkeletons();

  void computeRotation();
  void computeTranslation();

  void getRootTransforms();
  void getRootFrames();
  std::string getRootFrame(std::string child_frame);

  void callback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg,
                const std::string& topic_name);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_{};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{};

  std::vector<rclcpp::Subscription<
      hiros_skeleton_msgs::msg::StampedSkeleton>::SharedPtr>
      subs_{};

  Parameters params_{};

  std::unique_ptr<TfBuffer> buffer_ptr_{};

  // map<input_topic, transform>
  std::map<std::string, tf2::Stamped<tf2::Transform>> root_tfs_map_{};
  tf2::Stamped<tf2::Transform> transform_{{}, {}, {}};

  // map<input_topic, skeleton>
  std::map<std::string, hiros::skeletons::types::Skeleton> skeletons_map_{};
};

}  // namespace hdt
}  // namespace hiros

#endif
