// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom external packages dependencies
#include "skeleton_fusion/Filter.h"
#include "skeletons/utils.h"

const std::string k_bash_msg_reset{"\033[0m"};
const std::string k_bash_msg_green{"\033[32m"};

hiros::hdt::Filter::Filter() : Node("hiros_hdt_filter") { start(); }

hiros::hdt::Filter::~Filter() { stop(); }

void hiros::hdt::Filter::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Running" << k_bash_msg_reset);
}

void hiros::hdt::Filter::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Stopped" << k_bash_msg_reset);

  rclcpp::shutdown();
}

void hiros::hdt::Filter::configure() {
  getParams();
  setupRos();
}

void hiros::hdt::Filter::getParams() {
  getParam("kinect_input_topic", params_.kinect_input_topic);
  getParam("xsens_input_topic", params_.xsens_input_topic);
  getParam("output_topic", params_.output_topic);
  getParam("publish_tfs", params_.publish_tfs);
}

void hiros::hdt::Filter::setupRos() {
  if (params_.publish_tfs) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  kinect_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.kinect_input_topic, 10,
          std::bind(&Filter::kinect_callback, this, std::placeholders::_1));
  xsens_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.xsens_input_topic, 10,
          std::bind(&Filter::xsens_callback, this, std::placeholders::_1));

  fused_skel_pub_ = create_publisher<hiros_skeleton_msgs::msg::StampedSkeleton>(
      params_.output_topic, 10);
}

geometry_msgs::msg::TransformStamped hiros::hdt::Filter::ks2tf(
    const std::string& name,
    const hiros::skeletons::types::KinematicState& ks) const {
  geometry_msgs::msg::TransformStamped tf{};

  tf.header.frame_id = fused_skeleton_.frame;
  tf.header.stamp = rclcpp::Time{static_cast<long>(fused_skeleton_.time * 1e9)};
  tf.child_frame_id = name;
  tf.transform.translation = skeletons::utils::toVector3Msg(ks.pose.position);
  tf.transform.rotation = skeletons::utils::toMsg(ks.pose.orientation);

  return tf;
}

void hiros::hdt::Filter::publishTfs() {
  if (params_.publish_tfs) {
    for (const auto& link : fused_skeleton_.links) {
      if (!skeletons::utils::isNaN(link.center.pose.position) &&
          !skeletons::utils::isNaN(link.center.pose.orientation)) {
        tf_broadcaster_->sendTransform(ks2tf(
            std::to_string(fused_skeleton_.id) + "_l" + std::to_string(link.id),
            link.center));
      }
    }
  }
}

void hiros::hdt::Filter::publishFusedSkeleton() {
  fused_skel_pub_->publish(
      hiros::skeletons::utils::toStampedMsg(fused_skeleton_));
}

void hiros::hdt::Filter::processSkeletons() {
  publishTfs();
  publishFusedSkeleton();
}

void hiros::hdt::Filter::kinect_callback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  kinect_skeleton_ = skeletons::utils::toStruct(msg);
  fused_skeleton_ = kinect_skeleton_;  // TODO: remove
  processSkeletons();
}

void hiros::hdt::Filter::xsens_callback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  xsens_skeleton_ = skeletons::utils::toStruct(msg);
  fused_skeleton_ = xsens_skeleton_;  // TODO: remove
  processSkeletons();
}
