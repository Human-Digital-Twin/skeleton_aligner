// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/Aligner.h"
#include "skeleton_aligner/utils.h"

const std::string k_bash_msg_reset{"\033[0m"};
const std::string k_bash_msg_green{"\033[32m"};

hiros::hdt::Aligner::Aligner() : Node("hiros_hdt_aligner") { start(); }

hiros::hdt::Aligner::~Aligner() { stop(); }

void hiros::hdt::Aligner::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Running" << k_bash_msg_reset);
}

void hiros::hdt::Aligner::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Stopped" << k_bash_msg_reset);

  rclcpp::shutdown();
}

void hiros::hdt::Aligner::configure() {
  getParams();
  setupRos();
}

void hiros::hdt::Aligner::getParams() {
  getParam("kinect_input_topic", params_.kinect_input_topic);
  getParam("xsens_input_topic", params_.xsens_input_topic);
  getParam("output_topic", params_.output_topic);

  getParam("publish_tfs", params_.publish_tfs);

  getMarkersConfig("translation_markers", params_.translation_marker_ids);
  getMarkersConfig("rotation_markers", params_.rotation_marker_ids);
}

void hiros::hdt::Aligner::getMarkersConfig(
    const std::string& t_param_name,
    std::vector<utils::MarkerPair>& t_marker_ids) {
  std::vector<std::string> markers{};
  getParam(t_param_name, markers);

  int kinect_id{}, xsens_id{};
  for (const auto& marker : markers) {
    getParam(marker + ".kinect_id", kinect_id);
    getParam(marker + ".xsens_id", xsens_id);
    t_marker_ids.push_back({kinect_id, xsens_id});
  }
}

void hiros::hdt::Aligner::setupRos() {
  if (params_.publish_tfs) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  kinect_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.kinect_input_topic, 10,
          std::bind(&Aligner::kinectCallback, this, std::placeholders::_1));
  xsens_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.xsens_input_topic, 10,
          std::bind(&Aligner::xsensCallback, this, std::placeholders::_1));

  aligned_skel_pub_ = create_publisher<hiros_skeleton_msgs::msg::StampedSkeleton>(
      params_.output_topic, 10);
}

geometry_msgs::msg::TransformStamped hiros::hdt::Aligner::ks2tf(
    const std::string& name,
    const hiros::skeletons::types::KinematicState& ks) const {
  geometry_msgs::msg::TransformStamped tf{};

  tf.header.frame_id = aligned_skeleton_.frame;
  tf.header.stamp = rclcpp::Time{static_cast<long>(aligned_skeleton_.time * 1e9)};
  tf.child_frame_id = name;
  tf.transform.translation = skeletons::utils::toVector3Msg(ks.pose.position);
  tf.transform.rotation = skeletons::utils::toMsg(ks.pose.orientation);

  return tf;
}

void hiros::hdt::Aligner::publishTfs() {
  if (params_.publish_tfs) {
    for (const auto& link : aligned_skeleton_.links) {
      if (!skeletons::utils::isNaN(link.center.pose.position) &&
          !skeletons::utils::isNaN(link.center.pose.orientation)) {
        tf_broadcaster_->sendTransform(ks2tf(
            std::to_string(aligned_skeleton_.id) + "_l" + std::to_string(link.id),
            link.center));
      }
    }
  }
}

void hiros::hdt::Aligner::publishAlignedSkeleton() {
  aligned_skel_pub_->publish(
      hiros::skeletons::utils::toStampedMsg(aligned_skeleton_));
}

void hiros::hdt::Aligner::computeRotation() {
  if (params_.kinect_marker_ids.arePresentIn(kinect_skeleton_) &&
      params_.xsens_marker_ids.arePresentIn(xsens_skeleton_)) {
    // TODO: compute correct quaternion
    transform_.setRotation({0, 0, 0, 1});
  }
}

void hiros::hdt::Aligner::computeTranslation() {
  auto kinect_marker_ids{
      utils::kinectMarkerIds(params_.translation_marker_ids)};
  auto xsens_marker_ids{utils::xsensMarkerIds(params_.translation_marker_ids)};

  if (utils::skeletonContains(kinect_skeleton_, kinect_marker_ids) &&
      utils::skeletonContains(xsens_skeleton_, xsens_marker_ids)) {
    // T = [R t2-R*t1
    //      0       1]
    transform_.setOrigin(
        utils::avg(utils::extractMarkers(kinect_skeleton_, kinect_marker_ids)) -
        transform_.getBasis() * utils::avg(utils::extractMarkers(
                                    xsens_skeleton_, xsens_marker_ids)));
  }
}

void hiros::hdt::Aligner::computeTransform() {
  if (kinect_skeleton_.markers.empty() || xsens_skeleton_.markers.empty()) {
    // This way, when the Kinect skeleton is not available we keep the last
    // computed transform
    return;
  }

  computeRotation();
  computeTranslation();
}

void hiros::hdt::Aligner::alignSkeleton() {
  aligned_skeleton_ = xsens_skeleton_;
  hiros::hdt::utils::transform(aligned_skeleton_, transform_);
}

void hiros::hdt::Aligner::clearSkeletons() {
  kinect_skeleton_ = {};
  xsens_skeleton_ = {};
}

void hiros::hdt::Aligner::processSkeleton() {
  computeTransform();
  alignSkeleton();
  clearSkeletons();
  publishTfs();
  publishAlignedSkeleton();
}

void hiros::hdt::Aligner::kinectCallback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  kinect_skeleton_ = skeletons::utils::toStruct(msg);
}

void hiros::hdt::Aligner::xsensCallback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  xsens_skeleton_ = skeletons::utils::toStruct(msg);
  processSkeleton();
}
