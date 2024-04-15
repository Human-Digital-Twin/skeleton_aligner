// Eigen dependencies
#include "eigen3/Eigen/Eigen"

// ROS dependencies
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

  getParam("weight", params_.weight);
  buffer_ptr_ = std::make_unique<TfBuffer>(params_.weight);

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
  tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  kinect_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.kinect_input_topic, 10,
          std::bind(&Aligner::kinectCallback, this, std::placeholders::_1));
  xsens_skel_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
          params_.xsens_input_topic, 10,
          std::bind(&Aligner::xsensCallback, this, std::placeholders::_1));
}

void hiros::hdt::Aligner::updateTransform() {
  if (kinect_skeleton_.markers.empty() || xsens_skeleton_.markers.empty()) {
    // This way, when one of the skeletons is not available we keep the last
    // computed transform
    return;
  }

  computeTransform();
  publishTransform();
  clearSkeletons();
}

void hiros::hdt::Aligner::computeTransform() {
  computeRotation();
  computeTranslation();
  buffer_ptr_->push_back(transform_);
}

void hiros::hdt::Aligner::publishTransform() {
  geometry_msgs::msg::TransformStamped tf{};

  tf.header.frame_id = kinect_frame_id_;
  tf.header.stamp = now();
  tf.child_frame_id = xsens_frame_id_;
  tf.transform = tf2::toMsg(buffer_ptr_->avg());

  tf_broadcaster_->sendTransform(tf);
}

void hiros::hdt::Aligner::clearSkeletons() {
  kinect_skeleton_ = {};
  xsens_skeleton_ = {};
}

void hiros::hdt::Aligner::computeRotation() {
  auto kinect_marker_ids{utils::kinectMarkerIds(params_.rotation_marker_ids)};
  auto xsens_marker_ids{utils::xsensMarkerIds(params_.rotation_marker_ids)};

  if (kinect_marker_ids.size() != xsens_marker_ids.size()) {
    std::cerr << "Error: computeRotation() dimension mismatch" << std::endl;
    return;
  }

  if (utils::skeletonContains(kinect_skeleton_, kinect_marker_ids) &&
      utils::skeletonContains(xsens_skeleton_, xsens_marker_ids)) {
    Eigen::MatrixXd kinect_points{3, kinect_marker_ids.size()};
    Eigen::MatrixXd xsens_points{3, xsens_marker_ids.size()};

    unsigned int i{0};
    for (const auto& mk_id : kinect_marker_ids) {
      kinect_points.col(i++)
          << kinect_skeleton_.getMarker(mk_id).center.pose.position.x(),
          kinect_skeleton_.getMarker(mk_id).center.pose.position.y(),
          kinect_skeleton_.getMarker(mk_id).center.pose.position.z();
    }
    // Replace NaNs with zeros
    kinect_points = (!kinect_points.array().isNaN()).select(kinect_points, 0);

    i = 0;
    for (const auto& mk_id : xsens_marker_ids) {
      xsens_points.col(i++)
          << xsens_skeleton_.getMarker(mk_id).center.pose.position.x(),
          xsens_skeleton_.getMarker(mk_id).center.pose.position.y(),
          xsens_skeleton_.getMarker(mk_id).center.pose.position.z();
    }
    // Replace NaNs with zeros
    xsens_points = (!xsens_points.array().isNaN()).select(xsens_points, 0);

    // Center matrixes
    Eigen::MatrixXd kinect_centered{kinect_points.colwise() -
                                    kinect_points.rowwise().mean()};
    Eigen::MatrixXd xsens_centered{xsens_points.colwise() -
                                   xsens_points.rowwise().mean()};

    // Compute covariance matrix H
    Eigen::Matrix3d covariance{xsens_centered * kinect_centered.transpose()};

    // Compute SVD and R
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov{
        covariance, Eigen::ComputeThinU | Eigen::ComputeThinV};
    Eigen::Matrix3d rotation{svd_cov.matrixV() * svd_cov.matrixU().transpose()};

    // Special reflection case
    if (rotation.determinant() < 0) {
      rotation = svd_cov.matrixV() * Eigen::Vector3d(1., 1., -1.).asDiagonal() *
                 svd_cov.matrixU().transpose();
    }

    // Set transform rotation
    transform_.setRotation(hiros::skeletons::utils::toStruct(
        tf2::toMsg(Eigen::Quaterniond(rotation))));
  }
}

void hiros::hdt::Aligner::computeTranslation() {
  auto kinect_marker_ids{
      utils::kinectMarkerIds(params_.translation_marker_ids)};
  auto xsens_marker_ids{utils::xsensMarkerIds(params_.translation_marker_ids)};

  if (kinect_marker_ids.size() != xsens_marker_ids.size()) {
    std::cerr << "Error: computeTranslation() dimension mismatch" << std::endl;
    return;
  }

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

void hiros::hdt::Aligner::callback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& t_msg,
    std::string& t_frame_id, hiros::skeletons::types::Skeleton& t_skeleton) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  if (t_frame_id.empty()) {
    t_frame_id = t_msg.header.frame_id;
  }
  t_skeleton = skeletons::utils::toStruct(t_msg);

  updateTransform();
}

void hiros::hdt::Aligner::kinectCallback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  callback(msg, kinect_frame_id_, kinect_skeleton_);
}

void hiros::hdt::Aligner::xsensCallback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
  callback(msg, xsens_frame_id_, xsens_skeleton_);
}
