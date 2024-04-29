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
  getParam("input_topics", params_.input_topics);
  if (params_.input_topics.size() != 2) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Only supporting two input topics. Closing");
    stop();
    exit(EXIT_FAILURE);
  }

  // Initialize skeletons_map_
  for (const auto& topic : params_.input_topics) {
    skeletons_map_[topic] = {};
  }

  getParam("weight", params_.weight);
  buffer_ptr_ = std::make_unique<TfBuffer>(params_.weight);

  getMarkersConfig();
}

void hiros::hdt::Aligner::getMarkersConfig() {
  std::vector<std::string> input_sources{};
  getParam("input_sources", input_sources);
  if (input_sources.size() != params_.input_topics.size()) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Input topics and sources size mismatch. Closing");
    stop();
    exit(EXIT_FAILURE);
  }

  std::vector<long> ids{};
  for (auto i{0u}; i < input_sources.size(); ++i) {
    getParam("translation_ids." + input_sources.at(i), ids);
    params_.marker_ids.translation[params_.input_topics.at(i)] = ids;

    getParam("rotation_ids." + input_sources.at(i), ids);
    params_.marker_ids.rotation[params_.input_topics.at(i)] = ids;
  }
}

void hiros::hdt::Aligner::setupRos() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  for (const auto& topic : params_.input_topics) {
    subs_.push_back(create_subscription<
                    hiros_skeleton_msgs::msg::StampedSkeleton>(
        topic, 10,
        std::function<void(const hiros_skeleton_msgs::msg::StampedSkeleton&)>{
            std::bind(&Aligner::callback, this, std::placeholders::_1,
                      topic)}));
  }
}

void hiros::hdt::Aligner::updateTransform() {
  if (std::any_of(skeletons_map_.cbegin(), skeletons_map_.cend(),
                  [](const auto& pair) {
                    const auto& [topic, skeleton] = pair;
                    return skeleton.markers.empty();
                  })) {
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
  getRootTransforms();

  geometry_msgs::msg::TransformStamped tf{};

  const auto& parent_topic{root_tfs_map_.at(params_.input_topics.front())};
  const auto& child_topic{root_tfs_map_.at(params_.input_topics.back())};

  tf.header.stamp = now();
  tf.header.frame_id = parent_topic.frame_id_;
  tf.child_frame_id = child_topic.frame_id_;
  tf.transform =
      tf2::toMsg(parent_topic * buffer_ptr_->avg() * child_topic.inverse());

  static_tf_broadcaster_->sendTransform(tf);
}

void hiros::hdt::Aligner::clearSkeletons() {
  std::for_each(skeletons_map_.begin(), skeletons_map_.end(), [](auto& pair) {
    auto& [topic, skeleton] = pair;
    skeleton = {};
  });
}

void hiros::hdt::Aligner::computeRotation() {
  if (std::any_of(skeletons_map_.begin(), skeletons_map_.end(),
                  [&](const auto& pair) {
                    const auto& [topic, skeleton] = pair;
                    return !utils::skeletonContains(
                        skeleton, params_.marker_ids.rotation.at(topic));
                  })) {
    // Return if any skeleton is missing markers
    return;
  }

  std::map<std::string, Eigen::MatrixXd> points_map{};
  for (const auto& topic : params_.input_topics) {
    points_map[topic] = {3, params_.marker_ids.rotation.at(topic).size()};

    auto i{0u};
    for (const auto& mk_id : params_.marker_ids.rotation.at(topic)) {
      const auto& point{skeletons_map_.at(topic)
                            .getMarker(static_cast<int>(mk_id))
                            .center.pose.position};
      points_map.at(topic).col(i++) << point.x(), point.y(), point.z();
    }

    // Replace NaNs with zeros
    points_map.at(topic) = (!points_map.at(topic).array().isNaN())
                               .select(points_map.at(topic), 0.);
  }

  // Center matrixes
  std::map<std::string, Eigen::MatrixXd> centered_map{};
  for (const auto& topic : params_.input_topics) {
    centered_map[topic] =
        points_map.at(topic).colwise() - points_map.at(topic).rowwise().mean();
  }

  // Compute covariance matrix H
  Eigen::Matrix3d covariance{
      centered_map.at(params_.input_topics.back()) *
      centered_map.at(params_.input_topics.front()).transpose()};

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

void hiros::hdt::Aligner::computeTranslation() {
  if (std::any_of(skeletons_map_.cbegin(), skeletons_map_.cend(),
                  [&](const auto& pair) {
                    const auto& [topic, skeleton] = pair;
                    return !utils::skeletonContains(
                        skeleton, params_.marker_ids.rotation.at(topic));
                  })) {
    // Return if any skeleton is missing markers
    return;
  }

  const auto& parent_topic{params_.input_topics.front()};
  const auto& child_topic{params_.input_topics.back()};

  // T = [R t2-R*t1
  //      0       1]
  transform_.setOrigin(
      utils::avg(utils::extractMarkers(
          skeletons_map_.at(parent_topic),
          params_.marker_ids.translation.at(parent_topic))) -
      transform_.getBasis() *
          utils::avg(utils::extractMarkers(
              skeletons_map_.at(child_topic),
              params_.marker_ids.translation.at(child_topic))));
}

void hiros::hdt::Aligner::getRootTransforms() {
  if (root_tfs_map_.size() == params_.input_topics.size()) {
    return;
  }

  getRootFrames();

  for (const auto& topic : params_.input_topics) {
    tf2::fromMsg(tf_buffer_->lookupTransform(root_tfs_map_.at(topic).frame_id_,
                                             skeletons_map_.at(topic).frame,
                                             tf2::TimePointZero),
                 root_tfs_map_.at(topic));
  }
}

void hiros::hdt::Aligner::getRootFrames() {
  for (const auto& topic : params_.input_topics) {
    root_tfs_map_[topic].frame_id_ =
        getRootFrame(skeletons_map_.at(topic).frame);
  }
}

std::string hiros::hdt::Aligner::getRootFrame(std::string child_frame) {
  std::string parent_frame{};

  while (parent_frame.empty() || parent_frame != child_frame) {
    if (!tf_buffer_->_getParent(child_frame, tf2::TimePointZero,
                                parent_frame)) {
      return child_frame;
    }

    child_frame = parent_frame;
    tf_buffer_->_getParent(child_frame, tf2::TimePointZero, parent_frame);
  }

  return parent_frame;
}

void hiros::hdt::Aligner::callback(
    const hiros_skeleton_msgs::msg::StampedSkeleton& msg,
    const std::string& topic_name) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  skeletons_map_.at(topic_name) = skeletons::utils::toStruct(msg);
  updateTransform();
}
