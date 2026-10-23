// Standard dependencies
#include <algorithm>
#include <stdexcept>

// External dependencies
#include <Eigen/Dense>

// ROS dependencies
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/Aligner.h"

const std::string k_bash_msg_reset{"\033[0m"};
const std::string k_bash_msg_green{"\033[32m"};

namespace hdt::skeleton {

Aligner::Aligner(const rclcpp::NodeOptions& options)
    : Node("skeleton_aligner", options) {
  start();
}

Aligner::~Aligner() { stop(); }

void Aligner::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Running" << k_bash_msg_reset);
}

void Aligner::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     k_bash_msg_green << "Stopped" << k_bash_msg_reset);
  rclcpp::shutdown();
}

void Aligner::configure() {
  getParams();
  setupRos();
}

void Aligner::getParams() {
  getParam("input_topics", params_.input_topics);
  if (params_.input_topics.size() != 2) {
    throw std::invalid_argument(
        "Parameter 'input_topics' must contain exactly two topics.");
  }

  // Initialize skeletons_map_
  for (const auto& topic : params_.input_topics) {
    skeletons_map_[topic] = {};
  }

  getParam("weight", params_.weight);
  buffer_ptr_ = std::make_unique<TfBuffer>(params_.weight);

  getMarkersConfig();
}

void Aligner::getMarkersConfig() {
  std::vector<std::string> input_sources{};
  getParam("input_sources", input_sources);
  if (input_sources.size() != params_.input_topics.size()) {
    throw std::invalid_argument(
        "Input topics and sources size mismatch in configuration.");
  }

  std::vector<long> ids{};
  for (auto i{0u}; i < input_sources.size(); ++i) {
    getParam("translation_ids." + input_sources.at(i), ids);
    if (ids.empty()) {
      throw std::invalid_argument(
          "At least 1 translation ID is required for input source: " +
          input_sources.at(i));
    }
    params_.marker_ids.translation[params_.input_topics.at(i)] = ids;

    getParam("rotation_ids." + input_sources.at(i), ids);
    if (ids.size() < 3) {
      throw std::invalid_argument(
          "At least 3 rotation IDs are required for input source: " +
          input_sources.at(i));
    }
    params_.marker_ids.rotation[params_.input_topics.at(i)] = ids;
  }
}

void Aligner::setupRos() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  for (const auto& topic : params_.input_topics) {
    subs_.push_back(
        create_subscription<hiros_skeleton_msgs::msg::StampedSkeleton>(
            topic, 10,
            [this,
             topic](const hiros_skeleton_msgs::msg::StampedSkeleton& msg) {
              callback(msg, topic);
            }));
  }
}

void Aligner::updateTransform() {
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

void Aligner::computeTransform() {
  if (computeRotation() && computeTranslation()) {
    buffer_ptr_->push_back(transform_);
  }
}

void Aligner::publishTransform() {
  getRootTransforms();

  const auto& parent_tf{root_tfs_map_.at(params_.input_topics.front())};
  const auto& child_tf{root_tfs_map_.at(params_.input_topics.back())};

  const auto& avg_tf{buffer_ptr_->avg()};
  if (utils::isNaN(avg_tf)) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf{};
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = parent_tf.frame_id_;
  tf.child_frame_id = child_tf.frame_id_;
  tf.transform = tf2::toMsg(parent_tf * avg_tf * child_tf.inverse());

  static_tf_broadcaster_->sendTransform(tf);
}

void Aligner::clearSkeletons() {
  std::for_each(skeletons_map_.begin(), skeletons_map_.end(), [](auto& pair) {
    auto& [topic, skeleton] = pair;
    skeleton = {};
  });
}

utils::Topic2MarkerIdsMap Aligner::getAvailableMarkerIds(
    utils::Topic2MarkerIdsMap ids) const {
  for (const auto& topic : params_.input_topics) {
    for (auto idx{static_cast<int>(ids.at(topic).size()) - 1}; idx >= 0;
         --idx) {
      if (!skeletons_map_.at(topic).hasMarker(
              static_cast<int>(ids.at(topic).at(idx)))) {
        utils::removeMarkerAtIndex(ids, idx);
      }
    }
  }

  return ids;
}

bool Aligner::computeRotation() {
  auto available_ids{getAvailableMarkerIds(params_.marker_ids.rotation)};
  if (available_ids.at(params_.input_topics.front()).size() < 3) {
    // Return if skeletons do not have enough markers
    return false;
  }

  std::map<std::string, Eigen::MatrixXd> points_map{};
  for (const auto& topic : params_.input_topics) {
    points_map[topic] = Eigen::MatrixXd(3, available_ids.at(topic).size());

    auto i{0u};
    for (const auto& mk_id : available_ids.at(topic)) {
      const auto& point{skeletons_map_.at(topic)
                            .getMarker(static_cast<int>(mk_id))
                            .center.pose.position};
      points_map.at(topic).col(i++) << point.x(), point.y(), point.z();
    }

    // Replace NaNs with zeros
    points_map.at(topic) = (!points_map.at(topic).array().isNaN())
                               .select(points_map.at(topic), 0.);
  }

  // Center matrices
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

  return true;
}

bool Aligner::computeTranslation() {
  auto available_ids{getAvailableMarkerIds(params_.marker_ids.translation)};
  if (available_ids.at(params_.input_topics.front()).empty()) {
    // Return if skeletons do not have enough markers
    return false;
  }

  const auto& parent_topic{params_.input_topics.front()};
  const auto& child_topic{params_.input_topics.back()};

  // T = [R  t2-R*t1]
  //     [0        1]
  transform_.setOrigin(
      utils::avg(utils::extractMarkers(skeletons_map_.at(parent_topic),
                                       available_ids.at(parent_topic))) -
      transform_.getBasis() *
          utils::avg(utils::extractMarkers(skeletons_map_.at(child_topic),
                                           available_ids.at(child_topic))));

  return true;
}

void Aligner::getRootTransforms() {
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

void Aligner::getRootFrames() {
  for (const auto& topic : params_.input_topics) {
    root_tfs_map_[topic].frame_id_ =
        getRootFrame(skeletons_map_.at(topic).frame);
  }
}

std::string Aligner::getRootFrame(std::string child_frame) {
  std::string parent_frame{};

  while (parent_frame.empty() || parent_frame != child_frame) {
    try {
      tf_buffer_->_getParent(child_frame, tf2::TimePointZero, parent_frame);
      if (!parent_frame.empty() && parent_frame != child_frame) {
        child_frame = parent_frame;
      }
    } catch (const tf2::TransformException&) {
      break;
    }
  }

  return parent_frame;
}

void Aligner::callback(const hiros_skeleton_msgs::msg::StampedSkeleton& msg,
                       const std::string& topic_name) {
  skeletons_map_.at(topic_name) = hiros::skeletons::utils::toStruct(msg);
  updateTransform();
}

}  // namespace hdt::skeleton
