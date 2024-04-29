// Standard dependencies
#include <numeric>

// Eigen dependencies
#include "eigen3/Eigen/Eigen"

// ROS dependencies
#include "tf2_eigen/tf2_eigen.hpp"

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_aligner/utils.h"

bool hiros::hdt::utils::skeletonContains(
    const hiros::skeletons::types::Skeleton& skel,
    const std::vector<long>& marker_ids) {
  for (const auto& mk_id : marker_ids) {
    if (!skel.hasMarker(mk_id)) {
      return false;
    }
  }
  return true;
}

std::vector<hiros::skeletons::types::Point> hiros::hdt::utils::extractMarkers(
    const hiros::skeletons::types::Skeleton& skel,
    const std::vector<long>& marker_ids) {
  std::vector<hiros::skeletons::types::Point> res{};

  for (const auto& marker_id : marker_ids) {
    if (skel.hasMarker(marker_id)) {
      res.push_back(skel.getMarker(marker_id).center.pose.position);
    }
  }

  return res;
}

hiros::skeletons::types::Point hiros::hdt::utils::avg(
    const std::vector<hiros::skeletons::types::Point>& v) {
  return std::accumulate(v.begin(), v.end(),
                         hiros::skeletons::types::Point{0., 0., 0.}) /
         v.size();
}

bool hiros::hdt::utils::isNaN(const tf2::Transform& tf) {
  return hiros::skeletons::utils::isNaN(tf.getOrigin()) ||
         hiros::skeletons::utils::isNaN(tf.getRotation());
}

void hiros::hdt::utils::transform(hiros::skeletons::types::KinematicState& ks,
                                  const tf2::Transform& tf) {
  if (isNaN(tf)) {
    throw std::runtime_error("NaN transform");
  }

  if (!hiros::skeletons::utils::isNaN(ks.pose.position)) {
    ks.pose.position = tf * ks.pose.position;
  }
  if (!hiros::skeletons::utils::isNaN(ks.pose.orientation)) {
    ks.pose.orientation = tf * ks.pose.orientation;
  }
}

double hiros::hdt::utils::translationDistance(const tf2::Transform& tf1,
                                              const tf2::Transform& tf2) {
  return tf1.getOrigin().distance(tf2.getOrigin());
}

double hiros::hdt::utils::rotationDistance(const tf2::Transform& tf1,
                                           const tf2::Transform& tf2) {
  return tf1.getRotation().angleShortestPath(tf2.getRotation());
}

double hiros::hdt::utils::normalizedDistance(
    const tf2::Transform& tf1, const tf2::Transform& tf2,
    const double& max_translation_distance,
    const double& max_rotation_distance) {
  auto normalized_translation_distance{std::min(
      std::max(0., translationDistance(tf1, tf2) / max_translation_distance),
      1.)};
  auto normalized_rotation_distance{std::min(
      std::max(0., rotationDistance(tf1, tf2) / max_rotation_distance), 1.)};

  return std::max(normalized_translation_distance,
                  normalized_rotation_distance);
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Marker>& mks,
    const tf2::Transform& tf) {
  for (auto& mk : mks) {
    transform(mk.center, tf);
  }
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Link>& lks, const tf2::Transform& tf) {
  for (auto& lk : lks) {
    transform(lk.center, tf);
  }
}

void hiros::hdt::utils::transform(hiros::skeletons::types::Skeleton& skel,
                                  const tf2::Transform& tf) {
  transform(skel.markers, tf);
  transform(skel.links, tf);
  skel.bounding_box = hiros::skeletons::utils::computeBoundingBox(skel);
}

tf2::Transform hiros::hdt::utils::solveWeightedLeastSquares(
    const std::vector<tf2::Transform>& As,
    const std::vector<tf2::Transform>& bs, const double& weight) {
  if (As.size() != bs.size() || As.empty()) {
    std::cerr << "Error: least squares dimension mismatch or empty vectors"
              << std::endl;
    return {};
  }

  if (weight <= 0.) {
    std::cerr << "Error: negative weight" << std::endl;
    return {};
  }

  Eigen::MatrixXd A{};
  Eigen::MatrixXd b{};
  A.resize(12 * static_cast<long>(As.size()), 12);
  b.resize(12 * static_cast<long>(bs.size()), 1);

  Eigen::Matrix4d A_tmp{};
  Eigen::Matrix4d b_tmp{};
  double w{};
  auto row{-1};

  for (auto i{0ul}; i < As.size(); ++i) {
    w = std::pow(weight, .5 * (As.size() - i));  // w = sqrt(W)

    A_tmp << Eigen::Quaterniond(
                 As.at(i).getRotation().w(), As.at(i).getRotation().x(),
                 As.at(i).getRotation().y(), As.at(i).getRotation().z())
                 .normalized()
                 .toRotationMatrix(),
        Eigen::Vector3d(As.at(i).getOrigin().x(), As.at(i).getOrigin().y(),
                        As.at(i).getOrigin().z()),
        0, 0, 0, 1;
    A_tmp *= w;

    b_tmp << Eigen::Quaterniond(
                 bs.at(i).getRotation().w(), bs.at(i).getRotation().x(),
                 bs.at(i).getRotation().y(), bs.at(i).getRotation().z())
                 .normalized()
                 .toRotationMatrix(),
        Eigen::Vector3d(bs.at(i).getOrigin().x(), bs.at(i).getOrigin().y(),
                        bs.at(i).getOrigin().z()),
        0, 0, 0, 1;
    b_tmp *= w;

    A.row(++row) << A_tmp(0, 0), 0, 0, 0, A_tmp(0, 1), 0, 0, 0, A_tmp(0, 2), 0,
        0, 0;
    A.row(++row) << 0, A_tmp(0, 0), 0, 0, 0, A_tmp(0, 1), 0, 0, 0, A_tmp(0, 2),
        0, 0;
    A.row(++row) << 0, 0, A_tmp(0, 0), 0, 0, 0, A_tmp(0, 1), 0, 0, 0,
        A_tmp(0, 2), 0;
    A.row(++row) << 0, 0, 0, A_tmp(0, 0), 0, 0, 0, A_tmp(0, 1), 0, 0, 0,
        A_tmp(0, 2);

    A.row(++row) << A_tmp(1, 0), 0, 0, 0, A_tmp(1, 1), 0, 0, 0, A_tmp(1, 2), 0,
        0, 0;
    A.row(++row) << 0, A_tmp(1, 0), 0, 0, 0, A_tmp(1, 1), 0, 0, 0, A_tmp(1, 2),
        0, 0;
    A.row(++row) << 0, 0, A_tmp(1, 0), 0, 0, 0, A_tmp(1, 1), 0, 0, 0,
        A_tmp(1, 2), 0;
    A.row(++row) << 0, 0, 0, A_tmp(1, 0), 0, 0, 0, A_tmp(1, 1), 0, 0, 0,
        A_tmp(1, 2);

    A.row(++row) << A_tmp(2, 0), 0, 0, 0, A_tmp(2, 1), 0, 0, 0, A_tmp(2, 2), 0,
        0, 0;
    A.row(++row) << 0, A_tmp(2, 0), 0, 0, 0, A_tmp(2, 1), 0, 0, 0, A_tmp(2, 2),
        0, 0;
    A.row(++row) << 0, 0, A_tmp(2, 0), 0, 0, 0, A_tmp(2, 1), 0, 0, 0,
        A_tmp(2, 2), 0;
    A.row(++row) << 0, 0, 0, A_tmp(2, 0), 0, 0, 0, A_tmp(2, 1), 0, 0, 0,
        A_tmp(2, 2);

    row -= 12;

    b.row(++row) << b_tmp(0, 0);
    b.row(++row) << b_tmp(0, 1);
    b.row(++row) << b_tmp(0, 2);
    b.row(++row) << b_tmp(0, 3) - A_tmp(0, 3);

    b.row(++row) << b_tmp(1, 0);
    b.row(++row) << b_tmp(1, 1);
    b.row(++row) << b_tmp(1, 2);
    b.row(++row) << b_tmp(1, 3) - A_tmp(1, 3);

    b.row(++row) << b_tmp(2, 0);
    b.row(++row) << b_tmp(2, 1);
    b.row(++row) << b_tmp(2, 2);
    b.row(++row) << b_tmp(2, 3) - A_tmp(2, 3);
  }

  Eigen::MatrixXd sol_row{
      A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b)};

  Eigen::MatrixXd sol{4, 4};
  sol << sol_row(0), sol_row(1), sol_row(2), sol_row(3), sol_row(4), sol_row(5),
      sol_row(6), sol_row(7), sol_row(8), sol_row(9), sol_row(10), sol_row(11),
      0, 0, 0, 1;

  // Orthogonalize resulting rotation matrix, from
  // https://stackoverflow.com/questions/23080791/eigen-re-orthogonalization-of-rotation-matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd{
      sol.block<3, 3>(0, 0), Eigen::ComputeThinU | Eigen::ComputeThinV};
  Eigen::Matrix3d sol_rot{svd.matrixU() * svd.matrixV().transpose()};

  Eigen::Quaterniond q{sol_rot};
  Eigen::Vector3d t{sol.block<3, 1>(0, 3)};

  return tf2::Transform{hiros::skeletons::utils::toStruct(tf2::toMsg(q)),
                        hiros::skeletons::utils::toStruct(tf2::toMsg(t))};
}

tf2::Transform hiros::hdt::utils::weightedAverage(
    const std::vector<tf2::Transform>& tfs, const double& weight) {
  std::vector<tf2::Transform> identities{tfs.size()};
  tf2::Transform identity{};
  identity.setIdentity();
  std::fill(identities.begin(), identities.end(), identity);

  return utils::solveWeightedLeastSquares(identities, tfs, weight);
}
