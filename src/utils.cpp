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
    const hiros::skeletons::types::Skeleton& t_skel,
    const std::vector<long>& t_marker_ids) {
  for (const auto& mk_id : t_marker_ids) {
    if (!t_skel.hasMarker(mk_id)) {
      return false;
    }
  }
  return true;
}

std::vector<hiros::skeletons::types::Point> hiros::hdt::utils::extractMarkers(
    const hiros::skeletons::types::Skeleton& t_skel,
    const std::vector<long>& t_marker_ids) {
  std::vector<hiros::skeletons::types::Point> res{};

  for (const auto& marker_id : t_marker_ids) {
    if (t_skel.hasMarker(marker_id)) {
      res.push_back(t_skel.getMarker(marker_id).center.pose.position);
    }
  }

  return res;
}

hiros::skeletons::types::Point hiros::hdt::utils::avg(
    const std::vector<hiros::skeletons::types::Point>& t_v) {
  return std::accumulate(t_v.begin(), t_v.end(),
                         hiros::skeletons::types::Point{0., 0., 0.}) /
         t_v.size();
}

bool hiros::hdt::utils::isNaN(const tf2::Transform& t_tf) {
  return hiros::skeletons::utils::isNaN(t_tf.getOrigin()) ||
         hiros::skeletons::utils::isNaN(t_tf.getRotation());
}

void hiros::hdt::utils::transform(hiros::skeletons::types::KinematicState& t_ks,
                                  const tf2::Transform& t_tf) {
  if (isNaN(t_tf)) {
    throw std::runtime_error("NaN transform");
  }

  if (!hiros::skeletons::utils::isNaN(t_ks.pose.position)) {
    t_ks.pose.position = t_tf * t_ks.pose.position;
  }
  if (!hiros::skeletons::utils::isNaN(t_ks.pose.orientation)) {
    t_ks.pose.orientation = t_tf * t_ks.pose.orientation;
  }
}

double hiros::hdt::utils::translationDistance(const tf2::Transform& t_tf1,
                                              const tf2::Transform& t_tf2) {
  return t_tf1.getOrigin().distance(t_tf2.getOrigin());
}

double hiros::hdt::utils::rotationDistance(const tf2::Transform& t_tf1,
                                           const tf2::Transform& t_tf2) {
  return t_tf1.getRotation().angleShortestPath(t_tf2.getRotation());
}

double hiros::hdt::utils::normalizedDistance(
    const tf2::Transform& t_tf1, const tf2::Transform& t_tf2,
    const double& t_max_translation_distance,
    const double& t_max_rotation_distance) {
  auto normalized_translation_distance{std::min(
      std::max(0.,
               translationDistance(t_tf1, t_tf2) / t_max_translation_distance),
      1.)};
  auto normalized_rotation_distance{std::min(
      std::max(0., rotationDistance(t_tf1, t_tf2) / t_max_rotation_distance),
      1.)};

  return std::max(normalized_translation_distance,
                  normalized_rotation_distance);
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Marker>& t_mks,
    const tf2::Transform& t_tf) {
  for (auto& mk : t_mks) {
    transform(mk.center, t_tf);
  }
}

void hiros::hdt::utils::transform(
    std::vector<hiros::skeletons::types::Link>& t_lks,
    const tf2::Transform& t_tf) {
  for (auto& lk : t_lks) {
    transform(lk.center, t_tf);
  }
}

void hiros::hdt::utils::transform(hiros::skeletons::types::Skeleton& t_skel,
                                  const tf2::Transform& t_tf) {
  transform(t_skel.markers, t_tf);
  transform(t_skel.links, t_tf);
  t_skel.bounding_box = hiros::skeletons::utils::computeBoundingBox(t_skel);
}

tf2::Transform hiros::hdt::utils::solveWeightedLeastSquares(
    const std::vector<tf2::Transform>& t_As,
    const std::vector<tf2::Transform>& t_bs, const double& t_weight) {
  if (t_As.size() != t_bs.size() || t_As.empty()) {
    std::cerr << "Error: least squares dimension mismatch or empty vectors"
              << std::endl;
    return {};
  }

  if (t_weight <= 0.) {
    std::cerr << "Error: negative weight" << std::endl;
    return {};
  }

  Eigen::MatrixXd A{};
  Eigen::MatrixXd b{};
  A.resize(12 * static_cast<long>(t_As.size()), 12);
  b.resize(12 * static_cast<long>(t_bs.size()), 1);

  Eigen::Matrix4d A_tmp{};
  Eigen::Matrix4d b_tmp{};
  double w{};
  auto row{-1};

  for (auto i{0ul}; i < t_As.size(); ++i) {
    w = std::pow(t_weight, .5 * (t_As.size() - i));  // w = sqrt(W)

    A_tmp << Eigen::Quaterniond(
                 t_As.at(i).getRotation().w(), t_As.at(i).getRotation().x(),
                 t_As.at(i).getRotation().y(), t_As.at(i).getRotation().z())
                 .normalized()
                 .toRotationMatrix(),
        Eigen::Vector3d(t_As.at(i).getOrigin().x(), t_As.at(i).getOrigin().y(),
                        t_As.at(i).getOrigin().z()),
        0, 0, 0, 1;
    A_tmp *= w;

    b_tmp << Eigen::Quaterniond(
                 t_bs.at(i).getRotation().w(), t_bs.at(i).getRotation().x(),
                 t_bs.at(i).getRotation().y(), t_bs.at(i).getRotation().z())
                 .normalized()
                 .toRotationMatrix(),
        Eigen::Vector3d(t_bs.at(i).getOrigin().x(), t_bs.at(i).getOrigin().y(),
                        t_bs.at(i).getOrigin().z()),
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
    const std::vector<tf2::Transform>& t_tfs, const double& t_weight) {
  std::vector<tf2::Transform> identities{t_tfs.size()};
  tf2::Transform identity{};
  identity.setIdentity();
  std::fill(identities.begin(), identities.end(), identity);

  return utils::solveWeightedLeastSquares(identities, t_tfs, t_weight);
}
