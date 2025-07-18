#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "panda_utils/constants.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <cmath>
#include <tf2/LinearMath/Transform.hpp>

namespace indexes {

double joint_limit_index(const Eigen::Vector<double, 7> &current_joint_config,
                         const Eigen::Vector<double, 7> &joint_min_limits =
                             panda_constants::joint_min_limits,
                         const Eigen::Vector<double, 7> &joint_max_limits =
                             panda_constants::joint_max_limits) {
  double index = 0;
  for (int i = 0; i < 7; i++) {
    index = index + 1 / (current_joint_config[i] - joint_min_limits[i]) +
            1 / (joint_max_limits[i] - current_joint_config[i]);
  }
  return index;
}

Eigen::Vector<double, 7>
joint_limit_index_grad(const Eigen::Vector<double, 7> &current_joint_config,
                       const Eigen::Vector<double, 7> &joint_min_limits =
                           panda_constants::joint_min_limits,
                       const Eigen::Vector<double, 7> &joint_max_limits =
                           panda_constants::joint_max_limits,
                       double step = 1e-3) {
  double index_0;
  double index_forwarded;
  Eigen::Vector<double, 7> limit_index_grad{};
  Eigen::Vector<double, 7> forward_joint_config{};

  // Calculating index 0
  index_0 = joint_limit_index(current_joint_config, joint_min_limits,
                              joint_max_limits);

  // Calculating gradient
  for (int i = 0; i < 7; i++) {

    forward_joint_config = current_joint_config;
    forward_joint_config[i] = forward_joint_config[i] + step;
    index_forwarded = joint_limit_index(forward_joint_config, joint_min_limits,
                                        joint_max_limits);
    limit_index_grad(i) = (index_forwarded - index_0) / step;
  }
  return limit_index_grad;
}

double manip_index(const Eigen::Matrix<double, 6, 7> &jacob) {
  return std::sqrt((jacob * jacob.transpose()).determinant());
}

Eigen::Vector<double, 7> manip_grad(
    const Eigen::Vector<double, 7> &current_joint_config,
    std::function<Eigen::Matrix<double, 6, 7>(const Eigen::Vector<double, 7> &)>
        jacob_calc,
    double step = 1e-3) {

  double index_0;
  double index_forwarded;
  Eigen::Vector<double, 7> manip_grad;
  Eigen::Matrix<double, 6, 7> jacob;

  index_0 = manip_index(jacob_calc(current_joint_config));

  for (int i = 0; i < 7; i++) {
    Eigen::Vector<double, 7> forward_joint_config = current_joint_config;
    forward_joint_config[i] = forward_joint_config[i] + step;
    index_forwarded = manip_index(jacob_calc(forward_joint_config));
    manip_grad(i) = (index_forwarded - index_0) / step;
  }
  return manip_grad;
}

} // namespace indexes

namespace geom_utils {

// Calculate the euclidean distance between 2 transforms
inline double euclidean_distance(tf2::Transform transform_msg) {
  tf2::Transform transform{};
  tf2::fromMsg(transform_msg, transform);

  return transform.getOrigin().length();
}

std::array<double, 16>
get_transform_matrix(const geometry_msgs::msg::Pose &pose_msg) {
  std::array<double, 16> transform_mat;

  // Convert quaternion to rotation matrix
  Eigen::Quaterniond q(pose_msg.orientation.w, pose_msg.orientation.x,
                       pose_msg.orientation.y, pose_msg.orientation.z);
  q.normalize();

  Eigen::Matrix3d rotation = q.toRotationMatrix();

  // Create 4x4 transformation matrix
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  mat.block<3, 3>(0, 0) = rotation;
  mat(0, 3) = pose_msg.position.x;
  mat(1, 3) = pose_msg.position.y;
  mat(2, 3) = pose_msg.position.z;

  // Map to std::array in **column-major** layout
  Eigen::Map<Eigen::Matrix4d>(transform_mat.data()) = mat;

  return transform_mat;
}

geometry_msgs::msg::Pose get_pose(const std::array<double, 16> &transform_mat) {
  Eigen::Map<const Eigen::Matrix4d> eigen_matrix(transform_mat.data());

  geometry_msgs::msg::Pose pose_msg;

  pose_msg.position.x = eigen_matrix(0, 3); // Tx
  pose_msg.position.y = eigen_matrix(1, 3); // Ty
  pose_msg.position.z = eigen_matrix(2, 3); // Tz

  Eigen::Matrix3d rotation_matrix = eigen_matrix.block<3, 3>(0, 0);

  Eigen::Quaterniond quaternion(rotation_matrix);
  quaternion.normalize();

  pose_msg.orientation.x = quaternion.x();
  pose_msg.orientation.y = quaternion.y();
  pose_msg.orientation.z = quaternion.z();
  pose_msg.orientation.w = quaternion.w();

  return pose_msg;
}

Eigen::Matrix<double, 6, 7>
get_jacobian(const std::array<double, 42> &jacob_raw) {

  Eigen::Matrix<double, 6, 7> jacobian;
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < 6; j++) {
      jacobian(j, i) = jacob_raw[i * 7 + j];
    }
  }
  return jacobian;
}

} // namespace geom_utils
