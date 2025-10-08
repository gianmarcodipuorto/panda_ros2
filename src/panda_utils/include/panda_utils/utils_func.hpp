#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "panda_utils/constants.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <cmath>
#include <tf2/LinearMath/Transform.hpp>

namespace indexes {

inline double
joint_limit_index(const Eigen::Vector<double, 7> &current_joint_config,
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

inline Eigen::Vector<double, 7>
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

inline double manip_index(const Eigen::Matrix<double, 6, 7> &jacob) {
  return std::sqrt((jacob * jacob.transpose()).determinant());
}

inline Eigen::Vector<double, 7> manip_grad(
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

inline double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                   pow(p1.z - p2.z, 2));
}

inline double distance(geometry_msgs::msg::TransformStamped tf) {
  geometry_msgs::msg::Point p1{};
  geometry_msgs::msg::Point p2;

  p2.x = tf.transform.translation.x;
  p2.y = tf.transform.translation.y;
  p2.z = tf.transform.translation.z;

  return distance(p1, p2);
}

inline double distance(geometry_msgs::msg::TransformStamped tf1,
                       geometry_msgs::msg::TransformStamped tf2) {
  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;

  p1.x = tf1.transform.translation.x;
  p1.y = tf1.transform.translation.y;
  p1.z = tf1.transform.translation.z;

  p2.x = tf2.transform.translation.x;
  p2.y = tf2.transform.translation.y;
  p2.z = tf2.transform.translation.z;

  return distance(p1, p2);
}

std::array<double, 16> inline get_transform_matrix(
    const geometry_msgs::msg::Pose &pose_msg) {
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

inline geometry_msgs::msg::Pose
get_pose(const std::array<double, 16> &transform_mat) {
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

inline Eigen::Matrix<double, 6, 7>
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

struct  decay_laws {
  double lambda, beta, a, b, t_f;
  double q_i, q_i_dot, q_i_ddot;
  decay_laws(double t_f, double q_i, double q_i_dot, double q_i_ddot,
             double lambda = 5, double beta = 10)
      : lambda(lambda), beta(beta), t_f(t_f), q_i(q_i), q_i_dot(q_i_dot),
        q_i_ddot(q_i_ddot) {
    b = (q_i_ddot + q_i_dot * lambda) / (lambda - beta);
    a = q_i_dot - b;
  }
  double exponential_decay_velocity(double t) {
    if (t <= 0.0)
      return q_i_dot;
    if (t >= t_f)
      return 0.0;

    return a * exp(-lambda * t) + b * exp(-beta * t);
  }
  double exponential_decay_acceleration(double t) {
    if (t <= 0.0)
      return q_i_ddot;
    if (t >= t_f)
      return 0.0;

    return -lambda * a * exp(-lambda * t) - beta * b * exp(-beta * t);
  }

  double exponential_decay_position(double t) {
    if (t <= 0.0)
      return q_i;
    if (t > t_f)
      return exponential_decay_position(t_f);

    return q_i + (a / lambda) * (1 - exp(-lambda * t)) +
           (b / beta) * (1 - exp(-beta * t));
  }
};
