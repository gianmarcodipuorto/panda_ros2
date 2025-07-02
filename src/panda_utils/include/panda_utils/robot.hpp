#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <ostream>
#include <vector>

template <int ROWS, int COLS> using Matrix = Eigen::Matrix<double, ROWS, COLS>;
using Vector6 = Eigen::Vector<double, 6>;

struct DHParameters {
  double a;
  double alpha;
  double d;
  double theta;
};

struct CLIKParameters {
  double Ts = 0.01;
  double gamma = 1;
  double speed[6] = {0, 0, 0, 0, 0, 0};
  long max_iters = 200;
  double err_tolerance = 1e-6;
};

enum JointType { REVOLUTE, PRISMATIC };

enum OrientationConfiguration { ZYZ, UNIT_QUATERNION, RPY };

template <int NUM_JOINTS> class Robot {

public:
  Robot(const DHParameters dh_params[NUM_JOINTS],
        const JointType joint_types[NUM_JOINTS],
        OrientationConfiguration config);

  OrientationConfiguration &orientation_configuration();

  void fkine(double joint_values[NUM_JOINTS],
             Eigen::Isometry3d (&transforms)[NUM_JOINTS]);

  Matrix<6, NUM_JOINTS> geometrical_jacobian(double joint_values[NUM_JOINTS]);

  Matrix<6, NUM_JOINTS>
  geometrical_jacobian(const Eigen::Isometry3d (&transforms)[NUM_JOINTS]);

  Matrix<6, NUM_JOINTS>
  geometrical_jacobian(Eigen::Vector<double, NUM_JOINTS> joint_values);

  template <int ORIENTATION_ROWS>
  Matrix<3 + ORIENTATION_ROWS, 6>
  get_jacobian_transformation_matrix(OrientationConfiguration config,
                                     const std::vector<double> &params);

  Matrix<6, NUM_JOINTS> analytical_jacobian(double joint_values[NUM_JOINTS]);

  geometry_msgs::msg::Pose
  pose(const Eigen::Isometry3d (&transforms)[NUM_JOINTS]);

  geometry_msgs::msg::Pose pose(double joint_values[NUM_JOINTS]);

  geometry_msgs::msg::Pose pose(Eigen::Vector<double, NUM_JOINTS> joint_values);

  Eigen::Vector<double, NUM_JOINTS>
  clik(const geometry_msgs::msg::Pose &final_pose,
       const double joints[NUM_JOINTS],
       const CLIKParameters &clik_params = CLIKParameters());

  double clik_one_step(const geometry_msgs::msg::Pose &final_pose,
                       Eigen::Vector<double, NUM_JOINTS> &current_joints,
                       const double ts, const double gamma,
                       const Eigen::Vector<double, 6> &speed);

private:
  DHParameters dh_params[NUM_JOINTS];
  JointType joint_types[NUM_JOINTS];
  OrientationConfiguration config;
  Eigen::Quaterniond old_quaternion;
};

// Computes the homogeneous transformation following the standard DH convention
inline Eigen::Isometry3d dh_transform(DHParameters dhparam) {
  Eigen::Matrix4d T;
  Eigen::Isometry3d iso;
  T << cos(dhparam.theta), -sin(dhparam.theta) * cos(dhparam.alpha),
      sin(dhparam.theta) * sin(dhparam.alpha), dhparam.a * cos(dhparam.theta),

      sin(dhparam.theta), cos(dhparam.theta) * cos(dhparam.alpha),
      -cos(dhparam.theta) * sin(dhparam.alpha), sin(dhparam.theta) * dhparam.a,

      0.0, sin(dhparam.alpha), cos(dhparam.alpha), dhparam.d,

      0, 0, 0, 1;
  iso = T;
  return iso;
}

template <int NUM_JOINTS>
void Robot<NUM_JOINTS>::fkine(double joint_values[NUM_JOINTS],
                              Eigen::Isometry3d (&transforms)[NUM_JOINTS]) {
  for (size_t i = 0; i < NUM_JOINTS; i++) {

    JointType joint_type = this->joint_types[i];
    DHParameters dh_param = this->dh_params[i];

    switch (joint_type) {

    case JointType::REVOLUTE: {
      dh_param.theta = joint_values[i];
      break;
    }

    case JointType::PRISMATIC: {
      dh_param.d = joint_values[i];
      break;
    }
    default:
      break;
    }
    transforms[i] = dh_transform(dh_param);
  }
}

template <int NUM_JOINTS>
Robot<NUM_JOINTS>::Robot(const DHParameters dh_params[NUM_JOINTS],
                         const JointType joint_types[NUM_JOINTS],
                         OrientationConfiguration config)
    : config(config), old_quaternion(Eigen::Quaterniond::Identity()) {
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    this->dh_params[i] = dh_params[i];
    this->joint_types[i] = joint_types[i];
  }
}

template <int NUM_JOINTS>
Matrix<6, NUM_JOINTS> Robot<NUM_JOINTS>::geometrical_jacobian(
    const Eigen::Isometry3d (&transforms)[NUM_JOINTS]) {

  // std::cout << "In function geometrical_jacobian" << std::endl;
  Eigen::Isometry3d trans_to_end = Eigen::Isometry3d::Identity();
  for (Eigen::Isometry3d isom : transforms) {
    // std::cout << "Multiplying transforms" << std::endl;
    trans_to_end = trans_to_end * isom;
  }

  // std::cout << "Declaring variables" << std::endl;
  Eigen::Isometry3d p_i_1 = Eigen::Isometry3d::Identity();
  Eigen::Vector3d z_i_1 = Eigen::Vector3d::Identity();
  Eigen::Vector3d rotational_part;
  Eigen::Vector3d positional_part;
  Matrix<6, NUM_JOINTS> jacobian = Eigen::Matrix<double, 6, NUM_JOINTS>{};

  for (size_t i = 0; i < NUM_JOINTS; i++) {

    // Calculate p_i-1 and z_i-1
    if (i != 0) {
      Eigen::Isometry3d isom = transforms[i - 1];
      p_i_1 = p_i_1 * isom;
      z_i_1 = p_i_1.rotation().col(2);
    } else {
      z_i_1 = Eigen::Vector3d(0, 0, 1);
    }

    rotational_part = z_i_1;
    positional_part =
        rotational_part.cross(trans_to_end.translation() - p_i_1.translation());

    // std::cout << "Accessing jacobian" << std::endl;

    jacobian(0, i) = positional_part[0];
    jacobian(1, i) = positional_part[1];
    jacobian(2, i) = positional_part[2];
    jacobian(3, i) = rotational_part[0];
    jacobian(4, i) = rotational_part[1];
    jacobian(5, i) = rotational_part[2];
  }
  // std::cout << "Returning jacobian" << std::endl;
  return jacobian;
}

template <int NUM_JOINTS>
Matrix<6, NUM_JOINTS>
Robot<NUM_JOINTS>::geometrical_jacobian(double joint_values[NUM_JOINTS]) {

  Eigen::Isometry3d links_transforms[NUM_JOINTS]{};
  this->fkine(joint_values, links_transforms);
  Matrix<6, NUM_JOINTS> jacobian = this->geometrical_jacobian(links_transforms);
  // std::cout << jacobian << std::endl;
  return jacobian;
}

template <int NUM_JOINTS>
template <int ORIENTATION_ROWS>
Matrix<3 + ORIENTATION_ROWS, 6>
Robot<NUM_JOINTS>::get_jacobian_transformation_matrix(
    OrientationConfiguration config, const std::vector<double> &params) {

  Eigen::Matrix<double, 3 + ORIENTATION_ROWS, 6> transformation_matrix =
      Eigen::Matrix<double, 3 + ORIENTATION_ROWS, 6>::Zero();

  switch (config) {
  case OrientationConfiguration::ZYZ: {
    if (ORIENTATION_ROWS != 3 && params.size() != 3) {
      std::cout << "ERROR GET_JACOBIAN_TRANSFORMATION_MATRIX";
      rclcpp::shutdown();
    }
    double phi = params[0];
    double theta = params[1];

    transformation_matrix(0, 0) = 1;
    transformation_matrix(1, 1) = 1;
    transformation_matrix(2, 2) = 1;

    transformation_matrix(5, 3) = 1;

    transformation_matrix(3, 4) = -sin(phi);
    transformation_matrix(4, 4) = cos(phi);

    transformation_matrix(3, 5) = cos(phi) * sin(theta);
    transformation_matrix(4, 5) = sin(phi) * sin(theta);
    transformation_matrix(5, 5) = cos(theta);
  }
  case OrientationConfiguration::RPY:
    break;
  case OrientationConfiguration::UNIT_QUATERNION: {

    if (ORIENTATION_ROWS != 3 || params.size() != 4) {
      std::cout << "ERROR GET_JACOBIAN_TRANSFORMATION_MATRIX";
      rclcpp::shutdown();
    }

    double w = params[0];
    double x = params[1];
    double y = params[2];
    double z = params[3];

    transformation_matrix(0, 0) = 1;
    transformation_matrix(1, 1) = 1;
    transformation_matrix(2, 2) = 1;

    transformation_matrix(3, 3) = 0.5 * w;
    transformation_matrix(3, 4) = 0.5 * -z;
    transformation_matrix(3, 5) = 0.5 * y;

    transformation_matrix(4, 3) = 0.5 * z;
    transformation_matrix(4, 4) = 0.5 * w;
    transformation_matrix(4, 5) = 0.5 * -x;

    transformation_matrix(5, 3) = 0.5 * -y;
    transformation_matrix(5, 4) = 0.5 * x;
    transformation_matrix(5, 5) = 0.5 * w;

    break;
  }
  default:
    break;
  }

  // std::cout << transformation_matrix << std::endl;
  return transformation_matrix;
}

template <int NUM_JOINTS>
Matrix<6, NUM_JOINTS>
Robot<NUM_JOINTS>::analytical_jacobian(double joint_values[NUM_JOINTS]) {

  // std::cout << "In function analytical_jacobian";
  Eigen::Isometry3d transforms[NUM_JOINTS]{};
  this->fkine(joint_values, transforms);
  Matrix<6, NUM_JOINTS> jacobian = this->geometrical_jacobian(transforms);
  geometry_msgs::msg::Pose pose = this->pose(transforms);

  switch (this->config) {

  case ZYZ: {
    std::vector<double> params{};

    Eigen::Quaterniond quaternion{pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z};
    quaternion.normalize();
    Eigen::Vector3d orientation_euler_zyz{
        quaternion.toRotationMatrix().eulerAngles(2, 1, 2)}; // ZYZ

    params.push_back(orientation_euler_zyz[0]);
    params.push_back(orientation_euler_zyz[1]);
    params.push_back(orientation_euler_zyz[2]);

    Matrix<3 + 3, 6> transformation_matrix =
        this->get_jacobian_transformation_matrix<3>(this->config, params);
    jacobian = transformation_matrix.inverse() * jacobian;
    break;
  }
  case UNIT_QUATERNION: {
    std::vector<double> params{};

    Eigen::Quaterniond quaternion{pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z};
    quaternion.normalize();

    params.push_back(quaternion.w());
    params.push_back(quaternion.x());
    params.push_back(quaternion.y());
    params.push_back(quaternion.z());

    Matrix<3 + 3, 6> transformation_matrix =
        this->get_jacobian_transformation_matrix<3>(this->config, params);
    jacobian = transformation_matrix.inverse() * jacobian;
    break;
  }
  case RPY:
    break;
  }

  // std::cout << jacobian << std::endl;
  return jacobian;
}

template <int NUM_JOINTS>
geometry_msgs::msg::Pose
Robot<NUM_JOINTS>::pose(const Eigen::Isometry3d (&transforms)[NUM_JOINTS]) {

  Eigen::Isometry3d trans_to_end = Eigen::Isometry3d::Identity();
  for (Eigen::Isometry3d trans : transforms) {
    trans_to_end = trans_to_end * trans;
  }
  geometry_msgs::msg::Pose pose;
  auto rotation = trans_to_end.rotation();
  Eigen::Quaterniond quat(rotation);
  quat.normalize();

  pose.position.x = trans_to_end.translation().x();
  pose.position.y = trans_to_end.translation().y();
  pose.position.z = trans_to_end.translation().z();
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  return pose;
}

template <int NUM_JOINTS>
OrientationConfiguration &Robot<NUM_JOINTS>::orientation_configuration() {
  return this->config;
}
template <int NUM_JOINTS>
Matrix<6, NUM_JOINTS> Robot<NUM_JOINTS>::geometrical_jacobian(
    Eigen::Vector<double, NUM_JOINTS> joint_values) {

  double joints[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i] = joint_values[i];
  }
  return this->geometrical_jacobian(joints);
}

template <int NUM_JOINTS>
geometry_msgs::msg::Pose
Robot<NUM_JOINTS>::pose(Eigen::Vector<double, NUM_JOINTS> joint_values) {

  double joints[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i] = joint_values[i];
  }
  return this->pose(joints);
}

inline Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond &q,
                                        const Eigen::Quaterniond &oldQ) {
  auto tmp = q.vec().transpose() * oldQ.vec();
  if (tmp < -0.01) {
    Eigen::Quaterniond out(q);
    out.vec() = -out.vec();
    out.w() = -out.w();
    return out;
  }
  return q;
}

template <int NUM_JOINTS>
geometry_msgs::msg::Pose
Robot<NUM_JOINTS>::pose(double joint_values[NUM_JOINTS]) {

  Eigen::Isometry3d links_transforms[NUM_JOINTS]{};
  this->fkine(joint_values, links_transforms);
  return this->pose(links_transforms);
}

// Returns the norm of the clik iteration
template <int NUM_JOINTS>
double Robot<NUM_JOINTS>::clik_one_step(
    const geometry_msgs::msg::Pose &final_pose,
    Eigen::Vector<double, NUM_JOINTS> &current_joints, const double ts,
    const double gamma, const Eigen::Vector<double, 6> &speed) {

  using geometry_msgs::msg::Pose;

  Eigen::Matrix<double, 6, NUM_JOINTS> jacobian{};
  Eigen::Quaterniond current_quat{};
  Eigen::Vector<double, 6> error;
  Eigen::Vector3d error_quat{};
  double err_norm;
  Eigen::Quaterniond final_quat{
      final_pose.orientation.w, final_pose.orientation.x,
      final_pose.orientation.y, final_pose.orientation.z};
  final_quat.normalize();

  // Get current infos
  Pose current_pose = this->pose(current_joints);
  std::cout << "Current pose: [" << current_pose.position.x << ", "
            << current_pose.position.y << ", " << current_pose.position.z
            << ", " << current_pose.orientation.x << ", "
            << current_pose.orientation.y << ", " << current_pose.orientation.z
            << "]" << std::endl;

  jacobian = this->geometrical_jacobian(current_joints);
  auto jacob_pinv = jacobian.completeOrthogonalDecomposition();
  current_quat.w() = current_pose.orientation.w;
  current_quat.x() = current_pose.orientation.x;
  current_quat.y() = current_pose.orientation.y;
  current_quat.z() = current_pose.orientation.z;
  current_quat.normalize();

  current_quat = quaternionContinuity(current_quat, this->old_quaternion);
  this->old_quaternion = current_quat;

  // Construct errors
  error[0] = final_pose.position.x - current_pose.position.x;
  error[1] = final_pose.position.y - current_pose.position.y;
  error[2] = final_pose.position.z - current_pose.position.z;
  error_quat = (final_quat * current_quat.inverse()).vec();
  error.block<3, 1>(3, 0) = error_quat;
  err_norm = error.norm();

  std::cout << "Error: [" << error[0] << ", " << error[1] << ", " << error[2]
            << ", " << error[3] << ", " << error[4] << ", " << error[5] << "]"
            << std::endl;
  std::cout << "Error norm: " << err_norm << std::endl << std::endl;

  Eigen::Vector<double, 6> q_dot = speed + gamma * error;

  // Run algorithm
  current_joints = current_joints + ts * jacob_pinv.solve(q_dot);

  return err_norm;
}

template <int NUM_JOINTS>
Eigen::Vector<double, NUM_JOINTS>
Robot<NUM_JOINTS>::clik(const geometry_msgs::msg::Pose &final_pose,
                        const double joints[NUM_JOINTS],
                        const CLIKParameters &clik_params) {

  using geometry_msgs::msg::Pose;
  std::cout << "Desired pose: [" << final_pose.position.x << ", "
            << final_pose.position.y << ", " << final_pose.position.z << ", "
            << final_pose.orientation.x << ", " << final_pose.orientation.y
            << ", " << final_pose.orientation.z << "]" << std::endl;

  // Get CLIK related parameters
  double ts = clik_params.Ts;
  double gamma = clik_params.gamma / ts;
  Eigen::Vector<double, 6> speed;
  long max_iters = clik_params.max_iters;
  double err_tol = clik_params.err_tolerance;
  for (size_t i = 0; i < 6; i++) {
    speed[i] = clik_params.speed[i];
  }

  std::cout << "CLIK parameters: " << "Ts=" << ts << ", gamma=" << gamma
            << ", speed=[" << speed[0] << ", " << speed[1] << ", " << speed[2]
            << ", " << speed[3] << ", " << speed[4] << ", " << speed[5] << "]"
            << std::endl;

  Eigen::Vector<double, NUM_JOINTS> current_config;
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    current_config[i] = joints[i];
  }
  Pose current_pose = this->pose(current_config);
  std::cout << "Current pose: [" << current_pose.position.x << ", "
            << current_pose.position.y << ", " << current_pose.position.z
            << ", " << current_pose.orientation.x << ", "
            << current_pose.orientation.y << ", " << current_pose.orientation.z
            << "]" << std::endl;


  double err_norm = 1e6;

  for (long i = 0; i < max_iters && err_norm > err_tol; i++) {

    err_norm =
        this->clik_one_step(final_pose, current_config, ts, gamma, speed);

  }
  return current_config;
}
