#include "panda_utils/robot_model.hpp"
#include "algorithm/frames.hpp"
#include "algorithm/jacobian.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "spatial/fwd.hpp"
#include <fstream>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

namespace panda {

RobotModel::RobotModel(const std::string &urdf_param_or_path,
                       bool is_package_path)
    : model_(), data_(model_) {
  std::string urdf_xml;

  if (is_package_path) {
    std::ifstream urdf_file(urdf_param_or_path);
    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    urdf_xml = buffer.str();
  } else {
    urdf_xml = urdf_param_or_path; // direct URDF string
  }

  pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  data_ = pinocchio::Data(model_);
}

Eigen::MatrixXd RobotModel::getMassMatrix(const Eigen::VectorXd &q) {
  return pinocchio::crba(model_, data_, q);
}

Eigen::VectorXd RobotModel::getGravityVector(const Eigen::VectorXd &q) {
  return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

Eigen::VectorXd RobotModel::getNonLinearEffects(const Eigen::VectorXd &q,
                                                const Eigen::VectorXd &v) {
  return pinocchio::nonLinearEffects(model_, data_, q, v);
}

Eigen::VectorXd RobotModel::getCoriolisCentrifugal(const Eigen::VectorXd &q,
                                                   const Eigen::VectorXd &v) {
  return getNonLinearEffects(q, v) - getGravityVector(q);
}

Eigen::VectorXd
RobotModel::computeHessianTimesQDot(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &q_dot,
                                    const std::string &frame_id) {
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, q_dot);

  Eigen::MatrixXd Jdot(6, model_.nv);
  pinocchio::getFrameJacobianTimeVariation(model_, data_, model_.getFrameId(frame_id),
                                           pinocchio::LOCAL, Jdot);

  return Jdot * q_dot;
}

Eigen::MatrixXd
RobotModel::computeAnalyticalJacobian(const Eigen::VectorXd &q,
                                      const pinocchio::FrameIndex &frame_id) {
  // 1. Forward kinematics
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  // 2. Geometric Jacobian
  Eigen::MatrixXd J_geo(6, model_.nv);
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, J_geo);

  // 3. Rotation matrix
  const Eigen::Matrix3d &R = data_.oMf[frame_id].rotation();

  // 4. Compute roll-pitch-yaw angles
  Eigen::Vector3d rpy = R.eulerAngles(2, 1, 0).reverse(); // ZYX → XYZ

  // 5. Compute T_inv (angular velocity to RPY rate)
  double roll = rpy(0), pitch = rpy(1);

  Eigen::Matrix3d T_inv;
  T_inv << 1, std::sin(roll) * std::tan(pitch),
      std::cos(roll) * std::tan(pitch), 0, std::cos(roll), -std::sin(roll), 0,
      std::sin(roll) / std::cos(pitch), std::cos(roll) / std::cos(pitch);

  // 6. Compose analytical Jacobian
  Eigen::MatrixXd J_ana(6, model_.nv);
  J_ana.topRows(3) = J_geo.bottomRows(3);         // Linear part unchanged
  J_ana.bottomRows(3) = T_inv * J_geo.topRows(3); // Transform angular velocity

  return J_ana;
}

void RobotModel::computeForwardKinematics(const Eigen::VectorXd &q,
                                          const Eigen::VectorXd &v) {
  if (v.size() == model_.nv)
    pinocchio::forwardKinematics(model_, data_, q, v);
  else
    pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

pinocchio::SE3 RobotModel::getFramePose(const std::string &frame_name) {
  pinocchio::FrameIndex id = model_.getFrameId(frame_name);
  return data_.oMf[id];
}

geometry_msgs::msg::Pose RobotModel::getPose(const std::string &frame_name) {
  pinocchio::SE3 se3_pose = getFramePose(frame_name);
  geometry_msgs::msg::Pose pose;
  pose.position.x = se3_pose.translation()(0);
  pose.position.y = se3_pose.translation()(1);
  pose.position.z = se3_pose.translation()(2);

  Eigen::Matrix3d R = se3_pose.rotation();
  tf2::Matrix3x3 tf_rot(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
                        R(2, 0), R(2, 1), R(2, 2));

  tf2::Quaternion q;
  tf_rot.getRotation(q);
  q.normalize();

  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  return pose;
}

pinocchio::SE3
RobotModel::getFramePoseInBase(const std::string &frame_name,
                               const std::string &base_joint_name) {
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
  pinocchio::JointIndex base_joint_id = model_.getJointId(base_joint_name);

  const pinocchio::SE3 &T_world_to_frame = data_.oMf[frame_id];
  const pinocchio::SE3 &T_world_to_base = data_.oMi[base_joint_id];

  // Pose del frame rispetto alla base
  return T_world_to_base.inverse() * T_world_to_frame;
}

void RobotModel::computeAll(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &v) {
  pinocchio::computeAllTerms(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, v);
}
Eigen::MatrixXd RobotModel::getHessian(const std::string &frame_name) {
  const pinocchio::FrameIndex id = model_.getFrameId(frame_name);
  Eigen::MatrixXd Jdot(6, model_.nv);
  pinocchio::getFrameJacobianTimeVariation(model_, data_, id, pinocchio::LOCAL,
                                           Jdot);
  return Jdot;
}

Eigen::MatrixXd
RobotModel::getGeometricalJacobian(const std::string &frame_name) {
  const pinocchio::FrameIndex id = model_.getFrameId(frame_name);
  Eigen::MatrixXd Jdot(6, model_.nv);
  pinocchio::getFrameJacobian(model_, data_, id, pinocchio::LOCAL, Jdot);
  return Jdot;
}

const pinocchio::Model &RobotModel::getModel() const { return model_; }
const pinocchio::Data &RobotModel::getData() const { return data_; }

} // namespace panda
