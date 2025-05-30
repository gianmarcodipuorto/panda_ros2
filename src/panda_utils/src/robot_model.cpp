#include "panda_utils/robot_model.hpp"
#include <fstream>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>

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
  pinocchio::crba(model_, data_, q);
  return data_.M;
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

void RobotModel::computeAll(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &v) {
  pinocchio::computeAllTerms(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
}

const pinocchio::Model &RobotModel::getModel() const { return model_; }
const pinocchio::Data &RobotModel::getData() const { return data_; }

} // namespace my_robot
