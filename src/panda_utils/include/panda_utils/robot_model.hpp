#pragma once

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

namespace panda {

using namespace pinocchio;

class RobotModel {
public:
  RobotModel(const std::string &urdf_param_or_path,
             bool is_package_path = true);

  Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd &q);
  Eigen::VectorXd getGravityVector(const Eigen::VectorXd &q);
  Eigen::VectorXd getCoriolisCentrifugal(const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &v);
  Eigen::VectorXd getNonLinearEffects(const Eigen::VectorXd &q,
                                      const Eigen::VectorXd &v);

  void computeForwardKinematics(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &v = Eigen::VectorXd());
  pinocchio::SE3 getFramePose(const std::string &frame_name);
  void computeAll(const Eigen::VectorXd &q, const Eigen::VectorXd &v);

  const pinocchio::Model &getModel() const;
  const pinocchio::Data &getData() const;

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
};

} // namespace my_robot
