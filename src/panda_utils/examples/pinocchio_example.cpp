#include "algorithm/frames.hpp"
#include "multibody/fwd.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ostream>
#include <rclcpp/rclcpp.hpp>

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>

class PandaDynamicsNode : public rclcpp::Node {
public:
  PandaDynamicsNode() : Node("panda_dynamics_node") {
    using namespace pinocchio;

    // Locate the URDF
    std::string urdf_path =
        ament_index_cpp::get_package_share_directory("panda_world") +
        panda_constants::panda_model_effort;

    Robot<7> panda_mine{PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
                        UNIT_QUATERNION};

    panda::RobotModel robot{urdf_path, true};

    // Neutral configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.getModel().nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.getModel().nq);

    // Compute all terms
    robot.computeAll(q, v);

    Matrix<6, 7> jacobian = panda_mine.geometrical_jacobian(q);
    std::cout << "My geometrical jacobian: " << std::endl << std::endl;
    for (size_t i = 0; i < 6; i++) {
      std::cout << "[ " << jacobian.row(i) << "]" << std::endl;
    }

    std::cout << std::endl << std::endl;
    jacobian = robot.getGeometricalJacobian("fr3_link8");
    std::cout << "fr3_link8: " << std::endl;
    for (size_t i = 0; i < 6; i++) {
      std::cout << "[ " << jacobian.row(i) << "]" << std::endl;
    }

    std::cout << std::endl << std::endl;
    jacobian = robot.getGeometricalJacobian("fr3_joint8");
    std::cout << "fr3_joint8: " << std::endl;
    for (size_t i = 0; i < 6; i++) {
      std::cout << "[ " << jacobian.row(i) << "]" << std::endl;
    }
    // for (size_t i = 0; i < robot.getModel().frames.size(); i++) {
    //   jacobian.setZero();
    //   pinocchio::getFrameJacobian(
    //       robot.getModel(), robot.getData(),
    //       robot.getModel().getFrameId(robot.getModel().frames[i].name),
    //       pinocchio::LOCAL, jacobian);
    //
    //   std::cout << "Frame: " << robot.getModel().frames[i].name << std::endl;
    //   for (size_t j = 0; j < 6; j++) {
    //   std::cout << "[ " << jacobian.row(j) << "]" << std::endl;
    //   }
    // }

    // // Extract quantities
    // Eigen::MatrixXd M = robot.getMassMatrix(q);    // Mass matrix
    // Eigen::VectorXd g = robot.getGravityVector(q); // Gravity
    // Eigen::VectorXd c =
    //     robot.getCoriolisCentrifugal(q, v); // Coriolis + gravity
    //
    // std::cout << "Mass matrix:\n" << M << "\n\n";
    // std::cout << "Gravity vector:\n" << g.transpose() << "\n\n";
    // std::cout << "Coriolis centrifugal:\n" << c.transpose() << "\n\n";
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
