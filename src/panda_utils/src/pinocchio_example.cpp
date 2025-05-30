#include "panda_utils/constants.hpp"
#include "panda_utils/robot_model.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
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

    panda::RobotModel robot{urdf_path, true};

    // Neutral configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.getModel().nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.getModel().nq);

    // Compute all terms
    robot.computeAll(q, v);

    // Extract quantities
    Eigen::MatrixXd M = robot.getMassMatrix(q);    // Mass matrix
    Eigen::VectorXd g = robot.getGravityVector(q); // Gravity
    Eigen::VectorXd c =
        robot.getCoriolisCentrifugal(q, v); // Coriolis + gravity

    std::cout << "Mass matrix:\n" << M << "\n\n";
    std::cout << "Gravity vector:\n" << g.transpose() << "\n\n";
    std::cout << "Coriolis centrifugal:\n" << c.transpose() << "\n\n";
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
