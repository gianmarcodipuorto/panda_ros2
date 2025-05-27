#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/srv/calculate_jacobian.hpp"
#include "panda_interfaces/srv/clik.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <cstddef>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;
using Vector7 = Eigen::Vector<double, 7>;
using Vector6 = Eigen::Vector<double, 6>;
template <int ROWS, int COLS> using Matrix = Eigen::Matrix<double, ROWS, COLS>;
using namespace panda_interfaces::srv;

class CLIKServer : public rclcpp::Node {
public:
  CLIKServer()
      : Node("clik_server"),
        panda(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES, UNIT_QUATERNION) {
    // Initialize clients
    const int service_qos = 10;

    auto clik_calc = [this](const CLIK_Request::SharedPtr request,
                            CLIK_Response::SharedPtr response) {
      // Get parameters
      CLIKParameters clik_params{};
      clik_params.Ts = request->params.t_camp;
      clik_params.gamma = request->params.gamma;
      clik_params.err_tolerance = request->params.error_tolerance;
      clik_params.max_iters = request->params.iters;
      for (int i = 0; i < 6; i++) {
        clik_params.speed[i] = request->params.speed[i];
      }
      RCLCPP_DEBUG(this->get_logger(), "Got parameters");

      // Get initial and final pose
      double current_joint_config[7] = {
          request->joints.joint_values[0], request->joints.joint_values[1],
          request->joints.joint_values[2], request->joints.joint_values[3],
          request->joints.joint_values[4], request->joints.joint_values[5],
          request->joints.joint_values[6]};

      RCLCPP_INFO(this->get_logger(), "Calling CLIK");
      Vector7 final_joint_config = this->panda.clik(
          request->final_pose, current_joint_config, clik_params);

      for (size_t i = 0; i < 7; i++) {
        response->joints.joint_values[i] = final_joint_config[i];
      }
    };

    clik_service = this->create_service<CLIK>(
        panda_interface_names::clik_service_name, clik_calc, service_qos);
    RCLCPP_INFO(this->get_logger(), "Service CLIK ready");
  };

private:
  std::shared_ptr<rclcpp::Service<CLIK>> clik_service;
  Robot<7> panda;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CLIKServer>());
  rclcpp::shutdown();
  return 0;
}
