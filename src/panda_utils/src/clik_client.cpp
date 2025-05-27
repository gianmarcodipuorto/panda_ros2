#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/srv/calculate_jacobian.hpp"
#include "panda_interfaces/srv/clik.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "panda_utils/constants.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Rotation2D.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>

using namespace panda_interfaces::srv;
using namespace std::chrono_literals;

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  const int client_qos = 10;

  auto clik_node = rclcpp::Node::make_shared("clik_node");
  clik_node->declare_parameter<double>("t_camp", 0.01);
  clik_node->declare_parameter<double>("gamma", 0.1);

  clik_node->declare_parameter<double>("x", 0.5);
  clik_node->declare_parameter<double>("y", 0);
  clik_node->declare_parameter<double>("z", 0.1);

  rclcpp::Client<CLIK>::SharedPtr clik_client = clik_node->create_client<CLIK>(
      panda_interface_names::clik_service_name, client_qos);

  CLIK_Request::SharedPtr request = std::make_shared<CLIK_Request>();
  request->joints.joint_values = {0, 0, 0, 0, 0, 0, 0};
  geometry_msgs::msg::Pose final_pose;
  final_pose.position.x = clik_node->get_parameter("x").as_double();
  final_pose.position.y = clik_node->get_parameter("y").as_double();
  final_pose.position.z = clik_node->get_parameter("z").as_double();

  Eigen::Quaterniond final_quat{0.70711, 0.70711, 0, 0};
  final_quat.normalize();
  final_pose.orientation.w = final_quat.w();
  final_pose.orientation.x = final_quat.x();
  final_pose.orientation.y = final_quat.y();
  final_pose.orientation.z = final_quat.z();
  request->final_pose = final_pose;

  request->params.gamma = clik_node->get_parameter("gamma").as_double();
  request->params.t_camp = clik_node->get_parameter("t_camp").as_double();
  request->params.iters = 200;
  // request->params.speed[0] = 0.1;
  // request->params.speed[1] = 0.1;
  // request->params.speed[2] = 0.1;

  while (!clik_client->wait_for_service(1s)) {
    RCLCPP_INFO(clik_node->get_logger(), "CLIK service not ready yet");
  }
  auto result = clik_client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(clik_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(clik_node->get_logger(), "Service called");
    RCLCPP_INFO(clik_node->get_logger(), "Final joints are:");
    for (double joint : result.get()->joints.joint_values) {
      std::cout << joint << " ";
    }
  } else {
    RCLCPP_ERROR(clik_node->get_logger(), "Service aborted call");
  };
  rclcpp::shutdown();
  return 0;
}
