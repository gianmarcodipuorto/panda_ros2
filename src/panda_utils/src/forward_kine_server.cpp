#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <cstddef>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/event_handler.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <unistd.h>
#include <vector>

geometry_msgs::msg::TransformStamped create_tf_stamped(Eigen::Isometry3d tf,
                                                       rclcpp::Time now) {

  geometry_msgs::msg::TransformStamped tf_stamped;

  geometry_msgs::msg::Transform transform;

  tf_stamped.header.stamp = now;
  transform.translation.x = tf.translation().x();
  transform.translation.y = tf.translation().y();
  transform.translation.z = tf.translation().z();

  Eigen::Quaterniond quat{tf.rotation()};
  quat.normalize();

  transform.rotation.w = quat.w();
  transform.rotation.x = quat.x();
  transform.rotation.y = quat.y();
  transform.rotation.z = quat.z();
  tf_stamped.transform = transform;

  return tf_stamped;
}

class ForwardKineServer : public rclcpp::Node {

public:
  ForwardKineServer()
      : Node("forward_kine_server"),
        panda(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
              OrientationConfiguration::UNIT_QUATERNION) {
    const int service_qos{10};

    auto calculate_forward_kine =
        [this](const std::shared_ptr<panda_interfaces::srv::ForwardKine_Request>
                   request,
               std::shared_ptr<panda_interfaces::srv::ForwardKine_Response>
                   response) {
          // Get joint values and time as of now
          auto req_joints = request->joints.joint_values;
          double joints[7]{};
          for (size_t i = 0; i < req_joints.size(); i++) {
            joints[i] = req_joints[i];
          }
          auto now = this->get_clock()->now();
          RCLCPP_DEBUG(this->get_logger(), "Got joint values");

          Eigen::Isometry3d transforms[7]{};
          this->panda.fkine(joints, transforms);
          this->panda.analytical_jacobian(joints);

          // Construct transforms message
          for (size_t i = 0; i < 7; i++) {
            response->tfs.transforms.push_back(
                create_tf_stamped(transforms[i], now));
          }
          RCLCPP_DEBUG(this->get_logger(), "Filled transforms");

          return response;
        };
    service = this->create_service<panda_interfaces::srv::ForwardKine>(
        panda_interface_names::forward_kine_service_name,
        calculate_forward_kine, service_qos);
    RCLCPP_INFO_ONCE(this->get_logger(), "Created service %s",
                     panda_interface_names::forward_kine_service_name.c_str());
  }

private:
  rclcpp::Service<panda_interfaces::srv::ForwardKine>::SharedPtr service;
  Robot<7> panda;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardKineServer>());
  rclcpp::shutdown();
  return 0;
}
