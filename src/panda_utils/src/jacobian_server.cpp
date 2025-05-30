// This node expects the robot description to be passed in some way to him and
// exposes a service to calculate the jaobians of panda robot

#include "chrono"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_interfaces/msg/jacob_column.hpp"
#include "panda_interfaces/msg/jacobian_matrix.hpp"
#include "panda_interfaces/msg/result.hpp"
#include "panda_interfaces/srv/calculate_jacobian.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <cstddef>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <unistd.h>
#include <vector>
#include "panda_utils/constants.hpp"

using tf2_msgs::msg::TFMessage;
using namespace std::chrono_literals;

class JacobCalc : public rclcpp::Node {
public:
  JacobCalc()
      : Node("jacob_calc"),
        links_transforms(std::make_shared<std::vector<Eigen::Isometry3d>>()) {

    const int topic_qos = 10;

    auto handle_tfs = [this](TFMessage mess) {
      RCLCPP_DEBUG(this->get_logger(), "Assigning to links transforms");

      this->links_transforms = std::make_shared<std::vector<Eigen::Isometry3d>>(
          this->transformstoEigens(mess.transforms));
    };

    const std::string tf_topic{"/tf"};
    const std::string jacob_topic{"/geom_jacob"};

    // Subscribe to tfs topic
    sub = this->create_subscription<TFMessage>(tf_topic, topic_qos, handle_tfs);
    RCLCPP_INFO(this->get_logger(),
                "Created subscription to Transforms topic: %s",
                tf_topic.c_str());

    auto calc_jacobian =
        [this](
            const std::shared_ptr<
                panda_interfaces::srv::CalculateJacobian_Request>
                request,
            std::shared_ptr<panda_interfaces::srv::CalculateJacobian_Response>
                response) {
          auto links_transforms =
              this->transformstoEigens(request->transformations.transforms);

          panda_interfaces::msg::JacobianMatrix jacob;
          this->fill_jacob_with_isoms(links_transforms, jacob);
          response->jacobian = jacob;
          response->result.state = panda_interfaces::msg::Result::SUCCESS;
        };

    service = this->create_service<panda_interfaces::srv::CalculateJacobian>(
        panda_interface_names::jacob_calc_service_name, calc_jacobian, topic_qos);
    RCLCPP_INFO(this->get_logger(), "Service calculate_jacobian ready");

    RCLCPP_INFO(this->get_logger(), "Created jacob publisher");
    jacobian_publisher =
        this->create_publisher<panda_interfaces::msg::JacobianMatrix>(
            jacob_topic, 10);

    auto calc_jacobian_publisher = [this, jacob_topic]() -> void {
      if (this->links_transforms->empty()) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Service requested but simulation not started yet");
        return;
      }
      // Calculate Isometry till last joint
      Eigen::Isometry3d trans_to_end = Eigen::Isometry3d::Identity();
      for (auto isom : *links_transforms) {
        trans_to_end = trans_to_end * isom;
      }
      RCLCPP_DEBUG(this->get_logger(), "Calculated trans_to_end");

      panda_interfaces::msg::JacobianMatrix jacob;
      this->fill_jacob_with_isoms(*links_transforms, jacob);
      this->jacobian_publisher->publish(jacob);
    };

    RCLCPP_INFO(this->get_logger(),
                "Created timer for jacob publisher on topic");
    publish_jacob_timer =
        this->create_wall_timer(10ms, calc_jacobian_publisher);
  };

private:
  std::shared_ptr<rclcpp::Service<panda_interfaces::srv::CalculateJacobian>>
      service;
  std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> sub;
  std::shared_ptr<std::vector<Eigen::Isometry3d>> links_transforms;
  rclcpp::TimerBase::SharedPtr publish_jacob_timer;
  rclcpp::Publisher<panda_interfaces::msg::JacobianMatrix>::SharedPtr
      jacobian_publisher;

  const std::vector<Eigen::Isometry3d> transformstoEigens(
      const std::vector<geometry_msgs::msg::TransformStamped> transforms);

  void
  fill_jacob_with_isoms(const std::vector<Eigen::Isometry3d> links_transforms,
                        panda_interfaces::msg::JacobianMatrix &jacob);
};

void JacobCalc::fill_jacob_with_isoms(
    const std::vector<Eigen::Isometry3d> links_transforms,
    panda_interfaces::msg::JacobianMatrix &jacob) {
  Eigen::Isometry3d trans_to_end = Eigen::Isometry3d::Identity();
  for (auto isom : links_transforms) {
    trans_to_end = trans_to_end * isom;
  }
  RCLCPP_DEBUG(this->get_logger(), "Calculated trans_to_end");

  Eigen::Isometry3d p_i_1 = Eigen::Isometry3d::Identity();
  Eigen::Vector3d z_i_1 = Eigen::Vector3d::Identity();
  Eigen::Vector3d rotational_part;
  Eigen::Vector3d positional_part;

  for (size_t i = 0; i < links_transforms.size(); i++) {

    // Calculate p_i-1 and z_i-1
    auto isom = links_transforms[i-1];

    if (i != 0) {
      p_i_1 = p_i_1 * isom;
      z_i_1 = p_i_1.rotation().col(2);
    } else {
      z_i_1 = Eigen::Vector3d(0, 0, 1);
    }

    rotational_part = z_i_1;
    positional_part =
        rotational_part.cross(trans_to_end.translation() - p_i_1.translation());

    panda_interfaces::msg::JacobColumn jacob_col;
    jacob_col.data = {positional_part[0], positional_part[1],
                      positional_part[2], rotational_part[0],
                      rotational_part[1], rotational_part[2]};

    RCLCPP_DEBUG(this->get_logger(), "Assigning values to jacobian");
    // Assign jacobian values to response message
    jacob.jacobian_data.push_back(jacob_col);
  }

  jacob.layout = panda_interfaces::msg::JacobianMatrix::COLUMN_MAJOR;
  jacob.length = links_transforms.size() * 6;

  RCLCPP_DEBUG(this->get_logger(), "Jacobian calculated");
}

const std::vector<Eigen::Isometry3d> JacobCalc::transformstoEigens(
    const std::vector<geometry_msgs::msg::TransformStamped> transforms) {

  std::vector<Eigen::Isometry3d> eigen_transforms =
      std::vector<Eigen::Isometry3d>();

  for (auto tf : transforms) {

    RCLCPP_DEBUG(this->get_logger(), "Transforming to Eigen");
    const Eigen::Isometry3d isom = tf2::transformToEigen(tf.transform);

    RCLCPP_DEBUG(this->get_logger(), "Transformed to Eigen");
    eigen_transforms.push_back(isom);

    RCLCPP_DEBUG(this->get_logger(), "Pushed to vector of isometries");
  }
  return eigen_transforms;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JacobCalc>());
  rclcpp::shutdown();
  return 0;
}
