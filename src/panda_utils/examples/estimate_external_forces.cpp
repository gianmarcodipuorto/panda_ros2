#include "franka/exception.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/srv/wrist_contact.hpp"
#include "panda_utils/constants.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <franka/model.h>
#include <franka/robot.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <optional>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>

geometry_msgs::msg::Pose
convertMatrixToPose(const std::array<double, 16> &tf_matrix) {
  Eigen::Map<const Eigen::Matrix4d> eigen_matrix(tf_matrix.data());
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = eigen_matrix(0, 3);
  pose_msg.position.y = eigen_matrix(1, 3);
  pose_msg.position.z = eigen_matrix(2, 3);
  Eigen::Matrix3d rotation_matrix = eigen_matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(rotation_matrix);
  quaternion.normalize();
  pose_msg.orientation.x = quaternion.x();
  pose_msg.orientation.y = quaternion.y();
  pose_msg.orientation.z = quaternion.z();
  pose_msg.orientation.w = quaternion.w();
  return pose_msg;
}

Eigen::Matrix<double, 6, 7>
get_jacobian(const std::array<double, 42> &jacob_raw) {

  Eigen::Matrix<double, 6, 7> jacobian;
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < 6; j++) {
      jacobian(j, i) = jacob_raw[i * 7 + j];
    }
  }
  return jacobian;
}

Eigen::Matrix<double, 7, 6>
compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian,
                        const double &lambda) {
  return jacobian.transpose() *
         (jacobian * jacobian.transpose() +
          lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity())
             .inverse();
}

class ExternalForceEstimator : public rclcpp_lifecycle::LifecycleNode {
public:
  ExternalForceEstimator(std::shared_ptr<franka::Robot> robot = nullptr)

      : rclcpp_lifecycle::LifecycleNode("wrist_estimation_node"),
        panda_franka(robot) {
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);

    wrist_contact_index_pub = this->create_publisher<std_msgs::msg::String>(
        panda_interface_names::wrist_contact_index_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS());

    auto wrist_contact_index_cb = [this](const std_msgs::msg::String msg) {
      if (!msg.data.empty()) {
        wrist_contact_frame = msg.data;
      } else {
        wrist_contact_frame = std::nullopt;
      }
    };

    wrist_contact_index_sub = this->create_subscription<std_msgs::msg::String>(
        panda_interface_names::wrist_contact_index_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS(), wrist_contact_index_cb);

    tau_external_contribute_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/cmd/tau_ext_contribute",
            panda_interface_names::DEFAULT_TOPIC_QOS());

    external_forces_contribute_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/cmd/external_forces_contribute",
            panda_interface_names::DEFAULT_TOPIC_QOS());

    external_forces_contribute_calculated_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/cmd/external_forces_contribute_calculated",
            panda_interface_names::DEFAULT_TOPIC_QOS());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    std::string robot_ip = this->get_parameter("robot_ip").as_string();

    RCLCPP_INFO_STREAM(get_logger(), "Connecting to robot at IP: " << robot_ip);
    double load = 0.553455;
    std::array F_x_Cload{-0.010328, 0.000068, 0.148159};
    std::array load_inertia{0.02001,        0.000006527121, -0.0004590,
                            0.000006527121, 0.01936,        0.000003371038,
                            -0.0004590,     0.000003371038, 0.002245};
    try {
      if (!panda_franka) {
        panda_franka = std::make_shared<franka::Robot>(robot_ip);
      }
      panda_franka->setLoad(load, F_x_Cload, load_inertia);
      RCLCPP_INFO(get_logger(), "Franka robot initialized.");
    } catch (const franka::Exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Failed to connect to Franka robot: " << e.what());
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Activating WristEstimator.");

    run_flag.store(true);
    tf_publish_thread =
        std::thread(&ExternalForceEstimator::external_force_publish, this);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating WristEstimator.");
    run_flag.store(false);
    if (tf_publish_thread.joinable()) {
      tf_publish_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up WristEstimator.");
    run_flag.store(false);
    if (tf_publish_thread.joinable()) {
      tf_publish_thread.join();
    }
    // Release Franka resources
    panda_franka.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down WristEstimator.");
    run_flag.store(false);
    if (tf_publish_thread.joinable()) {
      tf_publish_thread.join();
    }
    panda_franka.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void external_force_publish() {
    rclcpp::Rate rate(100.0);
    while (run_flag.load() && rclcpp::ok()) {
      try {
        franka::RobotState current_state = panda_franka->readOnce();
        rclcpp::Time now = this->now();

        if (wrist_contact_frame.has_value()) {
          try {
            // Getting transform from EE to Kstiffness frame
            geometry_msgs::msg::TransformStamped ee_to_wrist_tf =
                tf_buffer->lookupTransform(
                    EE_frame, wrist_contact_frame.value(), tf2::TimePointZero);

            Eigen::Matrix4d matrix_ee_to_wrist;
            matrix_ee_to_wrist =
                tf2::transformToEigen(ee_to_wrist_tf.transform).matrix();

            std::array<double, 16> temp_array;
            Eigen::Map<Eigen::Matrix4d>(temp_array.data()) = matrix_ee_to_wrist;
            // Set Stiffness frame for jacobian calculation
            panda_franka->setK(temp_array);

            // Calculated jacobian at stiffness frame with current available
            // state
            std::array<double, 42> jacobian_arr =
                panda_franka->loadModel().zeroJacobian(
                    franka::Frame::kStiffness, current_state);

            // Calculate external forces as (jacobian^dagger)^T tau_ext = h_e

            Eigen::Vector<double, 7> tau_ext_filt;
            for (int i = 0; i < tau_ext_filt.size(); i++) {
              tau_ext_filt[i] = current_state.tau_ext_hat_filtered[i];
            }

            Eigen::Matrix<double, 6, 7> jacobian = get_jacobian(jacobian_arr);
            Eigen::Vector<double, 6> h_e =
                compute_jacob_pseudoinv(jacobian, 0.0).transpose() *
                tau_ext_filt;

            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 500.0,
                "External forces on stiffness frame: "
                    << h_e[0] << ", " << h_e[1] << ", " << h_e[2] << ", "
                    << h_e[3] << ", " << h_e[4] << ", " << h_e[5]);
            // Publishing external forces and external tau
            panda_interfaces::msg::DoubleArrayStamped arr;
            arr.header.stamp = this->now();

            // Publishing tau
            arr.data.resize(tau_ext_filt.size());
            for (int i = 0; i < tau_ext_filt.size(); i++) {
              arr.data[i] = tau_ext_filt[i];
            }
            tau_external_contribute_debug->publish(arr);

            // Publishing external forces calculated
            arr.data.resize(current_state.O_F_ext_hat_K.size());
            for (size_t i = 0; i < current_state.O_F_ext_hat_K.size(); i++) {
              arr.data[i] = current_state.O_F_ext_hat_K[i];
            }
            external_forces_contribute_calculated_debug->publish(arr);

            // Publishing external forces
            arr.data.resize(h_e.size());
            for (int i = 0; i < h_e.size(); i++) {
              arr.data[i] = h_e[i];
            }
            external_forces_contribute_calculated_debug->publish(arr);

          } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "Could not transform fr3_link8 to %s: %s",
                                 wrist_contact_frame.value().c_str(),
                                 ex.what());
          } catch (const franka::Exception &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "Got exception from franka library: %s",
                                 ex.what());
          }
        }

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Franka robot error in TF loop: " << e.what());
        run_flag.store(false); // Stop the loop on robot error
      }
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "TF publish loop stopped.");
  }

  std::shared_ptr<franka::Robot> panda_franka;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  std::atomic<bool> run_flag{false};
  std::thread tf_publish_thread;
  std::string EE_frame = "fr3_link8";

  std::optional<std::string> wrist_contact_frame{std::nullopt};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      wrist_contact_index_sub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wrist_contact_index_pub;
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      tau_external_contribute_debug{};
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      external_forces_contribute_calculated_debug{};
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      external_forces_contribute_debug{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExternalForceEstimator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
