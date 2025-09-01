#include "franka/exception.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/srv/wrist_contact.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

Eigen::Matrix<double, 6, 6>
get_adjoint(const geometry_msgs::msg::Transform tf) {
  Eigen::Isometry3d isom = tf2::transformToEigen(tf);
  Eigen::Matrix3d rotation_mat = isom.rotation();
  Eigen::Vector3d translation_vec = isom.translation();

  // Calculate skew matrix
  Eigen::Matrix3d skew_mat;
  // clang-format off
  skew_mat <<     0, -translation_vec.z(),  translation_vec.y(),
                  translation_vec.z(),     0, -translation_vec.x(),
                  -translation_vec.y(),  translation_vec.x(),     0;
  // clang-format on

  Eigen::Matrix<double, 6, 6> adjoint;
  adjoint.block<3, 3>(0, 0) = rotation_mat;
  adjoint.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  adjoint.block<3, 3>(3, 0) = skew_mat * rotation_mat;
  adjoint.block<3, 3>(3, 3) = rotation_mat;

  return adjoint;
}

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

Eigen::Matrix<double, 6, 7>
compute_jacob_pseudoinv_transposed(const Eigen::Matrix<double, 6, 7> &jacobian,
                                   const double &lambda) {
  return (jacobian * jacobian.transpose() +
          lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity())
             .inverse() *
         jacobian;
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
        rclcpp::Time now = this->now();

        if (wrist_contact_frame.has_value()) {
          std::array<double, 16> temp_array;
          try {
            // Getting transform from EE to Kstiffness frame
            //
            // geometry_msgs::msg::TransformStamped ee_to_wrist_tf =
            //     tf_buffer->lookupTransform(
            //         EE_frame, wrist_contact_frame.value(),
            //         tf2::TimePointZero);

            geometry_msgs::msg::TransformStamped ee_to_wrist_tf =
                tf_buffer->lookupTransform(wrist_contact_frame.value(),
                                           EE_frame, tf2::TimePointZero);

            Eigen::Matrix4d matrix_ee_to_wrist;
            matrix_ee_to_wrist =
                tf2::transformToEigen(ee_to_wrist_tf.transform).matrix();

            Eigen::Map<Eigen::Matrix4d>(temp_array.data()) = matrix_ee_to_wrist;

            // Set Stiffness frame for jacobian calculation
            panda_franka->setK(temp_array);
            franka::RobotState current_state = panda_franka->readOnce();
            current_state.EE_T_K = temp_array;

            //
            // Calculated jacobian at stiffness frame with current available
            // state
            std::array<double, 42> jacobian_arr =
                panda_franka->loadModel().zeroJacobian(
                    franka::Frame::kStiffness, current_state);

            // std::array<double, 42> jacobian_arr =
            //     panda_franka->loadModel().zeroJacobian(
            //         franka::Frame::kEndEffector, current_state);
            // auto adjoint = get_adjoint(ee_to_wrist_tf.transform);

            // Calculate external forces as (jacobian^dagger)^T tau_ext = h_e
            Eigen::Vector<double, 7> tau_ext_filt;
            for (int i = 0; i < tau_ext_filt.size(); i++) {
              tau_ext_filt[i] = current_state.tau_ext_hat_filtered[i];
            }

            Eigen::Matrix<double, 6, 7> jacobian = get_jacobian(jacobian_arr);
            Eigen::Vector<double, 6> h_e =
                compute_jacob_pseudoinv_transposed(jacobian, 1e-6) *
                tau_ext_filt;

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
            external_forces_contribute_debug->publish(arr);

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
          } catch (const franka::CommandException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "Got exception for a command: %s", ex.what());
            RCLCPP_WARN_STREAM_THROTTLE(
                get_logger(), *this->get_clock(), 1000,
                "EE to K tf: " << temp_array[0] << ", " << temp_array[1] << ", "
                               << temp_array[2] << ", " << temp_array[3] << ", "
                               << temp_array[4] << ", " << temp_array[5] << ", "
                               << temp_array[6] << ", " << temp_array[7] << ", "
                               << temp_array[8] << ", " << temp_array[9] << ", "
                               << temp_array[10] << ", " << temp_array[11]
                               << ", " << temp_array[12] << ", "
                               << temp_array[13] << ", " << temp_array[14]
                               << ", " << temp_array[15]);
          } catch (const franka::Exception &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "Got exception from franka library: %s",
                                 ex.what());
          }
        }

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Franka robot error in TF loop: " << e.what());
        // run_flag.store(false); // Stop the loop on robot error
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

class WristEstimator : public rclcpp_lifecycle::LifecycleNode {
public:
  WristEstimator(std::shared_ptr<franka::Robot> robot = nullptr)
      : rclcpp_lifecycle::LifecycleNode("wrist_estimation_node"),
        panda_franka(robot) {
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<std::vector<double>>(
        "world_base_link", std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0}); // x, y, z, w, x, y, z

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    wrist_contact_index_pub = this->create_publisher<std_msgs::msg::String>(
        panda_interface_names::wrist_contact_index_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS());

    auto wrist_contact_cb =
        [this](
            const panda_interfaces::srv::WristContact_Request::SharedPtr
                request,
            panda_interfaces::srv::WristContact_Response::SharedPtr response) {
          std_msgs::msg::String wrist;
          if (!request->contact) {
            wrist_contact_frame = std::nullopt;
            wrist.data = "";
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Currently unset wrist contact");
          } else {
            wrist_contact_frame = std::string(request->wrist.data);
            wrist.data = request->wrist.data;
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Currently set wrist in touch to "
                                   << wrist.data);
          }
          wrist_contact_index_pub->publish(wrist);
          response->set__success(true);
        };

    wrist_contact_server =
        this->create_service<panda_interfaces::srv::WristContact>(
            panda_interface_names::set_wrist_contact_service_name,
            wrist_contact_cb);
    wrist_contact_client =
        this->create_client<panda_interfaces::srv::WristContact>(
            panda_interface_names::set_wrist_contact_service_name);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    std::string robot_ip = this->get_parameter("robot_ip").as_string();
    auto world_base_link =
        this->get_parameter("world_base_link").as_double_array();
    RCLCPP_INFO_STREAM(get_logger(),
                       "Base link to world transform: "
                           << world_base_link[0] << ", " << world_base_link[1]
                           << ", " << world_base_link[2] << ", "
                           << world_base_link[3] << ", " << world_base_link[4]
                           << ", " << world_base_link[5] << ", "
                           << world_base_link[6]);
    RCLCPP_INFO_STREAM(get_logger(), "Connecting to robot at IP: " << robot_ip);
    try {
      if (!panda_franka) {
        panda_franka = std::make_shared<franka::Robot>(robot_ip);
      }
      panda_franka_model =
          std::make_unique<franka::Model>(panda_franka->loadModel());
      RCLCPP_INFO(get_logger(), "Franka robot and model initialized.");
    } catch (const franka::Exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Failed to connect to Franka robot: " << e.what());
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Activating WristEstimator.");
    std::vector<double> world_base_link_param =
        this->get_parameter("world_base_link").as_double_array();
    geometry_msgs::msg::TransformStamped world_to_base_transform_static;
    world_to_base_transform_static.header.frame_id = "world";
    world_to_base_transform_static.child_frame_id =
        "fr3_link0"; // Assuming the base link name

    world_to_base_transform_static.transform.translation.x =
        world_base_link_param[0];
    world_to_base_transform_static.transform.translation.y =
        world_base_link_param[1];
    world_to_base_transform_static.transform.translation.z =
        world_base_link_param[2];
    world_to_base_transform_static.transform.rotation.w =
        world_base_link_param[3];
    world_to_base_transform_static.transform.rotation.x =
        world_base_link_param[4];
    world_to_base_transform_static.transform.rotation.y =
        world_base_link_param[5];
    world_to_base_transform_static.transform.rotation.z =
        world_base_link_param[6];
    world_to_base_transform_static.header.stamp =
        this->now(); // Stamp it now for static broadcaster
    static_tf_broadcaster->sendTransform(world_to_base_transform_static);
    RCLCPP_INFO(get_logger(), "Published static world -> fr3_link0 transform.");
    run_flag.store(true);
    tf_publish_thread = std::thread(&WristEstimator::tf_publish_loop, this);
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
    panda_franka_model.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down WristEstimator.");
    run_flag.store(false);
    if (tf_publish_thread.joinable()) {
      tf_publish_thread.join();
    }
    panda_franka.reset();
    panda_franka_model.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void tf_publish_loop() {
    rclcpp::Rate rate(100.0); // Publish TFs at 100 Hz
    while (run_flag.load() && rclcpp::ok()) {
      try {
        franka::RobotState current_state = panda_franka->readOnce();
        rclcpp::Time now = this->now();

        // Map Franka frames to child frame names for TF publishing
        std::map<franka::Frame, std::string> franka_frame_enum_to_link_name;
        franka_frame_enum_to_link_name[franka::Frame::kJoint1] = "fr3_link1";
        franka_frame_enum_to_link_name[franka::Frame::kJoint2] = "fr3_link2";
        franka_frame_enum_to_link_name[franka::Frame::kJoint3] = "fr3_link3";
        franka_frame_enum_to_link_name[franka::Frame::kJoint4] = "fr3_link4";
        franka_frame_enum_to_link_name[franka::Frame::kJoint5] = "fr3_link5";
        franka_frame_enum_to_link_name[franka::Frame::kJoint6] = "fr3_link6";
        franka_frame_enum_to_link_name[franka::Frame::kJoint7] = "fr3_link7";
        franka_frame_enum_to_link_name[franka::Frame::kEndEffector] =
            "fr3_link8"; // Typically end-effector name

        // Store all absolute transforms (from fr3_link0)
        std::map<std::string, tf2::Transform> abs_transforms;
        abs_transforms["fr3_link0"] =
            tf2::Transform::getIdentity(); // Base frame is identity relative to
                                           // itself

        tf2::Transform prev_abs_tf =
            tf2::Transform::getIdentity(); // Represents fr3_link0_T_prev_link
        std::string prev_frame_name = "fr3_link0";

        for (auto const &[franka_enum_frame, child_link_name] :
             franka_frame_enum_to_link_name) {
          auto transform_matrix = panda_franka_model->pose(
              franka_enum_frame,
              current_state); // Gives fr3_link0_T_CurrentLink
          geometry_msgs::msg::Pose abs_pose_msg =
              convertMatrixToPose(transform_matrix);
          tf2::Transform current_abs_tf;
          tf2::fromMsg(abs_pose_msg, current_abs_tf);

          // Calculate relative transform: prev_link_T_current_link =
          // (fr3_link0_T_prev_link).inverse() * (fr3_link0_T_current_link)
          tf2::Transform relative_tf = prev_abs_tf.inverse() * current_abs_tf;

          geometry_msgs::msg::TransformStamped tf_stamped;
          tf_stamped.header.stamp = now;
          tf_stamped.header.frame_id = prev_frame_name;
          tf_stamped.child_frame_id = child_link_name;
          tf_stamped.transform = tf2::toMsg(relative_tf);
          tf_broadcaster->sendTransform(tf_stamped);

          prev_abs_tf = current_abs_tf;
          prev_frame_name = child_link_name;
        }

        // Printing value of distance between wrist and end effector
        std::optional<double> dist_left, dist_right;
        try {
          auto left_link8 = tf_buffer->lookupTransform(
              "fr3_link8", "left_wrist", tf2::TimePointZero);
          dist_left = geom_utils::distance(left_link8);
          RCLCPP_INFO_STREAM_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Distance from left wrist to EE: " << dist_left.value());

        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *this->get_clock(), 1000,
              "Could not transform fr3_link8 to left wrist: %s", ex.what());
        }

        try {
          auto right_link8 = tf_buffer->lookupTransform(
              "fr3_link8", "right_wrist", tf2::TimePointZero);
          dist_right = geom_utils::distance(right_link8);
          RCLCPP_INFO_STREAM_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Distance from right wrist to EE: " << dist_right.value());

        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *this->get_clock(), 1000,
              "Could not transform fr3_link8 to right wrist: %s", ex.what());
        }

        // Call wrist contact service
        std_msgs::msg::String wrist;
        if (dist_right.has_value() && dist_left.has_value()) {
          if (dist_right.value() < dist_left.value()) {
            // Publish dist_right
            wrist.data = "right_wrist";
          } else {
            // Publish dist_left
            wrist.data = "left_wrist";
          }
        } else if (dist_left.has_value() && !dist_right.has_value()) {
          // PUblish dist_left
          wrist.data = "left_wrist";
        } else if (!dist_left.has_value() && dist_right.has_value()) {
          // PUblish dist right
          wrist.data = "right_wrist";
        } else {
          wrist.data = "";
        }

        // Send request
        wrist_contact_index_pub->publish(wrist);

        // If a wrist contact frame is set, publish its transform relative to
        // the end effector
        if (wrist_contact_frame.has_value()) {
          try {
            // Lookup the transform from the end effector (fr3_link8) to the
            // wrist_contact_frame (e.g., "left_wrist") Assuming the
            // wrist_contact_frame is published by an external node relative to
            // "world". We need to publish T_fr3_link8_from_wrist to effectively
            // attach the wrist frame to fr3_link8. However,
            // `tf_buffer->lookupTransform(target_frame, source_frame, ...)`
            // gives T_target_from_source. So `lookupTransform("fr3_link8",
            // wrist_contact_frame.value(), ...)` gives T_fr3_link8_from_wrist.
            // This is exactly what we want to publish as a child of fr3_link8.
            geometry_msgs::msg::TransformStamped ee_to_wrist_tf =
                tf_buffer->lookupTransform("fr3_link8",
                                           wrist_contact_frame.value(),
                                           tf2::TimePointZero);

            geometry_msgs::msg::TransformStamped wrist_tf_stamped;
            wrist_tf_stamped.header.stamp = now;
            wrist_tf_stamped.header.frame_id = "fr3_link8"; // Parent
            wrist_tf_stamped.child_frame_id =
                wrist_contact_frame.value(); // Child
            wrist_tf_stamped.transform =
                ee_to_wrist_tf.transform; // Direct transform from EE to wrist

            tf_broadcaster->sendTransform(wrist_tf_stamped);
          } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "Could not transform fr3_link8 to %s: %s",
                                 wrist_contact_frame.value().c_str(),
                                 ex.what());
          }
        }

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Franka robot error in TF loop: " << e.what());
        // run_flag.store(false); // Stop the loop on robot error
      }
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "TF publish loop stopped.");
  }

  std::shared_ptr<franka::Robot> panda_franka;
  std::unique_ptr<franka::Model> panda_franka_model;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  std::atomic<bool> run_flag{false};
  std::thread tf_publish_thread;

  std::optional<std::string> wrist_contact_frame{std::nullopt};
  bool in_contact = false;
  rclcpp::Service<panda_interfaces::srv::WristContact>::SharedPtr
      wrist_contact_server;
  rclcpp::Client<panda_interfaces::srv::WristContact>::SharedPtr
      wrist_contact_client;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wrist_contact_index_pub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::string robot_ip = "10.224.20.198";
  auto robot = std::make_shared<franka::Robot>(robot_ip);
  auto node_wrist = std::make_shared<WristEstimator>(robot);
  auto node_force = std::make_shared<ExternalForceEstimator>(robot);
  rclcpp::executors::MultiThreadedExecutor
      executor; // Use MultiThreadedExecutor
  executor.add_node(node_wrist->get_node_base_interface());
  executor.add_node(node_force->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
