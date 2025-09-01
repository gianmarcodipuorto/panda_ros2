#include "franka/exception.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
#include <vector>

// Helper function to convert Franka's array<double, 16> to
// geometry_msgs::msg::Pose
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

class WristEstimator : public rclcpp_lifecycle::LifecycleNode {
public:
  WristEstimator() : rclcpp_lifecycle::LifecycleNode("wrist_estimation_node") {
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
        panda_interface_names::DEFAULT_TOPIC_QOS);

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
      panda_franka = std::make_unique<franka::Robot>(robot_ip);
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
        try {
          auto left_link8 = tf_buffer->lookupTransform(
              "fr3_link8", "left_wrist", tf2::TimePointZero);
          auto dist = geom_utils::distance(left_link8);
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Distance from left wrist to EE: " << dist);

        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *this->get_clock(), 1000,
              "Could not transform fr3_link8 to left wrist: %s", ex.what());
        }

        try {
          auto right_link8 = tf_buffer->lookupTransform(
              "fr3_link8", "right_wrist", tf2::TimePointZero);
          auto dist = geom_utils::distance(right_link8);
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Distance from right wrist to EE: " << dist);

        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *this->get_clock(), 1000,
              "Could not transform fr3_link8 to right wrist: %s", ex.what());
        }

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
        run_flag.store(false); // Stop the loop on robot error
      }
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "TF publish loop stopped.");
  }

  std::unique_ptr<franka::Robot> panda_franka;
  std::unique_ptr<franka::Model> panda_franka_model;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  std::atomic<bool> run_flag{false};
  std::thread tf_publish_thread;

  std::optional<std::string> wrist_contact_frame{std::nullopt};
  rclcpp::Service<panda_interfaces::srv::WristContact>::SharedPtr
      wrist_contact_server;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wrist_contact_index_pub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WristEstimator>();
  rclcpp::executors::MultiThreadedExecutor
      executor; // Use MultiThreadedExecutor
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
