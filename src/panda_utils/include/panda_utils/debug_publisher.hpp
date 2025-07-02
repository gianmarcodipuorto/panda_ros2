#pragma once
#include "franka/robot_state.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "panda_interfaces/msg/joint_torque_measure_stamped.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::AccelStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using panda_interfaces::msg::JointsCommand;
using panda_interfaces::msg::JointsEffort;
using panda_interfaces::msg::JointsPos;
using panda_interfaces::msg::JointTorqueMeasureStamped;
using sensor_msgs::msg::JointState;

struct debug_data {
  // Mutex to access data
  std::mutex mut;

  // Flag to indicate if the data has been published/contain data
  bool has_data;

  // Last commanded tau
  std::array<double, 7> tau_d_last;

  // Robot internal state in franka::RobotState compact struct
  franka::RobotState robot_state;

  // Gravity vector
  std::array<double, 7> gravity;

  // DLS lambda
  std::optional<double> lambda;

  // Minimum singular value
  std::optional<double> sigma_min;

  // control law y e.g. tau = B * y +  n
  std::optional<Eigen::Vector<double, 7>> y;

  // control law corrective cartesian component, before inverse jacobian
  // multiplication
  std::optional<Eigen::Vector<double, 6>> y_cartesian;

  // Desired pose
  std::optional<Pose> des_pose;

  // Desired twist
  std::optional<Twist> des_twist;

  // Desired Accel
  std::optional<Accel> des_accel;

  // Current twist
  Eigen::Vector<double, 6> current_twist;

  // Pose error (Quaternion as w, x, y, z)
  Eigen::Vector<double, 7> error_pose_vec;
};

class DebugPublisher {

public:
  debug_data &data() { return pub_data; }
  void publish(rclcpp::Time now);
  void create_pubs(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  DebugPublisher();

private:
  // Shared debug struct
  debug_data pub_data;

  // Data structure to publish
  panda_interfaces::msg::DoubleStamped double_stamped;
  panda_interfaces::msg::DoubleArrayStamped arr_stamped;
  panda_interfaces::msg::DoubleArrayStamped y_cartesian_stamped;
  JointsEffort effort_cmd;
  PoseStamped pose;
  TwistStamped twist;
  AccelStamped accel;

  // Publishers
  Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      min_singular_val_pub{};
  Publisher<PoseStamped>::SharedPtr pose_error_debug{};
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub_debug{};
  Publisher<JointsEffort>::SharedPtr gravity_contribute_debug{};
  Publisher<JointsEffort>::SharedPtr y_contribute_debug{};
  Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      y_cartesian_contribute_debug{};
  Publisher<TwistStamped>::SharedPtr velocity_error_debug{};
  Publisher<PoseStamped>::SharedPtr desired_pose_debug{};
  Publisher<TwistStamped>::SharedPtr desired_velocity_debug{};
  Publisher<AccelStamped>::SharedPtr desired_acceleration_debug{};
  Publisher<PoseStamped>::SharedPtr current_pose_debug{};
  Publisher<TwistStamped>::SharedPtr current_velocity_debug{};
  Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr lamda_dls_debug{};

  Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      manipulability_index_grad_debug{};
  Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      manipulability_index_debug{};

  Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      joint_limits_index_grad_debug{};
  Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      joint_limits_index_debug{};

  void assign_time(rclcpp::Time now);
};
