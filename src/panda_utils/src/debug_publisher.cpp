#include "panda_utils/debug_publisher.hpp"
#include "panda_utils/utils_func.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

DebugPublisher::DebugPublisher() {
  y_cartesian_stamped.data.resize(6);
  arr_stamped.data.resize(7);
}

void DebugPublisher::create_pubs(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp::QoS qos) {

  robot_joint_efforts_pub_debug =
      node->create_publisher<JointsEffort>("debug/cmd/effort_no_gravity", qos);

  gravity_contribute_debug =
      node->create_publisher<JointsEffort>("debug/cmd/gravity", qos);

  pose_error_debug = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "debug/error/pose", qos);

  velocity_error_debug =
      node->create_publisher<TwistStamped>("debug/error/velocity", qos);

  desired_pose_debug =
      node->create_publisher<PoseStamped>("debug/desired_pose", qos);

  desired_velocity_debug =
      node->create_publisher<TwistStamped>("debug/desired_velocity", qos);

  desired_acceleration_debug =
      node->create_publisher<AccelStamped>("debug/desired_acceleration", qos);

  current_pose_debug =
      node->create_publisher<PoseStamped>("debug/current_pose", qos);

  current_velocity_debug =
      node->create_publisher<TwistStamped>("debug/current_velocity", qos);

  current_jdot_qdot_debug =
      node->create_publisher<TwistStamped>("debug/current_jdot_qdot", qos);

  y_contribute_debug =
      node->create_publisher<JointsEffort>("debug/cmd/y_contribute", qos);

  y_cartesian_contribute_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/cmd/y_cartesian_contribute", qos);

  tau_external_contribute_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/cmd/tau_ext_contribute", qos);

  coriolis_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/coriolis", qos);

  external_forces_contribute_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/cmd/external_forces_contribute", qos);

  lamda_dls_debug =
      node->create_publisher<panda_interfaces::msg::DoubleStamped>(
          "debug/lambda", qos);

  manipulability_index_debug =
      node->create_publisher<panda_interfaces::msg::DoubleStamped>(
          "debug/manipulability_index", qos);

  manipulability_index_grad_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/manipulability_index_grad", qos);

  joint_limits_index_debug =
      node->create_publisher<panda_interfaces::msg::DoubleStamped>(
          "debug/joint_limits_index", qos);

  joint_limits_index_grad_debug =
      node->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
          "debug/joint_limits_index_grad", qos);

  min_singular_val_pub =
      node->create_publisher<panda_interfaces::msg::DoubleStamped>(
          panda_interface_names::min_singular_value_topic_name, qos);
}

void DebugPublisher::assign_time(rclcpp::Time now) {
  double_stamped.header.stamp = now;
  arr_stamped.header.stamp = now;
  y_cartesian_stamped.header.stamp = now;
  effort_cmd.header.stamp = now;
  pose.header.stamp = now;
  twist.header.stamp = now;
  accel.header.stamp = now;
}

void DebugPublisher::publish(rclcpp::Time now) {

  if (!pub_data.mut.try_lock()) {
    return;
  }
  if (!pub_data.has_data) {
    return;
  }

  try {

    assign_time(now);

    pose.pose = geom_utils::get_pose(pub_data.robot_state.O_T_EE);
    current_pose_debug->publish(pose);

    for (int i = 0; i < 7; i++) {
      effort_cmd.effort_values[i] = pub_data.tau_d_last[i];
    }
    robot_joint_efforts_pub_debug->publish(effort_cmd);

    for (int i = 0; i < 7; i++) {
      effort_cmd.effort_values[i] = pub_data.gravity[i];
    }
    gravity_contribute_debug->publish(effort_cmd);

    twist.twist.linear.x = pub_data.current_twist[0];
    twist.twist.linear.y = pub_data.current_twist[1];
    twist.twist.linear.z = pub_data.current_twist[2];

    twist.twist.angular.x = pub_data.current_twist[3];
    twist.twist.angular.y = pub_data.current_twist[4];
    twist.twist.angular.z = pub_data.current_twist[5];

    current_velocity_debug->publish(twist);

    if (pub_data.current_j_dot_q_dot.has_value()) {

      twist.twist.linear.x = pub_data.current_j_dot_q_dot.value()[0];
      twist.twist.linear.y = pub_data.current_j_dot_q_dot.value()[1];
      twist.twist.linear.z = pub_data.current_j_dot_q_dot.value()[2];

      twist.twist.angular.x = pub_data.current_j_dot_q_dot.value()[3];
      twist.twist.angular.y = pub_data.current_j_dot_q_dot.value()[4];
      twist.twist.angular.z = pub_data.current_j_dot_q_dot.value()[5];

      current_jdot_qdot_debug->publish(twist);
    }

    // Publish error on pose
    pose.pose.position.x = pub_data.error_pose_vec(0);
    pose.pose.position.y = pub_data.error_pose_vec(1);
    pose.pose.position.z = pub_data.error_pose_vec(2);
    pose.pose.orientation.w = pub_data.error_pose_vec(3);
    pose.pose.orientation.x = pub_data.error_pose_vec(4);
    pose.pose.orientation.y = pub_data.error_pose_vec(5);
    pose.pose.orientation.z = pub_data.error_pose_vec(6);
    pose_error_debug->publish(pose);

    // DLS LAMBDA
    if (pub_data.lambda.has_value()) {
      double_stamped.data = pub_data.lambda.value();
      lamda_dls_debug->publish(double_stamped);
    }

    // YOSHIKAWA INDEX
    // double_stamped.data = manip_index(jacobian);
    // manipulability_index_debug->publish(double_stamped);

    // Eigen::Vector<double, 7> manip_ind_gradient =
    //     manip_grad(current_joints_config_vec);
    // for (int i = 0; i < 7; i++) {
    //   arr_stamped.data[i] = manip_ind_gradient[i];
    // }
    // manipulability_index_grad_debug->publish(arr_stamped);

    // JOINT LIMIT INDEX
    Eigen::Vector<double, 7> current_joints_config_vec;
    for (int i = 0; i < current_joints_config_vec.size(); i++) {
      current_joints_config_vec[i] = pub_data.robot_state.q[i];
    }
    double_stamped.data = indexes::joint_limit_index(current_joints_config_vec);
    joint_limits_index_debug->publish(double_stamped);

    Eigen::Vector<double, 7> joint_limit_index_grad_vec =
        indexes::joint_limit_index_grad(current_joints_config_vec);
    for (int i = 0; i < 7; i++) {
      arr_stamped.data[i] = joint_limit_index_grad_vec[i];
    }
    joint_limits_index_grad_debug->publish(arr_stamped);

    if (pub_data.y.has_value()) {

      for (int i = 0; i < 7; i++) {
        effort_cmd.effort_values[i] = pub_data.y.value()[i];
      }
      y_contribute_debug->publish(effort_cmd);
    }

    if (pub_data.y_cartesian.has_value()) {
      for (size_t i = 0; i < 6; i++) {
        y_cartesian_stamped.data[i] = pub_data.y_cartesian.value()[i];
      }
      y_cartesian_contribute_debug->publish(y_cartesian_stamped);
    }

    if (pub_data.h_e.has_value()) {
      for (size_t i = 0; i < 6; i++) {
        y_cartesian_stamped.data[i] = pub_data.h_e.value()[i];
      }
      external_forces_contribute_debug->publish(y_cartesian_stamped);
    }

    if (pub_data.tau_ext.has_value()) {
      for (size_t i = 0; i < 7; i++) {
        arr_stamped.data[i] = pub_data.tau_ext.value()[i];
      }
      tau_external_contribute_debug->publish(arr_stamped);
    }

    if (pub_data.coriolis.has_value()) {
      for (size_t i = 0; i < 7; i++) {
        arr_stamped.data[i] = pub_data.coriolis.value()[i];
      }
      coriolis_debug->publish(arr_stamped);
    }

    if (pub_data.des_pose.has_value()) {
      pose.pose = pub_data.des_pose.value();
      desired_pose_debug->publish(pose);
    }

    if (pub_data.des_twist.has_value()) {

      twist.twist = pub_data.des_twist.value();
      desired_velocity_debug->publish(twist);
    }

    if (pub_data.des_accel.has_value()) {

      accel.accel = pub_data.des_accel.value();
      desired_acceleration_debug->publish(accel);

      // Twist error
      twist.twist.linear.x =
          pub_data.des_twist.value().linear.x - pub_data.current_twist[0];
      twist.twist.linear.y =
          pub_data.des_twist.value().linear.y - pub_data.current_twist[1];
      twist.twist.linear.z =
          pub_data.des_twist.value().linear.z - pub_data.current_twist[2];

      twist.twist.angular.x =
          pub_data.des_twist.value().angular.x - pub_data.current_twist[3];
      twist.twist.angular.y =
          pub_data.des_twist.value().angular.y - pub_data.current_twist[4];
      twist.twist.angular.z =
          pub_data.des_twist.value().angular.z - pub_data.current_twist[5];

      velocity_error_debug->publish(twist);
    }

    // VELOCITY

    // Publish minimum singular value
    if (pub_data.sigma_min.has_value()) {
      double_stamped.data = pub_data.sigma_min.value();
      min_singular_val_pub->publish(double_stamped);
    }
  } catch (std::exception &ex) {
    std::cout << "Error in debug publisher, not publishing: " << ex.what();
  }

  pub_data.has_data = false;
  pub_data.mut.unlock();
}
