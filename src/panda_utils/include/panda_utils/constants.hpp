#pragma once

#include "franka/robot.h"
#include "robot.hpp"
#include <rclcpp/qos.hpp>
#include <string>

const DHParameters PANDA_DH_PARAMETERS[7] = {
    {0.0, -M_PI_2, 0.3330, 0.0},   {0.0, M_PI_2, 0.0, 0.0},
    {0.0825, M_PI_2, 0.3160, 0.0}, {-0.0825, -M_PI_2, 0.0, 0.0},
    {0.0, M_PI_2, 0.3840, 0.0},    {0.0880, M_PI_2, 0.0, 0.0},
    {0.0, 0.0, 0.1070, 0.0}};

const JointType PANDA_JOINT_TYPES[7] = {
    JointType::REVOLUTE, JointType::REVOLUTE, JointType::REVOLUTE,
    JointType::REVOLUTE, JointType::REVOLUTE, JointType::REVOLUTE,
    JointType::REVOLUTE};

extern const rclcpp::Duration max_tf_age;
extern const double robot_radius_area; // meters

namespace panda_interface_names {

/////////////////////////////////////////////////////////////
const std::string pd_grav_controller_node_name{"pd_plus_gravity_controller"};
const std::string inverse_dynamics_controller_node_name{
    "inverse_dynamics_controller"};
const std::string controller_manager_node_name{"controller_manager"};
const std::string clik_node_name{"clik_cmd_pub"};
const std::string cart_traj_node_name{"cart_traj_server"};
const std::string exponential_stop_traj_node_name{"exponential_stop_server"};
const std::string loop_cart_traj_node_name{"loop_cart_traj_server"};

/////////////////////////////////////////////////////////////
const std::string joints_cmd_pos_service_name{"send_joints_pos_cmd"};
const std::string set_compliance_mode_service_name{"set_compliance_mode"};
const std::string set_wrist_contact_service_name{"set_wrist_contact_index"};

/////////////////////////////////////////////////////////////
// Joint space
const std::string panda_effort_cmd_topic_name{"/panda/cmd/effort"};
const std::string panda_pos_cmd_topic_name{"/panda/cmd/joint_pos"};
const std::string panda_joint_cmd_topic_name{"/panda/cmd/joint_cmd"};
// Cartesian space
const std::string panda_pose_cmd_topic_name{"/panda/cmd/pose"};
const std::string panda_twist_cmd_topic_name{"/panda/cmd/twist"};
const std::string panda_accel_cmd_topic_name{"/panda/cmd/accel"};

const std::string joint_state_topic_name{"/joint_states"};
const std::string panda_pose_state_topic_name{"/pose_state"};
const std::string torque_sensor_topic_name{"/tau_sensors"};
const std::string start_and_stop_clik_topic_name{"/clik_ctrl"};
const std::string min_singular_value_topic_name{"/sigma_min"};
const std::string pose_error_topic_name{"/pose_error"};

const std::string demo_state_topic_name{"/state_color"};
const std::string wrist_contact_index_topic_name{"/panda/wrist_contact_index"};

/////////////////////////////////////////////////////////////
const std::string panda_traj_move_action_name{"joint_traj_action"};
const std::string panda_cart_move_action_name{"cart_traj_action"};
const std::string panda_cart_loop_action_name{"loop_cart_traj_action"};
const std::string panda_exponential_stop_action_name{"exponential_stop_action"};

/////////////////////////////////////////////////////////////
const auto bridge_effort_cmd_topic_names = {
    "/joint1/cmd_force", "/joint2/cmd_force", "/joint3/cmd_force",
    "/joint4/cmd_force", "/joint5/cmd_force", "/joint6/cmd_force",
    "/joint7/cmd_force",
};
const auto bridge_pos_cmd_topic_names = {
    "/joint1/cmd_pos", "/joint2/cmd_pos", "/joint3/cmd_pos", "/joint4/cmd_pos",
    "/joint5/cmd_pos", "/joint6/cmd_pos", "/joint7/cmd_pos",
};

const std::vector<std::string> panda_joint_names = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7",
};

const std::vector<std::string> panda_link_names = {
    "fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
    "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8",
};

const inline rclcpp::QoS DEFAULT_TOPIC_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

const inline rclcpp::QoS CONTROLLER_SUBSCRIBER_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

const inline rclcpp::QoS CONTROLLER_PUBLISHER_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

} // namespace panda_interface_names
namespace panda_constants {
const std::string panda_model_effort{"/models/panda/panda_fr3.urdf"};
const std::string panda_model_effort_no_table{
    "/models/panda/panda_fr3_no_table.urdf"};

const Eigen::Vector<double, 7> effort_limits{87.0, 87.0, 87.0, 87.0,
                                             12.0, 12.0, 12.0};

const Eigen::Vector<double, 7> effort_speed_limits{
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

const Eigen::Vector<double, 7> joint_min_limits = Eigen::Vector<double, 7>{
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

const Eigen::Vector<double, 7> joint_max_limits = Eigen::Vector<double, 7>{
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const Eigen::Vector<double, 7> velocity_limits = Eigen::Vector<double, 7>{
    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};

const Eigen::Vector<double, 7> acceleration_limits =
    Eigen::Vector<double, 7>{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

} // namespace panda_constants
