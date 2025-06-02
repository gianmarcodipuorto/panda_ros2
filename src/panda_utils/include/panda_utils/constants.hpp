#pragma once

#include "robot.hpp"
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

namespace panda_interface_names {
/////////////////////////////////////////////////////////////
const std::string forward_kine_service_name{"forward_kine"};
const std::string jacob_calc_service_name{"calculate_jacobian"};
const std::string joints_cmd_pos_service_name{"send_joints_pos_cmd"};
const std::string clik_service_name{"clik"};
const std::string pd_grav_controller_node_name{"pd_plus_gravity_controller"};

/////////////////////////////////////////////////////////////
const std::string panda_effort_cmd_topic_name{"/panda/cmd/effort"};
const std::string panda_pos_cmd_topic_name{"/panda/cmd/joint_pos"};
const std::string panda_pose_cmd_topic_name{"/panda/cmd/pose"};
const std::string joint_state_topic_name{"/joint_states"};
const std::string force_torque_sensor_topic_name{"/ft_sensors"};
const std::string start_and_stop_clik_topic_name{"/clik_ctrl"};

/////////////////////////////////////////////////////////////
const std::string panda_traj_move_action_name{"joint_traj_action"};
const std::string panda_cart_move_action_name{"cart_traj_action"};

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

const int DEFAULT_TOPIC_QOS = 10;
} // namespace panda_interface_names
namespace panda_constants {
const std::string panda_model_effort{"/models/panda/panda_fr3.urdf"};
}
