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
const std::string forward_kine_service_name{"forward_kine"};
const std::string jacob_calc_service_name{"calculate_jacobian"};
const std::string joints_cmd_pos_service_name{"send_joints_pos_cmd"};
const std::string clik_service_name{"clik"};
} // namespace panda_interface_names
