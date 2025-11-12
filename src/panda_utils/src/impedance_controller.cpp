#include "franka/control_types.h"
#include "franka/exception.h"
#include "franka/lowpass_filter.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "image_processing/constants.hpp"
#include "multibody/fwd.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "panda_interfaces/msg/human_contact.hpp"
#include "panda_interfaces/msg/joint_torque_measure_stamped.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_interfaces/srv/wrist_contact.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/debug_publisher.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "panda_utils/utils_func.hpp"
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <memory>
#include <mutex>
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
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <vector>

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
using namespace std::chrono_literals;

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort_no_table;

enum class Mode {
  // Simulation only mode
  sim,
  // Real mode with franka panda library and communication
  franka
};

struct robot_state {
  std::optional<franka::RobotState> state;
  rclcpp::Time state_time;
  std::mutex mut;
};

/**
 * @brief Compute Jdot as finite difference: J_forw - J_back / (2 dt) = J_dot
 *
 * @param std::function<Eigen::Matrix<double, 6, 7>(Eigen::Vector<double, 7>)>
 * get_jacobian_func
 * @param const Eigen::Vector<double, 7> & current_joint_pos
 * @param const Eigen::Vector<double, 7> & current_joint_speed
 * @param const double delta_time = 1e-3
 * @return Eigen::Matrix<double, 6, 7>
 */
Eigen::Matrix<double, 6, 7>
get_j_dot(std::function<Eigen::Matrix<double, 6, 7>(Eigen::Vector<double, 7>)>
              get_jacobian_func,
          const Eigen::Vector<double, 7> &current_joint_pos,
          const Eigen::Vector<double, 7> &current_joint_speed) {
  const double delta_time = 1e-3; // Small time step for finite difference

  Eigen::Vector<double, 7> joint_pos_forw =
      current_joint_pos + current_joint_speed * delta_time;
  Eigen::Vector<double, 7> joint_pos_back =
      current_joint_pos - current_joint_speed * delta_time;

  Eigen::Matrix<double, 6, 7> J_forw = get_jacobian_func(joint_pos_forw);
  Eigen::Matrix<double, 6, 7> J_back = get_jacobian_func(joint_pos_back);

  Eigen::Matrix<double, 6, 7> J_dot = (J_forw - J_back) / (2 * delta_time);

  return J_dot;
}

Eigen::Matrix<double, 7, 6>
compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian) {
  return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::Matrix<double, 6, 7>
compute_jacob_pseudoinv_h_e(const Eigen::Matrix<double, 7, 6> &jacobian) {
  return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

void print_initial_franka_state(const franka::RobotState state,
                                const franka::Model &model,
                                rclcpp::Logger logger) {
  Eigen::Vector<double, 7> vec7;
  for (int i = 0; i < 7; i++) {
    vec7[i] = state.q[i];
  }

  for (size_t i = 0; i < state.q.size(); i++) {
    RCLCPP_INFO_STREAM(logger, "Joint " << i + 1 << ": " << state.q[i]);
  }
  Pose current_pose =
      geom_utils::get_pose(model.pose(franka::Frame::kFlange, state));
  RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position: ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");

  current_pose = geom_utils::get_pose(state.O_T_EE);
  RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position (with O_T_EE): ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");
  auto jacobian = geom_utils::get_jacobian(
      model.zeroJacobian(franka::Frame::kFlange, state));
  RCLCPP_INFO_STREAM_ONCE(logger, "Current jacobian: [" << jacobian << "]");

  // B(q)
  std::array<double, 49> mass_matrix_raw = model.mass(state);
  Eigen::Matrix<double, 7, 7> mass_matrix;
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < 7; j++) {
      mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
    }
  }

  std::array<double, 7> coriolis_raw = model.coriolis(state);
  Eigen::Vector<double, 7> coriolis;
  for (size_t i = 0; i < 7; i++) {
    coriolis(i) = coriolis_raw[i];
  }

  auto get_jacob = [&model](const Eigen::Vector<double, 7> &current_joint_pos) {
    auto state = franka::RobotState{};
    for (size_t i = 0; i < state.q.size(); i++) {
      state.q[i] = current_joint_pos[i];
    }

    return geom_utils::get_jacobian(
        model.zeroJacobian(franka::Frame::kFlange, state));
  };

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                      Eigen::ComputeThinV);
  double sigma_min = svd.singularValues().tail(1)(0);

  // DYNAMIC LAMBDA BASED ON MINIMUM SINGULAR VALUE
  double k_max = 1.0;
  double eps = 0.1;
  double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
  lambda = (sigma_min >= eps ? 0.0 : lambda);
  Eigen::Matrix<double, 7, 6> jacobian_pinv = compute_jacob_pseudoinv(jacobian);

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current mass matrix: [" << mass_matrix << "]");

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current coriolis vector: [" << coriolis << "]");

  RCLCPP_INFO_STREAM_ONCE(logger,
                          "Current pseudoinv: [" << jacobian_pinv << "]");

  RCLCPP_INFO_STREAM_ONCE(
      logger, "J dot: [" << get_j_dot(get_jacob, vec7, vec7.Zero()) << "]");
}

class ImpedanceController : public rclcpp_lifecycle::LifecycleNode {

public:
  ImpedanceController(const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::inverse_dynamics_controller_node_name),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 50.0);
    this->declare_parameter<double>("Kd", 10.0);
    this->declare_parameter<double>("Md", 1.0);
    this->declare_parameter<double>("Kp_rot", 50.0);
    this->declare_parameter<double>("Kd_rot", 10.0);
    this->declare_parameter<double>("Md_rot", 1.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<double>("safe_joint_speed", 0.7);
    this->declare_parameter<double>("safe_effort_perc", 0.5);
    this->declare_parameter<double>("safe_error_pose_norm", 0.035);
    this->declare_parameter<bool>("clamp", true);
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<bool>("use_robot", false);
    this->declare_parameter<bool>("use_franka_sim", false);
    this->declare_parameter<std::vector<double>>(
        "world_base_link",
        std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    Kp_rot = this->get_parameter("Kp_rot").as_double();
    Kd_rot = this->get_parameter("Kd_rot").as_double();
    Md_rot = this->get_parameter("Md_rot").as_double();

    // Safe limits
    joint_speed_safe_limit =
        this->get_parameter("safe_joint_speed").as_double();
    percentage_effort_safe_limit =
        this->get_parameter("safe_effort_perc").as_double();
    error_pose_norm_safe_limit =
        this->get_parameter("safe_error_pose_norm").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();
    bool use_robot = this->get_parameter("use_robot").as_bool();
    bool use_franka_sim = this->get_parameter("use_franka_sim").as_bool();
    this->get_parameter<std::vector<double>>("world_base_link",
                                             world_base_link);

    franka_frame_enum_to_link_name[franka::Frame::kJoint1] = "fr3_link1";
    franka_frame_enum_to_link_name[franka::Frame::kJoint2] = "fr3_link2";
    franka_frame_enum_to_link_name[franka::Frame::kJoint3] = "fr3_link3";
    franka_frame_enum_to_link_name[franka::Frame::kJoint4] = "fr3_link4";
    franka_frame_enum_to_link_name[franka::Frame::kJoint5] = "fr3_link5";
    franka_frame_enum_to_link_name[franka::Frame::kJoint6] = "fr3_link6";
    franka_frame_enum_to_link_name[franka::Frame::kJoint7] = "fr3_link7";
    franka_frame_enum_to_link_name[franka::Frame::kFlange] =
        "fr3_link8"; // Typically end-effector name

    robot_link_name_to_franka_frame["fr3_link1"] = franka::Frame::kJoint1;
    robot_link_name_to_franka_frame["fr3_link2"] = franka::Frame::kJoint2;
    robot_link_name_to_franka_frame["fr3_link3"] = franka::Frame::kJoint3;
    robot_link_name_to_franka_frame["fr3_link4"] = franka::Frame::kJoint4;
    robot_link_name_to_franka_frame["fr3_link5"] = franka::Frame::kJoint5;
    robot_link_name_to_franka_frame["fr3_link6"] = franka::Frame::kJoint6;
    robot_link_name_to_franka_frame["fr3_link7"] = franka::Frame::kJoint7;
    robot_link_name_to_franka_frame["fr3_link8"] = franka::Frame::kFlange;

    // Define mode of operation
    if (use_robot) {
      mode = Mode::franka;
      RCLCPP_INFO_STREAM(this->get_logger(), "Using mode franka");
    } else {
      mode = Mode::sim;
      RCLCPP_INFO_STREAM(this->get_logger(), "Using mode sim");
    }

    // Taking joint limits
    load_joint_limits();

    auto set_joint_state = [this](const JointState::SharedPtr msg) {
      current_joint_config = msg;
    };

    robot_joint_states_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS(), set_joint_state);

    auto set_cartesian_cmd =
        [this](
            const panda_interfaces::msg::CartesianCommand::ConstSharedPtr msg) {
          desired_cartesian_box.try_set(*msg);
        };

    auto set_external_tau_cb = [this](const JointTorqueMeasureStamped msg) {
      for (int i = 0; i < 7; i++) {
        extern_tau[i] = msg.measures.torque[i];
      }
    };

    auto set_human_contact_info_cb =
        [this](const panda_interfaces::msg::HumanContact msg) {
          human_contact_info = msg;
        };

    cartesian_cmd_sub =
        this->create_subscription<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd", panda_interface_names::DEFAULT_TOPIC_QOS(),
            set_cartesian_cmd);

    external_tau_sub = this->create_subscription<JointTorqueMeasureStamped>(
        panda_interface_names::torque_sensor_topic_name,
        panda_interface_names::CONTROLLER_SUBSCRIBER_QOS(),
        set_external_tau_cb);

    human_contact_sub =
        this->create_subscription<panda_interfaces::msg::HumanContact>(
            panda_interface_names::human_contact_topic,
            panda_interface_names::DEFAULT_TOPIC_QOS(),
            set_human_contact_info_cb);

    robot_joint_efforts_pub = this->create_publisher<JointsEffort>(
        panda_interface_names::panda_effort_cmd_topic_name,
        // Leave this as reliable otherwise the effort could not be delivered
        // to gz bridge interface
        panda_interface_names::DEFAULT_TOPIC_QOS());

    robot_frame_poses_pub =
        this->create_publisher<geometry_msgs::msg::PoseArray>(
            panda_interface_names::panda_frame_poses_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS());

    // Compliance mode Service
    auto compliance_mode_cb =
        [this](const panda_interfaces::srv::SetComplianceMode_Request::SharedPtr
                   request,
               panda_interfaces::srv::SetComplianceMode_Response::SharedPtr
                   response) {
          if (current_joints_speed.norm() >= 1e-2) {
            response->result = false;
            RCLCPP_ERROR(this->get_logger(),
                         "Speed too high: can't set compliance mode");
            return;
          }
          if (request->cmd) {

            RCLCPP_INFO(this->get_logger(), "Decreasing KP");
            auto start = this->now();
            Eigen::Matrix<double, 6, 6> initial_KP = KP;
            Eigen::Matrix<double, 6, 6> initial_KD = KD;
            Eigen::Matrix<double, 7, 7> initial_KD_J = KD_J;
            while ((this->now() - start).seconds() < compliance_switch_time) {
              // Normalized wrt the compliance switch time interval
              auto tau =
                  (this->now() - start).seconds() / compliance_switch_time;
              if (tau > 1.0) {
                tau = 1.0;
              }
              KD = (initial_KD * 0.6) * tau + initial_KD * (1.0 - tau);
              KP = initial_KP * exp(-9.0 * tau);
              rclcpp::sleep_for(5ms);
            }
            KP = KP * 0.0;

            set_md_kd(2.0);
            RCLCPP_INFO_STREAM(this->get_logger(), "KP = " << KP);
            RCLCPP_INFO_STREAM(this->get_logger(), "KD = " << KD);
            RCLCPP_INFO_STREAM(this->get_logger(), "KD_J = " << KD_J);
            compliance_mode.store(true);
            response->result = true;
            RCLCPP_INFO(this->get_logger(),
                        "Set controller to compliance mode");
          } else {
            if (mode == Mode::franka) {
              panda_franka_state.mut.lock();

              Eigen::Map<const Eigen::Vector<double, 7>>
                  current_joints_config_vec(panda_franka_state.state->q.data());
              if (current_joints_config_vec.array().isZero()) {
                response->result = false;
                panda_franka_state.mut.unlock();
                RCLCPP_ERROR(this->get_logger(), "Joint state is near 0");
                return;
              }
              std::lock_guard<std::mutex> mut(desired_cartesian_mutex);
              init_cartesian_cmd(current_joints_config_vec);

              panda_franka_state.mut.unlock();
            } else {
              Eigen::Vector<double, 7> current_joints_config_vec;
              for (size_t i = 0; i < current_joint_config->position.size();
                   i++) {
                current_joints_config_vec[i] =
                    current_joint_config->position[i];
              }
              init_cartesian_cmd(current_joints_config_vec);
            }
            set_kd_j();
            set_kp();
            set_kd();
            set_md();
            compliance_mode.store(false);
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "Unset controller compliance mode");
          }
        };

    compliance_mode_server =
        this->create_service<panda_interfaces::srv::SetComplianceMode>(
            panda_interface_names::set_compliance_mode_service_name,
            compliance_mode_cb);

    enable_frame_publishing_client =
        this->create_client<std_srvs::srv::SetBool>(
            "/set_publishing_pose_frames");

    robot_pose_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            panda_interface_names::panda_pose_state_topic_name,
            panda_interface_names::CONTROLLER_PUBLISHER_QOS()));

    joint_states_pub =
        std::make_shared<realtime_tools::RealtimePublisher<JointState>>(
            this->create_publisher<JointState>(
                panda_interface_names::joint_state_topic_name,
                panda_interface_names::DEFAULT_TOPIC_QOS()));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {

    using namespace std::chrono_literals;
    debug_pub.create_pubs(shared_from_this(),
                          panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    // Reconfigure parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();

    RCLCPP_INFO_STREAM(
        get_logger(),
        "Configuring node "
            << this->get_name() << " using "
            << (this->get_clock()->ros_time_is_active() ? "simulation clock"
                                                        : "system clock")
            << " and rate clock "
            << (this->control_loop_rate->get_type() == RCL_ROS_TIME
                    ? "simulation clock"
                    : "system clock")
            << ", and control loop rate (Hz) "
            << 1.0 / (control_loop_rate->period().count() * 1e-9)
            << " with Kp = " << Kp << ", Kd = " << Kd << ", Md = " << Md
            << ", Kp_rot = " << Kp_rot << ", Kd_rot = " << Kd_rot
            << ", Md_rot = " << Md_rot);

    switch (mode) {
    case Mode::franka: {
      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No real time kernel available for franka lib communication");
        return CallbackReturn::FAILURE;
      }

      panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Connected to robot with ip "
                             << this->get_parameter("robot_ip").as_string());

      // Collision behavior set to 100 N / 100 N/m allowing the compliance
      // operations
      std::array<double, 7> temp{100.0, 100.0, 100.0, 100.0,
                                 100.0, 100.0, 100.0};
      std::array<double, 6> temp2{100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
      panda_franka->setCollisionBehavior(temp, temp, temp, temp, temp2, temp2,
                                         temp2, temp2);
      RCLCPP_INFO_STREAM(this->get_logger(), "Set collision behaviour");
      panda_franka->setLoad(load, F_x_Cload, load_inertia);
      auto state = panda_franka->readOnce();
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Load: " << state.m_load << " Load EE: " << state.m_ee
                                  << "\n F_x_Cload: " << state.F_x_Cload[0]
                                  << ", " << state.F_x_Cload[1] << ", "
                                  << state.F_x_Cload[2]);

      RCLCPP_INFO_STREAM(this->get_logger(), "Set load on real robot");

      panda_franka_model = panda_franka.value().loadModel();

      // Debug prints before activation
      print_initial_franka_state(panda_franka->readOnce(),
                                 panda_franka_model.value(),
                                 this->get_logger());

      set_kp();

      set_kd();

      set_md();

      set_kd_j();

      // Used for J_dot calculation
      auto get_jacob = [this](
                           const Eigen::Vector<double, 7> &current_joint_pos) {
        auto state = franka::RobotState{};
        for (size_t i = 0; i < state.q.size(); i++) {
          state.q[i] = current_joint_pos[i];
        }

        if (compliance_mode.load() && last_joint_contact_frame.has_value()) {

          jacobian = geom_utils::get_jacobian(panda_franka_model->zeroJacobian(
              last_joint_contact_frame.value(), state));

        } else {
          jacobian = geom_utils::get_jacobian(
              panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
        }
        return jacobian;
      };

      joint_state_to_pub.position.resize(7);
      joint_state_to_pub.velocity.resize(7);
      joint_state_to_pub.effort.resize(7);
      joint_state_to_pub.name = std::vector<std::string>{
          "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

      // Callback passed to libfranka library. The callback handles the control
      // of the robot, implementing the control law. Furthermore, the callback
      // handles the update of the robot state in the ROS2 network, copying the
      // variables within the `franka::RobotState` object and exposing them to
      // the other nodes through Publishers
      robot_control_callback =
          [this, get_jacob](const franka::RobotState &state,
                            franka::Duration dt) -> franka::Torques {
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        // Access the optional RealtimeThreadSafeBox
        // If it has something use the new value, else use the latest value
        // available
        {
          std::lock_guard<std::mutex> mut(desired_cartesian_mutex);
          update_cartesian_cmd();
        }

        // Get q and q_dot
        //
        Eigen::Map<const Eigen::Vector<double, 7>> current_joints_config_vec(
            state.q.data());

        if (dt.toSec() != 0.0) {
          for (int i = 0; i < 7; i++) {
            // The joints' speed are filtered, the cutoff freq can be fine tuned
            current_joints_speed[i] = franka::lowpassFilter(
                dt.toSec(), state.dq[i], current_joints_speed[i],
                joints_speed_cutoff_freq);
          }

        } else {
          for (int i = 0; i < 7; i++) {
            current_joints_speed[i] = state.dq[i];
          }
        }

        Pose current_pose;
        Eigen::Quaterniond error_quat{};
        Eigen::Vector<double, 6> error_pose_vec{};
        std::array<double, 49> mass_matrix_raw;

        // Get current pose and jacobian according to frame in contact or last
        // frame if not in compliance mode
        if (compliance_mode.load() && last_joint_contact_frame.has_value()) {

          current_pose = geom_utils::get_pose(panda_franka_model->pose(
              last_joint_contact_frame.value(), state));

          jacobian = geom_utils::get_jacobian(panda_franka_model->zeroJacobian(
              last_joint_contact_frame.value(), state));
        } else {
          current_pose = geom_utils::get_pose(
              panda_franka_model->pose(franka::Frame::kFlange, state));
          jacobian = geom_utils::get_jacobian(
              panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
        }

        Eigen::Quaterniond current_quat{};
        current_quat.w() = current_pose.orientation.w;
        current_quat.x() = current_pose.orientation.x;
        current_quat.y() = current_pose.orientation.y;
        current_quat.z() = current_pose.orientation.z;
        current_quat.normalize();
        current_quat = quaternionContinuity(current_quat, old_quaternion);
        old_quaternion = current_quat;

        // Calculate pose error
        Eigen::AngleAxisd error_angle_axis;
        {
          Eigen::Quaterniond desired_quat{};
          desired_quat.w() = desired_pose.orientation.w;
          desired_quat.x() = desired_pose.orientation.x;
          desired_quat.y() = desired_pose.orientation.y;
          desired_quat.z() = desired_pose.orientation.z;
          desired_quat.normalize();

          error_quat = desired_quat * current_quat.inverse();
          error_quat.normalize();
          error_angle_axis = Eigen::AngleAxisd{error_quat};

          error_pose_vec(0) = desired_pose.position.x - current_pose.position.x;
          error_pose_vec(1) = desired_pose.position.y - current_pose.position.y;
          error_pose_vec(2) = desired_pose.position.z - current_pose.position.z;
          error_pose_vec(3) = error_quat.x();
          error_pose_vec(4) = error_quat.y();
          error_pose_vec(5) = error_quat.z();
        }

        // B(q)
        mass_matrix_raw = this->panda_franka_model.value().mass(state);
        Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(
            mass_matrix_raw.data());

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
            this->panda_franka_model.value().coriolis(state).data());

        // Calculate jacobian SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double sigma_min = svd.singularValues().tail(1)(0);

        // DYNAMIC LAMBDA BASED ON MINIMUM SINGULAR VALUE
        Eigen::Matrix<double, 7, 6> jacobian_pinv =
            compute_jacob_pseudoinv(jacobian);

        // Getting external tau
        Eigen::Map<const Eigen::Vector<double, 7>> tau_ext_measured(
            state.tau_ext_hat_filtered.data());

        Eigen::Matrix<double, 7, 6> jacobian_transposed = jacobian.transpose();

        if (dt.toSec() == 0.0) {
          extern_tau = extern_tau.Zero();
        } else {
          for (int i = 0; i < 6; i++) {
            extern_tau[i] =
                franka::lowpassFilter(dt.toSec(), tau_ext_measured[i],
                                      extern_tau[i], external_tau_cutoff_freq);
          }
        }

        // While in compliance mode, based on the link frame touched by the
        // operator, we consider the external forces acting on that frame
        // excluding the external torques of the subsequent joints. E.g. if the
        // operator touches the 4th frame, we consider only the torques acting
        // on joints 1-4
        Eigen::Vector<double, 7> tau_ext = extern_tau;
        if (dt.toSec() == 0.0) {
          h_e = h_e.Zero();
        } else {
          if (compliance_mode.load() && last_joint_contact_frame.has_value()) {
            int index = 0;
            switch (last_joint_contact_frame.value()) {
            case franka::Frame::kJoint1: {
              index = 0;
              break;
            }
            case franka::Frame::kJoint2: {
              index = 1;
              break;
            }
            case franka::Frame::kJoint3: {
              index = 2;
              break;
            }
            case franka::Frame::kJoint4: {
              index = 3;
              break;
            }
            case franka::Frame::kJoint5: {
              index = 4;
              break;
            }
            case franka::Frame::kJoint6: {
              index = 5;
              break;
            }
            case franka::Frame::kJoint7: {
              index = 6;
              break;
            }
            case franka::Frame::kFlange: {
              index = 6;
              break;
            }
            case franka::Frame::kEndEffector:
            case franka::Frame::kStiffness:
              break;
            }
            for (int i = index + 1; i < tau_ext.size(); i++) {
              tau_ext[i] = 0.0;
            }
          }
          h_e_measured =
              compute_jacob_pseudoinv_h_e(jacobian_transposed) * tau_ext;

          h_e = h_e_measured;
        }

        // Calculate quantities for control
        // Calculated through libfranka lib for better accuracy
        // WARN: the libfranka lib considers the torque command sent to the
        // robot as part of the real torque commanded.
        // The torque commanded is the sum of 3 terms:
        // - tau_d = user commanded torque
        // - tua_g = torque required for gravity compensation
        // - tau_f = torque required to compensate motor friction
        //

        Eigen::Vector<double, 7> y;
        Eigen::Vector<double, 6> y_cartesian;
        Eigen::Vector<double, 6> current_twist;
        Eigen::Vector<double, 6> error_twist;
        {
          current_twist = jacobian * current_joints_speed;
          error_twist = desired_twist_vec - current_twist;
        }

        {
          // clang-format off
          if (compliance_mode.load()) {

            y = jacobian_pinv * MD_1 *
                (
                  -KD * current_twist 
                  - MD *
                   get_j_dot(get_jacob, current_joints_config_vec,
                             current_joints_speed) *
                   current_joints_speed
                  - h_e
                );
          } else {
            y_cartesian =
                (
                  MD * desired_accel_vec +
                  KD * error_twist + KP * error_pose_vec 
                  -MD *
                      get_j_dot(get_jacob, current_joints_config_vec,
                                current_joints_speed) *
                      current_joints_speed
                  - h_e
                );
            y = jacobian_pinv * MD_1 * y_cartesian;
          }
          // clang-format on
        }

        Eigen::Vector<double, 7> control_input_vec =
            mass_matrix * y + coriolis + extern_tau -
            KD_J * current_joints_speed;

        // Clamp tau
        clamp_control(control_input_vec);

        // Safety checks
        if (dt.toSec() == 0.0) {
          for (int i = 0; i < control_input_vec.size(); i++) {
            control_input_vec[i] = 0.0;
          }
        } else {

          RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Running safety checks");
          try {
            for (int i = 0; i < control_input_vec.size(); i++) {
              if (abs(control_input_vec[i]) >=
                  percentage_effort_safe_limit * effort_limits[i]) {
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                        "Running safety check: effort limit");
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Torque abs value over limit ("
                                        << percentage_effort_safe_limit * 100.0
                                        << "%)");
                panda_franka->stop();
                start_flag.store(false);
                return franka::MotionFinished(franka::Torques(state.tau_J_d));
              } else if (abs(state.dq[i]) >= joint_speed_safe_limit) {
                RCLCPP_INFO_STREAM_ONCE(
                    this->get_logger(),
                    "Running safety check: joint limit speed");
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Joint velocity over the safety value "
                                        << joint_speed_safe_limit);
                panda_franka->stop();
                start_flag.store(false);
                return franka::MotionFinished(franka::Torques(state.tau_J_d));
              }
            }
            if (current_pose.position.z <= 0.15) {
              RCLCPP_ERROR_STREAM(this->get_logger(),
                                  "Height of the end affector wrt base under "
                                  "allowed value 0.15m");
              panda_franka->stop();
              start_flag.store(false);
              return franka::MotionFinished(franka::Torques(state.tau_J_d));
            }

          } catch (std::exception &ex) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Error in safety checks: " << ex.what());
            panda_franka->stop();
            start_flag.store(false);
            return franka::MotionFinished(franka::Torques(state.tau_J_d));
          }
          RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                  "Finished safety checks first time");
        }

        // Apply control

        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Filling tau command");
        last_control_input = control_input_vec;

        if (control_input_vec.array().isNaN().any() ||
            control_input_vec.array().isInf().any()) {
          RCLCPP_ERROR_STREAM(
              this->get_logger(),
              "Control input vec Nan or Inf: "
                  << control_input_vec
                  << ", Desired pose: " << desired_pose.position.x << ", "
                  << desired_pose.position.y << ", " << desired_pose.position.z
                  << ", Desired twist: " << desired_twist_vec
                  << ", Desired accel: " << desired_accel_vec
                  << ", Jacobian pinv: " << jacobian_pinv << ", error twist: "
                  << error_twist << ", error pose: " << error_pose_vec
                  << ", extern_tau: " << extern_tau << ", coriolis: "
                  << coriolis << ", joint vel: " << current_joints_speed
                  << ", jacobian: " << jacobian
                  << ", current joint pos: " << current_joints_config_vec);

          panda_franka->stop();
          start_flag.store(false);
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        std::array<double, 7> tau;
        for (size_t i = 0; i < 7; ++i) {
          tau[i] = control_input_vec[i];
        }

        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Filling debug data");

        // Fill struct for TFs prints
        if (panda_franka_state.mut.try_lock()) {
          panda_franka_state.state = state;
          panda_franka_state.mut.unlock();
        }

        // Fill struct for debug data publishing
        if (debug_pub.data().mut.try_lock()) {

          try {
            debug_pub.data().h_e = h_e_measured;
            debug_pub.data().h_e_calculated = h_e;
            debug_pub.data().tau_ext = tau_ext_measured;
            debug_pub.data().tau_ext_calculated = extern_tau;
            debug_pub.data().error_theta =
                error_angle_axis.angle() * 180.0 / M_PI;
            debug_pub.data().sigma_min = sigma_min;
            debug_pub.data().current_twist = current_twist;
            debug_pub.data().des_twist = desired_twist;
            debug_pub.data().des_accel = desired_accel;
            debug_pub.data().current_j_dot_q_dot =
                get_j_dot(get_jacob, current_joints_config_vec,
                          current_joints_speed) *
                current_joints_speed;
            debug_pub.data().gravity = panda_franka_model->gravity(state);
            debug_pub.data().coriolis = coriolis;
            debug_pub.data().filtered_joints_vec = current_joints_speed;
            debug_pub.data().error_pose_vec.head(3) = error_pose_vec.head(3);
            debug_pub.data().error_pose_vec.tail(3) = error_pose_vec.tail(3);
            // w value of the pose message in a vector<7>
            debug_pub.data().error_pose_vec(3) = 1.0;
            debug_pub.data().tau_d_calculated = tau;
            debug_pub.data().tau_d_last = state.tau_J_d;
            debug_pub.data().tau_read = state.tau_J;
            debug_pub.data().y = y;
            debug_pub.data().y_cartesian = y_cartesian;

            // Robot state
            debug_pub.data().robot_state->q = state.q;
            debug_pub.data().robot_state->dq = state.dq;
            debug_pub.data().robot_state->O_T_EE = state.O_T_EE;
            debug_pub.data().current_pose = current_pose;
            debug_pub.data().has_data = true;
          } catch (std::exception &ex) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Error copying data in controller: " << ex.what());
          }
          debug_pub.data().mut.unlock();
        }

        // The pose and the joints' values are published directly by the main
        // control thread
        PoseStamped pose_stamp;
        pose_stamp.header.stamp = this->now();
        pose_stamp.pose = current_pose;
        robot_pose_pub->try_publish(pose_stamp);

        joint_state_to_pub.header.stamp = this->now();
        for (size_t i = 0; i < joint_state_to_pub.position.size(); i++) {
          joint_state_to_pub.position[i] = state.q[i];
          joint_state_to_pub.velocity[i] = state.dq[i];
          joint_state_to_pub.effort[i] = state.tau_J[i];
        }

        joint_states_pub->try_publish(joint_state_to_pub);

        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sent command first time");
        return franka::Torques(tau);
      };
      break;
    }
    case Mode::sim:
      break;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {

    RCLCPP_INFO(get_logger(), "Activating...");
    using namespace std::chrono_literals;

    last_control_input.resize(panda.getModel().nq);
    for (int i = 0; i < last_control_input.size(); i++) {
      last_control_input[i] = 0.0;
    }

    switch (mode) {
    case Mode::franka: {

      publish_static_transforms();

      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(this->get_logger(),
                     "The robot thread has no real time kernel, shutting down");
        start_flag.store(false);
        rclcpp::shutdown();
      }

      // Initializing control setpoint variables
      init_cartesian_cmd();

      update_cartesian_cmd();

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Desired pose is: ["
                             << desired_pose.position.x << ", "
                             << desired_pose.position.y << ", "
                             << desired_pose.position.z << "], ["
                             << desired_pose.orientation.w << ", "
                             << desired_pose.orientation.x << ", "
                             << desired_pose.orientation.y << ", "
                             << desired_pose.orientation.z << "]");

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Desired accel is: ["
              << desired_accel_vec[0] << ", " << desired_accel_vec[1] << ", "
              << desired_accel_vec[2] << "], [" << desired_accel_vec[3] << ", "
              << desired_accel_vec[4] << ", " << desired_accel_vec[5] << "]");

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Desired twist is: ["
              << desired_twist_vec[0] << ", " << desired_twist_vec[1] << ", "
              << desired_twist_vec[2] << "], [" << desired_twist_vec[3] << ", "
              << desired_twist_vec[4] << ", " << desired_twist_vec[5] << "]");

      // Init old quaternion for quaternion continuity
      old_quaternion.w() = desired_pose.orientation.w;
      old_quaternion.x() = desired_pose.orientation.x;
      old_quaternion.y() = desired_pose.orientation.y;
      old_quaternion.z() = desired_pose.orientation.z;
      old_quaternion.normalize();

      if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return CallbackReturn::FAILURE;
      }

      start_flag.store(true);

      control_thread = std::thread{[this]() {
        // Configuring real time thread
        try {

          // Frame poses publisher thread
          std::thread{[this]() {
            while (!enable_frame_publishing_client->wait_for_service()) {
            }
            std_srvs::srv::SetBool_Request req;
            req.data = true;
            enable_frame_publishing_client->async_send_request(
                std::make_shared<std_srvs::srv::SetBool_Request>(req));
            poses_publish_loop();
          }}.detach();

          // Applied external force frame
          std::thread{[this]() {
            // Define the frames we have to change gains for
            std::vector<franka::Frame> frame_to_change_gains{
                franka::Frame::kJoint1, franka::Frame::kJoint2,
                franka::Frame::kJoint3, franka::Frame::kJoint4};

            while (start_flag.load() && rclcpp::ok()) {
              if (compliance_mode.load()) {
                if (!human_contact_info.in_contact_wrist.data.empty() &&
                    !human_contact_info.joint_frame.data.empty()) {
                  try {
                    auto last_joint_contact_frame_new =
                        robot_link_name_to_franka_frame[human_contact_info
                                                            .joint_frame.data];

                    // If the frame in contact is in the list we store its value
                    // and change gains, when the operator doesnt touch anymore
                    // that frame, or touches one of the frame not in the list,
                    // the gains are switched again to the default values
                    if (!last_joint_contact_frame.has_value()) {
                      // Update md
                      if (std::find(frame_to_change_gains.begin(),
                                    frame_to_change_gains.end(),
                                    last_joint_contact_frame_new) !=
                          frame_to_change_gains.end()) {
                        set_md_kd(7.0);
                      }
                      last_joint_contact_frame = last_joint_contact_frame_new;
                    }
                  } catch (std::exception &ex) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Error calculating external force frame: %s",
                                 ex.what());
                    last_joint_contact_frame = std::nullopt;
                    set_md_kd(2.0);
                  }
                } else {
                  last_joint_contact_frame = std::nullopt;
                  set_md_kd(2.0);
                  std::this_thread::sleep_for(500ms);
                }
              } else {
                last_joint_contact_frame = std::nullopt;
                std::this_thread::sleep_for(1s);
              }
              std::this_thread::sleep_for(50ms);
            }
          }}.detach();

          // Debug publisher thread
          std::thread{[this]() {
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Started print control thread");
            while (start_flag.load() && rclcpp::ok()) {

              std::this_thread::sleep_for(5ms);

              try {

                debug_pub.data().mut.lock();
                debug_pub.publish(this->now());

              } catch (std::exception &ex) {
                RCLCPP_ERROR_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000.0,
                    "Error printing debug data: " << ex.what());
              }

              debug_pub.data().has_data = false;
              debug_pub.data().mut.unlock();
            }
            RCLCPP_INFO(this->get_logger(), "Shutdown print control thread");
          }}.detach();

          if (realtime_tools::configure_sched_fifo(99)) {
            RCLCPP_INFO(this->get_logger(), "Set real time priority");
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Real time priority not set, shutting down");
            start_flag.store(false);
            rclcpp::shutdown();
          }
          // clang-format on
          panda_franka->control(robot_control_callback);
        } catch (const franka::Exception &ex) {

          start_flag.store(false);
          if (panda_franka.has_value()) {
            try {
              panda_franka->stop();
            } catch (const franka::Exception &ex) {
              RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            }
          }
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
      }};

      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with real time robot");
      break;
    }
    case Mode::sim: {
      // Start control loop thread
      start_flag.store(true);
      control_thread =
          std::thread{std::bind(&ImpedanceController::control, this)};
      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with simulated robot");
      break;
    }
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    // Stop thread and join it
    start_flag.store(false);
    if (mode != Mode::sim) {
      if (panda_franka.has_value()) {
        try {
          panda_franka->stop();
        } catch (const franka::Exception &ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
        panda_franka.reset();
      }
    }
    if (control_thread.joinable()) {
      control_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    // Stop thread and join it
    start_flag.store(false);
    if (control_thread.joinable()) {
      control_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    // Stop thread and join it
    start_flag.store(false);
    if (mode != Mode::sim) {
      if (panda_franka.has_value()) {
        try {
          panda_franka->stop();
        } catch (const franka::Exception &ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
        panda_franka.reset();
      }
    }
    if (control_thread.joinable()) {
      control_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief This function publishes the static Tf representing the base link of
   * the robot wrt to the world Tf
   */
  void publish_static_transforms() {
    geometry_msgs::msg::TransformStamped world_to_base_transform_static;
    world_to_base_transform_static.header.stamp = this->now();
    world_to_base_transform_static.header.frame_id = "world";
    world_to_base_transform_static.child_frame_id = "fr3_link0";

    // Assuming world_base_link parameter order is x, y, z, w, x, y, z for
    // quaternion
    world_to_base_transform_static.transform.translation.x = world_base_link[0];
    world_to_base_transform_static.transform.translation.y = world_base_link[1];
    world_to_base_transform_static.transform.translation.z = world_base_link[2];

    world_to_base_transform_static.transform.rotation.w = world_base_link[3];
    world_to_base_transform_static.transform.rotation.x = world_base_link[4];
    world_to_base_transform_static.transform.rotation.y = world_base_link[5];
    world_to_base_transform_static.transform.rotation.z = world_base_link[6];

    static_tf_broadcaster->sendTransform(world_to_base_transform_static);
    RCLCPP_INFO(this->get_logger(),
                "Published static world -> fr3_link0 transform.");
  }

  /**
   * @brief Converts the transformation matrices of each frame into the
   * corresponding `PoseStamped` message and publishes it
   */
  void poses_publish_loop() {

    rclcpp::Rate rate(100.0);
    franka::RobotState current_state;
    rclcpp::Time now;

    while (start_flag.load() && rclcpp::ok()) {

      if (panda_franka_state.mut.try_lock() && panda_franka_model.has_value()) {
        if (panda_franka_state.state.has_value()) {
          current_state = panda_franka_state.state.value();
          now = panda_franka_state.state_time;
          panda_franka_state.state = std::nullopt;
        }
      }

      geometry_msgs::msg::PoseArray pose_arr;
      pose_arr.header.stamp = this->now();
      pose_arr.header.frame_id = robot_base_frame_name;

      for (auto const &[franka_enum_frame, child_link_name] :
           franka_frame_enum_to_link_name) {
        auto transform_matrix =
            panda_franka_model->pose(franka_enum_frame, current_state);
        geometry_msgs::msg::Pose abs_pose_msg =
            geom_utils::get_pose(transform_matrix);
        pose_arr.poses.push_back(abs_pose_msg);
      }

      panda_franka_state.mut.unlock();
      robot_frame_poses_pub->publish(pose_arr);
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "Poses publish loop stopped.");
  }

  ~ImpedanceController() {
    start_flag.store(false);
    if (mode != Mode::sim) {
      if (panda_franka.has_value()) {
        try {
          panda_franka->stop();
        } catch (const franka::Exception &ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
        panda_franka.reset();
      }
    }
    if (control_thread.joinable()) {
      control_thread.join();
    }
  }

private:
  // Mode of operation of the controller
  Mode mode = Mode::sim;
  // Subscribers
  rclcpp::Subscription<JointState>::SharedPtr robot_joint_states_sub{};
  rclcpp::Subscription<panda_interfaces::msg::CartesianCommand>::SharedPtr
      cartesian_cmd_sub{};
  rclcpp::Subscription<JointTorqueMeasureStamped>::SharedPtr external_tau_sub{};
  rclcpp::Subscription<panda_interfaces::msg::HumanContact>::SharedPtr
      human_contact_sub{};

  // Commands publisher
  // Used in simulation to control the robot
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub{};

  // Robot pose publisher and debug
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      robot_pose_pub{};
  realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_pub{};
  Publisher<geometry_msgs::msg::PoseArray>::SharedPtr robot_frame_poses_pub{};

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  // Clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_frame_publishing_client;

  // Robot related variables
  panda::RobotModel panda;
  // Robot object used to communicate with the FCI interface
  std::optional<franka::Robot> panda_franka;
  // Robot model given by the FCI
  std::optional<franka::Model> panda_franka_model;
  robot_state panda_franka_state;
  // Robot load variables
  double load = 0.0;
  std::array<double, 3> F_x_Cload{0.0, 0.0, 0.0};
  std::array<double, 9> load_inertia{0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0};
  panda_interfaces::msg::HumanContact human_contact_info{};
  std::optional<franka::Frame> last_joint_contact_frame{std::nullopt};
  Eigen::Quaterniond old_quaternion;
  Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();

  std::mutex joint_state_mutex;
  JointState::SharedPtr current_joint_config{nullptr};
  Eigen::Vector<double, 7> extern_tau{};
  Eigen::Vector<double, 7> extern_tau_filtered =
      Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 6> h_e{};
  Eigen::Vector<double, 6> h_e_measured{};
  std::vector<double> world_base_link;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      robot_control_callback;
  JointState joint_state_to_pub{};

  std::mutex desired_pose_mutex;
  std::mutex desired_twist_mutex;
  std::mutex desired_accel_mutex;
  Pose desired_pose{};
  Twist desired_twist{};
  Accel desired_accel{};
  Eigen::Vector<double, 6> desired_twist_vec = Eigen::Vector<double, 6>::Zero();
  Eigen::Vector<double, 6> desired_accel_vec = Eigen::Vector<double, 6>::Zero();

  Eigen::Vector<double, 7> current_joints_speed =
      Eigen::Vector<double, 7>::Zero();

  panda_interfaces::msg::CartesianCommand desired_cartesian_cmd{};
  std::mutex desired_cartesian_mutex;
  std::optional<panda_interfaces::msg::CartesianCommand>
      desired_cartesian_cmd_opt{std::nullopt};
  realtime_tools::RealtimeThreadSafeBox<panda_interfaces::msg::CartesianCommand>
      desired_cartesian_box{};

  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd effort_speed_limits{};
  Eigen::VectorXd joint_min_limits{};
  Eigen::VectorXd joint_max_limits{};
  Eigen::VectorXd velocity_limits{};
  Eigen::VectorXd acceleration_limits{};

  const std::string frame_id_name{"fr3_joint8"};
  const std::string robot_base_frame_name{"fr3_link0"};

  // Map Franka frames to child frame names for TF publishing
  std::map<franka::Frame, std::string> franka_frame_enum_to_link_name;
  std::map<std::string, franka::Frame> robot_link_name_to_franka_frame;

  // Control loop related variables
  rclcpp::Rate::SharedPtr control_loop_rate;
  double Kp{};
  double Kd{};
  double Md{};
  double Kp_rot{};
  double Kd_rot{};
  double Md_rot{};
  Eigen::Matrix<double, 6, 6> KP{};
  Eigen::Matrix<double, 6, 6> KD{};
  Eigen::Matrix<double, 6, 6> MD{};
  Eigen::Matrix<double, 6, 6> MD_1{};
  Eigen::Matrix<double, 7, 7> KD_J{};
  double joints_speed_cutoff_freq{30.0};
  double external_tau_cutoff_freq{30.0};
  double joint_speed_safe_limit{};
  double percentage_effort_safe_limit{};
  double error_pose_norm_safe_limit{};
  std::thread control_thread;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> compliance_mode{false};
  double compliance_switch_time{1.0};
  double gains_switch_time{1.0};
  bool clamp;
  Eigen::VectorXd last_control_input;

  // Human detection
  human_presence::HumanPresentState presence_state;

  void set_kp() {

    Eigen::Vector<double, 6> KP_{Kp, Kp, Kp, Kp_rot, Kp_rot, Kp_rot};
    Eigen::Matrix<double, 6, 6> final_KP =
        Eigen::Matrix<double, 6, 6>::Identity();
    final_KP.diagonal() = KP_;
    if (final_KP == KP) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Set KP: " << KP);
      return;
    }
    KP = Eigen::Matrix<double, 6, 6>::Zero();
    auto start = this->now();
    while ((this->now() - start).seconds() < gains_switch_time) {
      auto tau = (this->now() - start).seconds() / gains_switch_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
      KP = final_KP * tau;
      rclcpp::sleep_for(5ms);
    }
    KP = final_KP * 1.0;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set KP: " << KP);
  }

  void set_kd(double mul = 1.0) {
    Eigen::Vector<double, 6> KD_{Kd,           Kd,           Kd,
                                 Kd_rot * mul, Kd_rot * mul, Kd_rot * mul};
    Eigen::Matrix<double, 6, 6> final_KD =
        Eigen::Matrix<double, 6, 6>::Identity();
    final_KD.diagonal() = KD_;
    if (final_KD == KD) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Set KD: " << KD);
      return;
    }
    Eigen::Matrix<double, 6, 6> initial_KD = KD;
    auto start = this->now();
    while ((this->now() - start).seconds() < gains_switch_time) {
      auto tau = (this->now() - start).seconds() / gains_switch_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
      KD = initial_KD + tau * (final_KD - initial_KD);
      rclcpp::sleep_for(5ms);
    }
    KD = final_KD * 1.0;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set KD: " << KD);
  }

  void set_md(double mul = 1.0) {

    Eigen::Vector<double, 6> MD_{Md,           Md,           Md,
                                 Md_rot * mul, Md_rot * mul, Md_rot * mul};
    Eigen::Vector<double, 6> MD_1_{1.0 / Md,
                                   1.0 / Md,
                                   1.0 / Md,
                                   1.0 / (Md_rot * mul),
                                   1.0 / (Md_rot * mul),
                                   1.0 / (Md_rot * mul)};

    Eigen::Matrix<double, 6, 6> final_MD =
        Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> final_MD_1 =
        Eigen::Matrix<double, 6, 6>::Identity();
    final_MD.diagonal() = MD_;
    final_MD_1.diagonal() = MD_1_;
    if (final_MD == MD && final_MD_1 == MD_1) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Set MD: " << MD);
      return;
    }
    Eigen::Matrix<double, 6, 6> initial_MD = MD;
    Eigen::Matrix<double, 6, 6> initial_MD_1 = MD_1;
    auto start = this->now();
    while ((this->now() - start).seconds() < gains_switch_time) {
      auto tau = (this->now() - start).seconds() / gains_switch_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
      MD = initial_MD + tau * (final_MD - initial_MD);
      MD_1 = initial_MD_1 + tau * (final_MD_1 - initial_MD_1);
      rclcpp::sleep_for(5ms);
    }
    MD = final_MD * 1.0;
    MD_1 = final_MD_1 * 1.0;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set MD: " << MD);
  }

  /**
   * @brief Changes the value of the virtual mass and damping gains for the
   * rotational part. Can be easily expanded to the translational gains.
   *
   * @param double mul = 1.0
   */
  void set_md_kd(double mul = 1.0) {

    Eigen::Vector<double, 6> MD_{Md,           Md,           Md,
                                 Md_rot * mul, Md_rot * mul, Md_rot * mul};
    Eigen::Vector<double, 6> MD_1_{1.0 / Md,
                                   1.0 / Md,
                                   1.0 / Md,
                                   1.0 / (Md_rot * mul),
                                   1.0 / (Md_rot * mul),
                                   1.0 / (Md_rot * mul)};
    double eta = Kd_rot / (2 * std::sqrt(Kp_rot * Md_rot));
    double Kd_rot_mod = 2 * eta * std::sqrt(Kp_rot * Md_rot * mul);

    Eigen::Matrix<double, 6, 6> final_MD =
        Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> final_MD_1 =
        Eigen::Matrix<double, 6, 6>::Identity();
    final_MD.diagonal() = MD_;
    final_MD_1.diagonal() = MD_1_;

    Eigen::Vector<double, 6> KD_{Kd,         Kd,         Kd,
                                 Kd_rot_mod, Kd_rot_mod, Kd_rot_mod};
    Eigen::Matrix<double, 6, 6> final_KD =
        Eigen::Matrix<double, 6, 6>::Identity();
    final_KD.diagonal() = KD_;

    if (KD == final_KD && MD == final_MD && MD_1 == final_MD_1) {
      return;
    }

    Eigen::Matrix<double, 6, 6> initial_KD = KD;
    Eigen::Matrix<double, 6, 6> initial_MD = MD;
    Eigen::Matrix<double, 6, 6> initial_MD_1 = MD_1;

    auto start = this->now();
    while ((this->now() - start).seconds() < gains_switch_time) {
      auto tau = (this->now() - start).seconds() / gains_switch_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
      KD = initial_KD + tau * (final_KD - initial_KD);
      MD = initial_MD + tau * (final_MD - initial_MD);
      MD_1 = initial_MD_1 + tau * (final_MD_1 - initial_MD_1);
      rclcpp::sleep_for(5ms);
    }
    KD = final_KD * 1.0;
    MD = final_MD * 1.0;
    MD_1 = final_MD_1 * 1.0;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set MD: " << MD);
    RCLCPP_INFO_STREAM(this->get_logger(), "Set KD: " << KD);
  }

  void set_kd_j() {

    // Damping coefficient to handle joint drifts caused by 7th DOF not used
    // in control law
    double joint_damping = 10.0;
    // double effort_ratio = 12.0 / 87.0;
    double effort_ratio = 0.6;
    Eigen::Vector<double, 7> KD_J_{joint_damping,
                                   joint_damping,
                                   joint_damping,
                                   joint_damping,
                                   joint_damping * effort_ratio,
                                   joint_damping * effort_ratio,
                                   joint_damping * effort_ratio};
    Eigen::Matrix<double, 7, 7> final_KD_J =
        Eigen::Matrix<double, 7, 7>::Identity();
    final_KD_J.diagonal() = KD_J_;
    if (final_KD_J == KD_J) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Set KD_J: " << KD_J);
      return;
    }
    Eigen::Matrix<double, 7, 7> initial_KD_J = KD_J;
    auto start = this->now();
    while ((this->now() - start).seconds() < gains_switch_time) {
      auto tau = (this->now() - start).seconds() / gains_switch_time;
      if (tau > 1.0) {
        tau = 1.0;
      }
      KD_J = initial_KD_J + tau * (final_KD_J - initial_KD_J);
      rclcpp::sleep_for(5ms);
    }
    KD_J = final_KD_J * 1.0;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set KD_J: " << KD_J);
  }

  void publish_efforts(const Eigen::VectorXd &efforts) {
    JointsEffort efforts_cmd;
    for (int i = 0; i < efforts.size(); i++) {
      efforts_cmd.effort_values[i] = efforts[i];
    }
    efforts_cmd.header.stamp = this->now();
    robot_joint_efforts_pub->publish(efforts_cmd);
  }
  void control();

  void clamp_control(Eigen::Vector<double, 7> &control_input) {

    for (int i = 0; i < control_input.size(); i++) {
      if (control_input[i] > effort_limits[i]) {
        control_input[i] = effort_limits[i];
      } else if (control_input[i] < -effort_limits[i]) {
        control_input[i] = -effort_limits[i];
      }
    }
  }

  void clamp_control_speed(Eigen::Vector<double, 7> &control_input,
                           const double dt) {

    for (int i = 0; i < control_input.size(); i++) {
      if ((abs(control_input[i] - last_control_input[i]) / dt) >
          effort_speed_limits[i]) {
        if (control_input[i] - last_control_input[i] < 0.0) {
          control_input[i] =
              last_control_input[i] - effort_speed_limits[i] * dt;
        } else {
          control_input[i] =
              last_control_input[i] + effort_speed_limits[i] * dt;
        }
      }
    }
  }

  void clamp_vec(Eigen::VectorXd &vec, const Eigen::VectorXd &min_limits,
                 const Eigen::VectorXd &max_limits) {
    for (int i = 0; i < vec.size(); ++i) {
      vec[i] = std::min(std::max(vec[i], min_limits[i]), max_limits[i]);
    }
  }

  /**
   * @brief Load joint limits based on the `panda` variable, depending on the
   * URDF used for the robot. Loads the effort speed statically.
   */
  void load_joint_limits() {
    RCLCPP_INFO(this->get_logger(), "Getting effort speed limits");
    effort_limits = panda.getModel().effortLimit;

    RCLCPP_INFO(this->get_logger(), "Getting effort speed limits");
    effort_speed_limits.resize(panda.getModel().nq);
    for (int i = 0; i < panda.getModel().nq; i++) {
      effort_speed_limits[i] = 1000.0;
    }

    RCLCPP_INFO(this->get_logger(), "Getting joint pos limits");
    joint_min_limits = panda.getModel().lowerPositionLimit;
    joint_max_limits = panda.getModel().upperPositionLimit;
  }

  void init_cartesian_cmd(const std::optional<Eigen::Vector<double, 7>>
                              &current_joints_pos = std::nullopt) {
    Pose desired_pose;
    if (mode == Mode::franka) {
      // Initializing control setpoint variables
      if (!start_flag.load()) {
        desired_pose = geom_utils::get_pose(panda_franka_model.value().pose(
            franka::Frame::kFlange, panda_franka->readOnce()));
      } else if (current_joints_pos.has_value()) {
        franka::RobotState state;
        for (int i = 0; i < 7; i++) {
          state.q[i] = current_joints_pos.value()[i];
        }
        desired_pose = geom_utils::get_pose(
            panda_franka_model.value().pose(franka::Frame::kFlange, state));
      }
    } else {
      if (current_joints_pos.has_value()) {
        panda.computeAll(current_joints_pos.value(),
                         Eigen::Vector<double, 7>::Zero());
        desired_pose = panda.getPose(frame_id_name);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Mode franka/sim_franka but no joints pos specified");
        rclcpp::shutdown();
        return;
      }
    }

    desired_cartesian_cmd.pose = desired_pose;
    desired_cartesian_cmd.twist = desired_twist;
    desired_cartesian_cmd.accel = desired_accel;

    desired_twist_vec[0] = 0.0;
    desired_twist_vec[1] = 0.0;
    desired_twist_vec[2] = 0.0;
    desired_twist_vec[3] = 0.0;
    desired_twist_vec[4] = 0.0;
    desired_twist_vec[5] = 0.0;

    desired_accel_vec[0] = 0.0;
    desired_accel_vec[1] = 0.0;
    desired_accel_vec[2] = 0.0;
    desired_accel_vec[3] = 0.0;
    desired_accel_vec[4] = 0.0;
    desired_accel_vec[5] = 0.0;

    desired_cartesian_box.set(desired_cartesian_cmd);
  }

  void update_cartesian_cmd() {
    desired_cartesian_cmd_opt = desired_cartesian_box.try_get();
    if (desired_cartesian_cmd_opt.has_value()) {
      desired_cartesian_cmd = desired_cartesian_cmd_opt.value();

      desired_pose = desired_cartesian_cmd.pose;
      desired_twist = desired_cartesian_cmd.twist;
      desired_accel = desired_cartesian_cmd.accel;

      desired_twist_vec[0] = desired_twist.linear.x;
      desired_twist_vec[1] = desired_twist.linear.y;
      desired_twist_vec[2] = desired_twist.linear.z;
      desired_twist_vec[3] = desired_twist.angular.x;
      desired_twist_vec[4] = desired_twist.angular.y;
      desired_twist_vec[5] = desired_twist.angular.z;

      desired_accel_vec[0] = desired_accel.linear.x;
      desired_accel_vec[1] = desired_accel.linear.y;
      desired_accel_vec[2] = desired_accel.linear.z;
      desired_accel_vec[3] = desired_accel.angular.x;
      desired_accel_vec[4] = desired_accel.angular.y;
      desired_accel_vec[5] = desired_accel.angular.z;
    }
  }

  // Debug publisher object used to publish info on the network
  DebugPublisher debug_pub;
};

void ImpedanceController::control() {
  // Impedance controller

  Eigen::Vector<double, 7> control_input;
  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 6> y_cartesian;
  Eigen::Matrix<double, 7, 6> jacobian_pinv;
  Eigen::Vector<double, 7> current_joints_config_vec;
  Eigen::Vector<double, 7> current_joints_speed =
      Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 7> last_joints_speed = Eigen::Vector<double, 7>::Zero();
  Eigen::Matrix<double, 6, 7> jacobian;
  Eigen::Matrix<double, 6, 6> B_a;
  Eigen::Matrix<double, 7, 6> jacobian_cap;
  Eigen::Matrix<double, 7, 7> tau_projector;
  PoseStamped pose_debug;
  TwistStamped twist_debug;
  AccelStamped accel_debug;
  panda_interfaces::msg::DoubleStamped sigma;
  JointsEffort cmd;
  panda_interfaces::msg::DoubleArrayStamped arr_stamped;
  panda_interfaces::msg::DoubleArrayStamped y_cartesian_stamped;
  y_cartesian_stamped.data.resize(6);
  arr_stamped.data.resize(7);
  panda_interfaces::msg::DoubleStamped double_stamped;

  auto get_jacob = [this](const Eigen::Vector<double, 7> &current_joint_pos) {
    this->panda.computeAll(current_joint_pos, Eigen::Vector<double, 7>::Zero());
    return this->panda.getGeometricalJacobian(this->frame_id_name);
  };

  RCLCPP_INFO(this->get_logger(), "Waiting for simulation to start");
  rclcpp::Time last_control_cycle = this->now();
  while (rclcpp::Time{current_joint_config->header.stamp} - this->now() ==
         rclcpp::Duration{0, 0}) {
    RCLCPP_INFO(this->get_logger(), "Waiting for simulation to start");
  }
  RCLCPP_INFO(this->get_logger(), "Simulation started");

  // Get current pose in simulation environment
  current_joint_config = nullptr;
  while (!current_joint_config) {
    RCLCPP_INFO(this->get_logger(), "Waiting for joint config");
  }

  for (size_t i = 0; i < current_joint_config->position.size(); i++) {
    current_joints_config_vec[i] = current_joint_config->position[i];
  }
  init_cartesian_cmd(current_joints_config_vec);
  update_cartesian_cmd();

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Desired Position: ["
          << desired_pose.position.x << ", " << desired_pose.position.y << ", "
          << desired_pose.position.z << "]; Orientation: [] "
          << desired_pose.orientation.w << ", " << desired_pose.orientation.x
          << ", " << desired_pose.orientation.y << ", "
          << desired_pose.orientation.z << "]");

  set_kp();

  set_kd();

  set_md();

  set_kd_j();

  while (start_flag.load() && rclcpp::ok()) {

    pose_debug.header.stamp = this->now();
    twist_debug.header.stamp = this->now();
    accel_debug.header.stamp = this->now();
    arr_stamped.header.stamp = this->now();
    double_stamped.header.stamp = this->now();
    y_cartesian_stamped.header.stamp = this->now();
    sigma.header.stamp = this->now();

    update_cartesian_cmd();

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_config_vec[i] = current_joint_config->position[i];
      }

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_speed[i] = franka::lowpassFilter(
            0.001, current_joint_config->velocity[i], current_joints_speed[i],
            joints_speed_cutoff_freq);
      }
    }

    panda.computeAll(current_joints_config_vec, current_joints_speed);

    if (compliance_mode.load()) {
      jacobian = panda.getGeometricalJacobian(frame_id_name);
    } else {
      jacobian = panda.getGeometricalJacobian(frame_id_name);
    }

    // Get current pose
    Pose current_pose_tmp = panda.getPose(frame_id_name);
    Eigen::Quaterniond current_quat{};
    current_quat.w() = current_pose_tmp.orientation.w;
    current_quat.x() = current_pose_tmp.orientation.x;
    current_quat.y() = current_pose_tmp.orientation.y;
    current_quat.z() = current_pose_tmp.orientation.z;
    current_quat.normalize();
    current_quat = quaternionContinuity(current_quat, old_quaternion);
    old_quaternion = current_quat;

    Eigen::Quaterniond desired_quat{};
    Eigen::Quaterniond error_quat{};
    Eigen::Vector<double, 6> error_pose_vec{};
    // Calculate pose error
    {
      std::lock_guard<std::mutex> lock(desired_pose_mutex);

      desired_quat.w() = desired_pose.orientation.w;
      desired_quat.x() = desired_pose.orientation.x;
      desired_quat.y() = desired_pose.orientation.y;
      desired_quat.z() = desired_pose.orientation.z;
      desired_quat.normalize();

      error_quat = desired_quat * current_quat.inverse();
      error_quat.normalize();

      error_pose_vec(0) = desired_pose.position.x - current_pose_tmp.position.x;
      error_pose_vec(1) = desired_pose.position.y - current_pose_tmp.position.y;
      error_pose_vec(2) = desired_pose.position.z - current_pose_tmp.position.z;

      error_pose_vec(3) = error_quat.x();
      error_pose_vec(4) = error_quat.y();
      error_pose_vec(5) = error_quat.z();
    }

    // Calculate jacobian SVD

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);

    double sigma_min = svd.singularValues().tail(1)(0);

    // Doesn't account for friction
    Eigen::Vector<double, 7> non_linear_effects = panda.getNonLinearEffects(
        current_joints_config_vec, current_joints_speed);
    Eigen::Vector<double, 7> gravity =
        panda.getGravityVector(current_joints_config_vec);
    Eigen::Matrix<double, 7, 7> mass_matrix =
        panda.getMassMatrix(current_joints_config_vec);

    // Quadratic

    jacobian_pinv = compute_jacob_pseudoinv(jacobian);

    Eigen::Vector<double, 6> current_twist, error_twist;
    {
      std::lock_guard<std::mutex> lock(desired_twist_mutex);
      current_twist = jacobian * current_joints_speed;
      error_twist = desired_twist_vec - current_twist;
    }

    for (int i = 0; i < 7; i++) {
      extern_tau_filtered[i] =
          franka::lowpassFilter(0.001, extern_tau[i], extern_tau_filtered[i],
                                external_tau_cutoff_freq);
    }
    h_e = jacobian_pinv.transpose() * extern_tau_filtered;
    {
      std::lock_guard<std::mutex> lock(desired_accel_mutex);
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "h_e: " << h_e);

      // clang-format off
      if (compliance_mode.load()) {

        y = jacobian_pinv * MD_1 *
            (
              -KD * current_twist 
             - jacobian_pinv.transpose() * extern_tau
              - MD *
                   get_j_dot(get_jacob, current_joints_config_vec,
                             current_joints_speed) *
                   current_joints_speed

            );
      } else {
        y_cartesian =
            (
             MD * desired_accel_vec +
             KD * error_twist + KP * error_pose_vec -
             MD *
                 get_j_dot(get_jacob, current_joints_config_vec,
                           current_joints_speed) *
                 current_joints_speed
             - compute_jacob_pseudoinv_h_e(jacobian.transpose()) * extern_tau
        );

        y = jacobian_pinv * MD_1 * y_cartesian;
      }
      // clang-format on
    }

    control_input = mass_matrix * y + non_linear_effects + extern_tau - gravity;

    // Clamping control input
    //
    clamp_control(control_input);

    // Apply control
    //
    publish_efforts(control_input + gravity);

    // Save last control input
    //
    last_control_input = control_input;
    last_control_cycle = this->now();

    pose_debug.pose = current_pose_tmp;

    if (debug_pub.data().mut.try_lock()) {

      debug_pub.data().h_e = h_e;
      debug_pub.data().h_e_calculated =
          compute_jacob_pseudoinv_h_e(jacobian.transpose()) * extern_tau;
      debug_pub.data().tau_ext = extern_tau;
      debug_pub.data().sigma_min = sigma_min;
      debug_pub.data().current_twist = current_twist;
      debug_pub.data().current_pose = current_pose_tmp;
      debug_pub.data().des_twist = desired_twist;
      debug_pub.data().des_pose = desired_pose;
      debug_pub.data().des_accel = desired_accel;
      debug_pub.data().error_pose_vec.head(3) = error_pose_vec.head(3);
      debug_pub.data().error_pose_vec.tail(3) = error_pose_vec.tail(3);
      debug_pub.data().error_pose_vec(3) = 1.0;
      for (int i = 0; i < gravity.size(); i++) {
        debug_pub.data().gravity.value()[i] = gravity[i];
      }
      for (int i = 0; i < control_input.size(); i++) {
        debug_pub.data().tau_d_last.value()[i] = control_input[i];
      }
      for (int i = 0; i < control_input.size(); i++) {
        debug_pub.data().tau_d_calculated.value()[i] = control_input[i];
      }
      debug_pub.data().y = y;
      debug_pub.data().y_cartesian = y_cartesian;

      // Robot state
      for (int i = 0; i < current_joints_config_vec.size(); i++) {
        debug_pub.data().robot_state.value().q[i] =
            current_joints_config_vec[i];
      }
      for (int i = 0; i < current_joints_speed.size(); i++) {
        debug_pub.data().robot_state.value().dq[i] = current_joints_speed[i];
      }
      debug_pub.data().robot_state.value().O_T_EE =
          geom_utils::get_transform_matrix(current_pose_tmp);

      debug_pub.data().has_data = true;
      debug_pub.data().mut.unlock();
    }

    debug_pub.publish(this->now());

    // Sleep
    //
    control_loop_rate->sleep();
  }

  if (!rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Requested shutdown");
    return;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImpedanceController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
