#include "algorithm/jacobian.hpp"
#include "franka/control_types.h"
#include "franka/exception.h"
#include "franka/rate_limiting.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "multibody/fwd.hpp"
#include "panda_interfaces/msg/joint_torque_measure_stamped.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spatial/explog-quaternion.hpp"
#include "spatial/fwd.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <franka/model.h>
#include <franka/robot.h>
#include <memory>
#include <mutex>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
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
#include <realtime_tools/realtime_tools/realtime_publisher.hpp>
#include <thread>

template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using panda_interfaces::msg::JointsCommand;
using panda_interfaces::msg::JointsEffort;
using panda_interfaces::msg::JointsPos;
using panda_interfaces::msg::JointTorqueMeasureStamped;
using sensor_msgs::msg::JointState;

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort;

struct debug_data {
  std::mutex mut;
  bool has_data;
  std::array<double, 7> tau_d_last;
  franka::RobotState robot_state;
  std::array<double, 7> gravity;
};

geometry_msgs::msg::Pose
convertMatrixToPose(const std::array<double, 16> &tf_matrix) {
  Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(tf_matrix.data());

  Eigen::Vector3d translation = T.block<3, 1>(0, 3);

  Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);

  Eigen::Quaterniond quat(rotation);
  quat.normalize();

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = translation.x();
  pose_msg.position.y = translation.y();
  pose_msg.position.z = translation.z();

  pose_msg.orientation.x = quat.x();
  pose_msg.orientation.y = quat.y();
  pose_msg.orientation.z = quat.z();
  pose_msg.orientation.w = quat.w();

  return pose_msg;
}

class InverseDynamicsController : public rclcpp_lifecycle::LifecycleNode {

public:
  InverseDynamicsController(
      const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::inverse_dynamics_controller_node_name),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 700.0);
    this->declare_parameter<double>("Kd", 50.0);
    this->declare_parameter<double>("Md", 1.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<bool>("clamp", true);
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<bool>("use_robot", false);
    this->declare_parameter<bool>("use_franka_sim", false);

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();
    use_robot = this->get_parameter("use_robot").as_bool();
    use_franka_sim = this->get_parameter("use_franka_sim").as_bool();

    if (use_robot) {
      panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Connected to robot with ip "
                             << this->get_parameter("robot_ip").as_string());
      double load = 0.553455;
      std::array F_x_Cload{-0.010328, 0.000068, 0.148159};
      std::array load_inertia{0.02001,        0.000006527121, -0.0004590,
                              0.000006527121, 0.01936,        0.000003371038,
                              -0.0004590,     0.000003371038, 0.002245};
      panda_franka->setLoad(load, F_x_Cload, load_inertia);
    }

    // Taking joint limits
    // WARNING: Joint limits are defined in the urdf, change their value into
    // the urdf

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

    // joint_min_limits = Eigen::Vector<double, 7>{
    //     -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    //
    // joint_max_limits = Eigen::Vector<double, 7>{
    //   2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

    RCLCPP_INFO(this->get_logger(), "Getting velocity limits");
    velocity_limits = panda.getModel().velocityLimit;
    // velocity_limits = Eigen::Vector<double,
    // 7>{2.1750, 2.1750, 2.1750, 2.1750,
    //                                            2.6100, 2.6100, 2.6100};

    RCLCPP_INFO(this->get_logger(), "Getting acceleration limits");
    // acceleration_limits.resize(panda.getModel().nq);
    // for (int i = 0; i < panda.getModel().nq; i++) {
    //   acceleration_limits[i] = 10.0;
    // }
    acceleration_limits =
        Eigen::Vector<double, 7>{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

    auto set_joint_state = [this](const JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      current_joint_config = msg;
    };

    robot_joint_states_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS(), set_joint_state);

    auto set_joint_measured_torques =
        [this](const JointTorqueMeasureStamped::SharedPtr msg) {
          current_measured_joint_torques = msg;
        };

    robot_measured_torque_sub =
        this->create_subscription<JointTorqueMeasureStamped>(
            panda_interface_names::torque_sensor_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS(),
            set_joint_measured_torques);

    auto set_desired_joints_command =
        [this](const JointsCommand::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(desired_cmd_mutex);
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Received joint command");

          desired_joints_position.resize(msg->positions.size());
          for (size_t i = 0; i < msg->positions.size(); i++) {
            desired_joints_position[i] = msg->positions[i];
          }

          desired_joints_velocity.resize(msg->velocities.size());
          for (size_t i = 0; i < msg->velocities.size(); i++) {
            desired_joints_velocity[i] = msg->velocities[i];
          }

          desired_joints_accelerations.resize(msg->accelerations.size());
          for (size_t i = 0; i < msg->accelerations.size(); i++) {
            desired_joints_accelerations[i] = msg->accelerations[i];
          }

          clamp_joint_config();
        };

    desired_joint_command_sub = this->create_subscription<JointsCommand>(
        panda_interface_names::panda_joint_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS(), set_desired_joints_command);

    robot_joint_efforts_pub = this->create_publisher<JointsEffort>(
        panda_interface_names::panda_effort_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS());

    robot_joint_efforts_pub_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/effort_no_gravity",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    gravity_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/gravity", panda_interface_names::DEFAULT_TOPIC_QOS());

    position_error_debug = this->create_publisher<JointsPos>(
        "debug/error/joint_position",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    velocity_error_debug = this->create_publisher<JointsPos>(
        "debug/error/joint_velocity",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    desired_position_debug = this->create_publisher<JointsPos>(
        "debug/desired_joint_position",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    desired_velocity_debug = this->create_publisher<JointsPos>(
        "debug/desired_joint_velocity",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    desired_acceleration_debug = this->create_publisher<JointsPos>(
        "debug/desired_joint_acceleration",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    current_position_debug = this->create_publisher<JointsPos>(
        "debug/current_joint_position",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    current_velocity_debug = this->create_publisher<JointsPos>(
        "debug/current_joint_velocity",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    y_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/y_contribute", panda_interface_names::DEFAULT_TOPIC_QOS());

    y_desired_accel_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/y_contribute_desired_accel",
        panda_interface_names::DEFAULT_TOPIC_QOS());

    mass_matrix_val_debug = this->create_publisher<std_msgs::msg::Float64>(
        "debug/mass_matrix_norm", panda_interface_names::DEFAULT_TOPIC_QOS());

    // Compliance mode Service

    auto compliance_mode_cb =
        [this](const panda_interfaces::srv::SetComplianceMode_Request::SharedPtr
                   request,
               panda_interfaces::srv::SetComplianceMode_Response::SharedPtr
                   response) {
          if (request->cmd) {
            compliance_mode.store(true);
            response->result = true;
            RCLCPP_INFO(this->get_logger(),
                        "Set controller to compliance mode");
          } else {
            compliance_mode.store(false);
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "Unset controller compliance mode");
          }
        };

    compliance_mode_server =
        this->create_service<panda_interfaces::srv::SetComplianceMode>(
            panda_interface_names::set_compliance_mode_service_name,
            compliance_mode_cb);

    // robot_pose_pub = std::make_shared<
    //     realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    //     this->create_publisher<geometry_msgs::msg::PoseStamped>(
    //         panda_interface_names::panda_pose_state_topic_name,
    //         panda_interface_names::DEFAULT_TOPIC_QOS()));

    joint_states_pub =
        std::make_shared<realtime_tools::RealtimePublisher<JointState>>(
            this->create_publisher<JointState>(
                panda_interface_names::joint_state_topic_name,
                panda_interface_names::DEFAULT_TOPIC_QOS()));
  }

  ~InverseDynamicsController() {
    if (control_thread.joinable()) {
      start_flag.store(false);
      control_thread.join();
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {

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
            << " with Kp = " << Kp << ",Kd = " << Kd << " and Md = " << Md);

    Eigen::Vector3d translation = Eigen::Vector3d{0, 0, 0.75};
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    T_0_b = pinocchio::SE3{orientation.toRotationMatrix(), translation};

    if (use_franka_sim) {
      RCLCPP_INFO(this->get_logger(),
                  "Started controller with libfranka model to load");
      // Not setting real time prioriry when required
      franka::Robot robot =
          franka::Robot(this->get_parameter("robot_ip").as_string(),
                        franka::RealtimeConfig::kIgnore);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Connected to robot with ip "
                             << this->get_parameter("robot_ip").as_string());
      panda_franka_model = robot.loadModel();
      RCLCPP_INFO(this->get_logger(), "Loaded robot model library");
    }

    if (use_robot) {
      panda_franka_model = panda_franka.value().loadModel();
      joint_state_to_pub.position.resize(7);
      joint_state_to_pub.velocity.resize(7);
      joint_state_to_pub.effort.resize(7);

      Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp, Kp, Kp};
      // Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp * 3, Kp * 3, Kp * 4};
      Eigen::Matrix<double, 7, 7> KP = Eigen::Matrix<double, 7, 7>::Identity();
      KP.diagonal() = KP_;

      Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd, Kd, Kd};
      // Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd * 3, Kd * 3, Kd * 4};
      Eigen::Matrix<double, 7, 7> KD = Eigen::Matrix<double, 7, 7>::Identity();
      KD.diagonal() = KD_;

      robot_control_callback = [this, KP,
                                KD](const franka::RobotState &state,
                                    franka::Duration dt) -> franka::Torques {
        // If the thread flag is changed or a shutdown has been requested,
        // always returns last command
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        // Get q and q_dot
        //
        Eigen::Vector<double, 7> current_joints_config_vec;
        for (size_t i = 0; i < 7; i++) {
          current_joints_config_vec[i] = state.q[i];
        }

        Eigen::Vector<double, 7> current_joints_speed;
        for (size_t i = 0; i < 7; i++) {
          current_joints_speed[i] = state.dq[i];
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
        // B(q)
        std::array<double, 49> mass_matrix_raw =
            this->panda_franka_model.value().mass(state);
        Eigen::Matrix<double, 7, 7> mass_matrix;
        for (size_t i = 0; i < 7; i++) {
          for (size_t j = 0; j < 7; j++) {
            mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
          }
        }

        // Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
        //     this->panda_franka_model.value().coriolis(state).data());
        std::array<double, 7> coriolis_raw =
            this->panda_franka_model.value().coriolis(state);
        Eigen::Vector<double, 7> coriolis;
        for (size_t i = 0; i < 7; i++) {
          coriolis(i) = coriolis_raw[i];
        }

        Eigen::Vector<double, 7> y;
        // If compliance mode: Kp = 0
        if (compliance_mode.load()) {
          y = -KD * current_joints_speed;
        } else {
          y = desired_joints_accelerations +
              KD * (desired_joints_velocity - current_joints_speed) +
              KP * (desired_joints_position - current_joints_config_vec);
        }

        // Inverse dynamics torque commanded
        Eigen::Vector<double, 7> control_input_vec = mass_matrix * y + coriolis;

        // Clamping control input
        // Torque limits for fr3 indicated by libfranka lib
        //
        // https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3
        //
        // The value of last control input on first iteration is 0. Assuming a
        // fake cycle before the first one with 1 kHz freq
        //////////////////////////////////////////////////////////////////////
        // WARN: tau speed not clamped //////////////////////////////////
        //////////////////////////////////////////////////////////////////////
        // Clamp tau_dot
        // if (dt.toSec() == 0.0) {
        //   clamp_control_speed(control_input_vec, 1.0 / 1e3);
        // } else {
        //   clamp_control_speed(control_input_vec, dt.toSec());
        // }

        // Clamp tau
        clamp_control(control_input_vec);

        try {
          for (int i = 0; i < control_input_vec.size(); i++) {
            if (abs(control_input_vec[i]) >= 6.0 * effort_limits[i] / 10.0) {
              RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                      "Running safety check: effort limit");
              RCLCPP_ERROR_STREAM(this->get_logger(),
                                  "Torque abs value over limit (60%)");
              panda_franka->stop();
              start_flag.store(false);
              return franka::MotionFinished(franka::Torques(state.tau_J_d));
            }
          }
        } catch (std::exception &ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "Error in safety checks: " << ex.what());
          panda_franka->stop();
          start_flag.store(false);
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        // Apply control

        last_control_input = control_input_vec;
        std::array<double, 7> tau;
        for (size_t i = 0; i < 7; ++i) {
          tau[i] = control_input_vec[i];
        }

        // Fill struct for debug prints
        if (print_debug.mut.try_lock()) {
          print_debug.has_data = true;
          print_debug.robot_state = state;
          print_debug.tau_d_last = tau;
          print_debug.gravity = panda_franka_model.value().gravity(state);
          print_debug.mut.unlock();
        }

        // joint_state_to_pub.header.stamp = this->now();
        // for (size_t i = 0; i < 7; i++) {
        //   joint_state_to_pub.position[i] = state.q[i];
        //   joint_state_to_pub.velocity[i] = state.dq[i];
        //   joint_state_to_pub.effort[i] = state.tau_J[i];
        // }
        //
        // joint_states_pub->tryPublish(joint_state_to_pub);

        return franka::Torques(tau);
      };
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {

    RCLCPP_INFO(get_logger(), "Activating...");
    /////////////////////////////////////////////////////////////////////////////////
    // WARN: In activate can't receive callback messages. Setting current config
    // to 0 vector (based on joint limits)
    desired_joints_position.resize(panda.getModel().nq);
    desired_joints_position.setZero();
    desired_joints_velocity.resize(panda.getModel().nq);
    desired_joints_velocity.setZero();
    desired_joints_accelerations.resize(panda.getModel().nq);
    desired_joints_accelerations.setZero();

    clamp_joint_config();

    last_control_input.resize(panda.getModel().nq);
    for (int i = 0; i < last_control_input.size(); i++) {
      // The last control input is indeed 0 when activated the first time,
      // either with gazebo or libfranka lib, that considers only the tau_d as
      // user commanded tau
      last_control_input[i] = 0.0;
    }

    if (use_robot) {

      // Setting initial state to the current one of the robot if using real
      // robot
      franka::RobotState initial_state = panda_franka->readOnce();
      for (size_t i = 0; i < 7; i++) {
        desired_joints_position[i] = initial_state.q[i];
      }
      RCLCPP_INFO(this->get_logger(), "Set initial state with real time robot");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial joint state: ["
                             << initial_state.q[0] << ", " << initial_state.q[1]
                             << ", " << initial_state.q[2] << ", "
                             << initial_state.q[3] << ", " << initial_state.q[4]
                             << ", " << initial_state.q[5] << ", "
                             << initial_state.q[6] << "]");

      start_flag.store(true);
      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(this->get_logger(),
                     "The robot thread has no real time kernel, shutting down");
        start_flag.store(false);
        rclcpp::shutdown();
      }
      // Try-catch version
      control_thread = std::thread{[this]() {
        // Configuring real time thread
        if (realtime_tools::configure_sched_fifo(99)) {
          RCLCPP_INFO(this->get_logger(), "Set real time priority");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Real time priority not set, shutting down");
          start_flag.store(false);
          rclcpp::shutdown();
        }

        try {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Starting control thread with real time robot");
          std::thread{[this]() {
            JointsEffort cmd;
            JointsPos pos;
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Started print control thread");
            while (start_flag.load()) {
              std::this_thread::sleep_for(1ms);
              if (this->print_debug.mut.try_lock()) {
                if (print_debug.has_data) {

                  // Publish current pose and joint state
                  publish_robot_state_libfranka(print_debug.robot_state);

                  cmd.header.stamp = this->now();
                  pos.header.stamp = this->now();
                  for (int i = 0; i < 7; i++) {
                    cmd.effort_values[i] = print_debug.tau_d_last[i];
                  }
                  robot_joint_efforts_pub_debug->publish(cmd);

                  for (int i = 0; i < 7; i++) {
                    cmd.effort_values[i] = print_debug.gravity[i];
                  }
                  gravity_contribute_debug->publish(cmd);

                  // for (int i = 0; i < y.size(); i++) {
                  //   cmd.effort_values[i] = y[i];
                  // }
                  // y_contribute_debug->publish(cmd);

                  // for (int i = 0; i < y.size(); i++) {
                  //   cmd.effort_values[i] = y_accel[i];
                  // }
                  // y_desired_accel_debug->publish(cmd);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = desired_joints_position[i] -
                                          print_debug.robot_state.q[i];
                  }
                  position_error_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = desired_joints_velocity[i] -
                                          print_debug.robot_state.dq[i];
                  }
                  velocity_error_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = print_debug.robot_state.q[i];
                  }
                  current_position_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = print_debug.robot_state.dq[i];
                  }
                  current_velocity_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = desired_joints_position[i];
                  }
                  desired_position_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = desired_joints_velocity[i];
                  }
                  desired_velocity_debug->publish(pos);

                  for (int i = 0; i < 7; i++) {
                    pos.joint_values[i] = desired_joints_accelerations[i];
                  }
                  desired_acceleration_debug->publish(pos);

                  print_debug.has_data = false;
                }

                print_debug.mut.unlock();
              }
            }
            RCLCPP_INFO(this->get_logger(), "Shutdown print control thread");
          }}.detach();

          // control function says "sets realtime priority for the current
          // thread"
          panda_franka->control(robot_control_callback);
        } catch (const franka::Exception &ex) {
          start_flag.store(false);
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
      }};

      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with real time robot");
      return CallbackReturn::SUCCESS;
    } else {
      // Start control loop thread
      start_flag.store(true);
      if (use_franka_sim) {
        control_thread = std::thread{
            std::bind(&InverseDynamicsController::control_libfranka_sim, this)};
        RCLCPP_INFO(this->get_logger(), "Started control thread with simulated "
                                        "robot and libfranka model lib");
      } else {

        control_thread =
            std::thread{std::bind(&InverseDynamicsController::control, this)};
        RCLCPP_INFO(this->get_logger(),
                    "Started control thread with simulated robot");
      }

      return CallbackReturn::SUCCESS;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    // Stop thread and join it
    start_flag.store(false);
    if (control_thread.joinable()) {
      control_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    // Stop thread and join it
    start_flag.store(false);
    if (control_thread.joinable()) {
      control_thread.join();
    }
    return CallbackReturn::SUCCESS;
  }

private:
  // Subscribers
  rclcpp::Subscription<JointState>::SharedPtr robot_joint_states_sub{};
  rclcpp::Subscription<JointsCommand>::SharedPtr desired_joint_command_sub{};
  rclcpp::Subscription<JointTorqueMeasureStamped>::SharedPtr
      robot_measured_torque_sub{};

  // Commands publisher
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub{};
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub_debug{};
  Publisher<JointsEffort>::SharedPtr gravity_contribute_debug{};
  Publisher<JointsEffort>::SharedPtr y_contribute_debug{};
  Publisher<JointsEffort>::SharedPtr y_desired_accel_debug{};
  Publisher<JointsPos>::SharedPtr position_error_debug{};
  Publisher<JointsPos>::SharedPtr velocity_error_debug{};
  Publisher<JointsPos>::SharedPtr desired_position_debug{};
  Publisher<JointsPos>::SharedPtr desired_velocity_debug{};
  Publisher<JointsPos>::SharedPtr desired_acceleration_debug{};
  Publisher<JointsPos>::SharedPtr current_position_debug{};
  Publisher<JointsPos>::SharedPtr current_velocity_debug{};
  Publisher<std_msgs::msg::Float64>::SharedPtr mass_matrix_val_debug{};

  // Robot pose publisher
  // realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  //     robot_pose_pub{};
  realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_pub{};

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  // Robot related variables
  panda::RobotModel panda;
  std::optional<franka::Robot> panda_franka;
  std::optional<franka::Model> panda_franka_model;
  bool use_robot;
  bool use_franka_sim;
  JointState::SharedPtr current_joint_config{nullptr};
  std::mutex joint_state_mutex;
  JointState joint_state_to_pub{};
  JointTorqueMeasureStamped::SharedPtr current_measured_joint_torques{nullptr};
  debug_data print_debug;
  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      robot_control_callback;

  std::mutex desired_cmd_mutex;
  Eigen::VectorXd desired_joints_position;
  Eigen::VectorXd desired_joints_velocity;
  Eigen::VectorXd desired_joints_accelerations;

  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd effort_speed_limits{};
  Eigen::Vector<double, 7> joint_min_limits{};
  Eigen::Vector<double, 7> joint_max_limits{};
  Eigen::Vector<double, 7> velocity_limits{};
  Eigen::Vector<double, 7> acceleration_limits{};

  const std::string frame_id_name{"fr3_link7"};

  pinocchio::SE3 T_0_b{};

  // Control loop related variables
  rclcpp::Rate::SharedPtr control_loop_rate;
  double Kp{};
  double Kd{};
  double Md{};
  std::thread control_thread;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> compliance_mode{false};
  bool clamp;
  Eigen::VectorXd last_control_input;

  void publish_efforts(const Eigen::VectorXd &efforts) {
    JointsEffort efforts_cmd;
    for (int i = 0; i < efforts.size(); i++) {
      efforts_cmd.effort_values[i] = efforts[i];
    }
    efforts_cmd.header.stamp = this->now();
    robot_joint_efforts_pub->publish(efforts_cmd);
  }
  void control();
  void control_libfranka_sim();

  void clamp_control(Eigen::Vector<double, 7> &control_input) {

    for (int i = 0; i < control_input.size(); i++) {
      if (control_input[i] > effort_limits[i]) {
        control_input[i] = effort_limits[i];
      } else if (control_input[i] < -effort_limits[i]) {
        control_input[i] = -effort_limits[i];
      }
    }
  }

  void clamp_control_speed(Eigen::VectorXd &control_input, const double dt) {

    for (int i = 0; i < control_input.size(); i++) {
      if (abs(control_input[i] - last_control_input[i]) / dt >
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

  void clamp_vec(Eigen::VectorXd &vec,
                 const Eigen::Vector<double, 7> &min_limits,
                 const Eigen::Vector<double, 7> &max_limits) {
    for (int i = 0; i < vec.size(); ++i) {
      vec[i] = std::min(std::max(vec[i], min_limits[i]), max_limits[i]);
    }
  }

  void clamp_joint_config() {

    for (int i = 0; i < desired_joints_position.size(); ++i) {
      desired_joints_position[i] =
          std::min(std::max(desired_joints_position[i], joint_min_limits[i]),
                   joint_max_limits[i]);
      // Clamp associated joint speed and acceleration if joint position
      // saturated
      if (desired_joints_position[i] == joint_min_limits[i] ||
          desired_joints_position[i] == joint_max_limits[i]) {
        desired_joints_velocity[i] = 0.0;
        desired_joints_accelerations[i] = 0.0;
      }
    }
    clamp_vec(desired_joints_velocity, -velocity_limits, velocity_limits);

    clamp_vec(desired_joints_accelerations, -acceleration_limits,
              acceleration_limits);
    // RCLCPP_INFO_STREAM(
    //     this->get_logger(),
    //     "Desired configuration after clamping: ["
    //         << desired_joints_position[0] << ", " <<
    //         desired_joints_position[1]
    //         << ", " << desired_joints_position[2] << ", "
    //         << desired_joints_position[3] << ", " <<
    //         desired_joints_position[4]
    //         << ", " << desired_joints_position[5] << ", "
    //         << desired_joints_position[6] << "]\n"
    //         << "[" << desired_joints_velocity[0] << ", "
    //         << desired_joints_velocity[1] << ", " <<
    //         desired_joints_velocity[2]
    //         << ", " << desired_joints_velocity[3] << ", "
    //         << desired_joints_velocity[4] << ", " <<
    //         desired_joints_velocity[5]
    //         << ", " << desired_joints_velocity[6] << "]\n"
    //         << "[" << desired_joints_accelerations[0] << ", "
    //         << desired_joints_accelerations[1] << ", "
    //         << desired_joints_accelerations[2] << ", "
    //         << desired_joints_accelerations[3] << ", "
    //         << desired_joints_accelerations[4] << ", "
    //         << desired_joints_accelerations[5] << ", "
    //         << desired_joints_accelerations[6] << "]\n");
  }

  void publish_robot_state_libfranka(const franka::RobotState state) {

    // Publish robot pose stamped
    geometry_msgs::msg::PoseStamped current_pose;
    // WARN: Verify that this is the right frame considered in simulation
    current_pose.pose = convertMatrixToPose(
        panda_franka_model.value().pose(franka::Frame::kFlange, state));
    // current_pose.pose = convertMatrixToPose(state.O_T_EE);
    current_pose.header.stamp = this->now();
    // robot_pose_pub->tryPublish(current_pose);

    // Publish joint state
    joint_state_to_pub.header.stamp = this->now();
    for (size_t i = 0; i < 7; i++) {
      joint_state_to_pub.position[i] = state.q[i];
      joint_state_to_pub.velocity[i] = state.dq[i];
      joint_state_to_pub.effort[i] = state.tau_J[i];
    }

    joint_states_pub->tryPublish(joint_state_to_pub);
  }
};

void InverseDynamicsController::control() {
  // Inverse dynamics controller
  //
  //

  Eigen::Vector<double, 7> non_linear_effects;
  Eigen::Vector<double, 7> gravity;
  Eigen::Vector<double, 7> control_input;
  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 7> err_pos;
  Eigen::Vector<double, 7> err_vel;
  Eigen::Matrix<double, 7, 7> mass_matrix;
  Eigen::Vector<double, 7> y_accel;
  Eigen::Vector<double, 7> current_joints_config_vec;
  Eigen::Vector<double, 7> current_joints_speed;

  JointsEffort cmd;
  JointsPos pos;
  std_msgs::msg::Float64 mass;

  Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp * 3, Kp * 3, Kp * 4};
  // Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp, Kp, Kp};
  Eigen::Matrix<double, 7, 7> KP = Eigen::Matrix<double, 7, 7>::Identity();
  KP.diagonal() = KP_;

  Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd * 3, Kd * 3, Kd * 4};
  // Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd, Kd, Kd};
  Eigen::Matrix<double, 7, 7> KD = Eigen::Matrix<double, 7, 7>::Identity();
  KD.diagonal() = KD_;

  // RCLCPP_INFO(this->get_logger(), "Waiting for simulation to start");
  // rclcpp::Time last_control_cycle = this->now();
  // while (rclcpp::Time{current_joint_config->header.stamp} - this->now() ==
  //        rclcpp::Duration{0, 0}) {
  // }
  // for (size_t i = 0; i < 7; i++) {
  //   desired_joints_position[i] = current_joint_config->position[i];
  // }

  desired_joints_position.setZero();
  clamp_joint_config();
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Current desired config at initial loop: ["
          << desired_joints_position[0] << ", " << desired_joints_position[1]
          << ", " << desired_joints_position[2] << ", "
          << desired_joints_position[3] << ", " << desired_joints_position[4]
          << ", " << desired_joints_position[5] << ", "
          << desired_joints_position[6] << "]");

  while (start_flag.load() && rclcpp::ok()) {

    pos.header.stamp = this->now();

    // Update robot model
    //

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_config_vec[i] = current_joint_config->position[i];
      }

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_speed[i] = current_joint_config->velocity[i];
      }
    }

    panda.computeAll(current_joints_config_vec, current_joints_speed);

    // Calculate quantities for control
    //

    // Doesn't account for friction
    non_linear_effects = panda.getNonLinearEffects(current_joints_config_vec,
                                                   current_joints_speed);
    gravity = panda.getGravityVector(current_joints_config_vec);

    {
      std::lock_guard<std::mutex> lock(desired_cmd_mutex);
      err_pos = desired_joints_position - current_joints_config_vec;
      err_vel = desired_joints_velocity - current_joints_speed;
      mass_matrix = panda.getMassMatrix(current_joints_config_vec);
      y_accel = mass_matrix * Md * desired_joints_accelerations;

      if (compliance_mode.load()) {
        y = -KD * current_joints_speed;
      } else {
        y = Md * desired_joints_accelerations + KD * (err_vel) + KP * (err_pos);
      }
    }

    // Consider control input without the gravitational contribute
    control_input = mass_matrix * y + non_linear_effects - gravity;

    // Clamping control input
    //
    // WARN: This section has to clamp the control signal WITHOUT the gravity
    // (and motor friction) contributions as per libfranka library logic on
    // commanded torque.
    if (clamp) {
      // clamp_control_speed(control_input,
      //                     (control_loop_rate->period().count() * 1e-9));
      clamp_control(control_input);
    }

    // Apply control
    //
    RCLCPP_DEBUG_STREAM(this->get_logger(), control_input);
    // Re-sum the gravitational effort contribute
    publish_efforts(control_input + gravity);
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Seconds from last control published: "
    //                        << (this->now() - last_control_cycle).seconds());
    // last_control_cycle = this->now();

    cmd.header.stamp = this->now();

    for (int i = 0; i < control_input.size(); i++) {
      cmd.effort_values[i] = control_input[i];
    }
    robot_joint_efforts_pub_debug->publish(cmd);

    for (int i = 0; i < gravity.size(); i++) {
      cmd.effort_values[i] = gravity[i];
    }
    gravity_contribute_debug->publish(cmd);

    for (int i = 0; i < y.size(); i++) {
      cmd.effort_values[i] = y[i];
    }
    y_contribute_debug->publish(cmd);

    for (int i = 0; i < y.size(); i++) {
      cmd.effort_values[i] = y_accel[i];
    }
    y_desired_accel_debug->publish(cmd);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = err_pos[i];
    }
    position_error_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = err_vel[i];
    }
    velocity_error_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = current_joints_config_vec[i];
    }
    current_position_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = current_joints_speed[i];
    }
    current_velocity_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_position[i];
    }
    desired_position_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_velocity[i];
    }
    desired_velocity_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_accelerations[i];
    }
    desired_acceleration_debug->publish(pos);

    mass.data = panda.getMassMatrix(current_joints_config_vec).norm();
    mass_matrix_val_debug->publish(mass);

    // Save last control input without the gravitational contribute
    //
    last_control_input = control_input;

    // Print current pose to topic
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.pose = panda.getPose(frame_id_name);
    current_pose.header.stamp = this->now();
    // robot_pose_pub->tryPublish(current_pose);

    // Sleep
    //
    control_loop_rate->sleep();
  }

  if (!rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Requested shutdown");
    return;
  }
}

void InverseDynamicsController::control_libfranka_sim() {
  // Inverse dynamics controller
  //
  //

  Eigen::Vector<double, 7> non_linear_effects;
  Eigen::Vector<double, 7> gravity;
  Eigen::Vector<double, 7> control_input;
  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 7> err_pos;
  Eigen::Vector<double, 7> err_vel;
  Eigen::Matrix<double, 7, 7> mass_matrix;
  Eigen::Vector<double, 7> y_accel;
  Eigen::Vector<double, 7> current_joints_config_vec;
  Eigen::Vector<double, 7> current_joints_speed;
  franka::RobotState state;

  JointsEffort cmd;
  JointsPos pos;
  std_msgs::msg::Float64 mass;

  Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp * 3, Kp * 3, Kp * 4};
  // Eigen::Vector<double, 7> KP_{Kp, Kp, Kp, Kp, Kp, Kp, Kp};
  Eigen::Matrix<double, 7, 7> KP = Eigen::Matrix<double, 7, 7>::Identity();
  KP.diagonal() = KP_;

  Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd * 3, Kd * 3, Kd * 4};
  // Eigen::Vector<double, 7> KD_{Kd, Kd, Kd, Kd, Kd, Kd, Kd};
  Eigen::Matrix<double, 7, 7> KD = Eigen::Matrix<double, 7, 7>::Identity();
  KD.diagonal() = KD_;

  rclcpp::Time last_control_cycle = this->now();

  // Franka panda joint config with real robot
  // franka::RobotState initial_state = panda_franka->readOnce();
  // for (size_t i = 0; i < 7; i++) {
  //   desired_joints_position[i] = initial_state.q[i];
  // }

  RCLCPP_INFO(this->get_logger(), "Waiting for simulation to start");
  while (rclcpp::Time{current_joint_config->header.stamp} - this->now() ==
         rclcpp::Duration{0, 0}) {
  }
  for (size_t i = 0; i < 7; i++) {
    desired_joints_position[i] = current_joint_config->position[i];
  }
  clamp_joint_config();

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Current desired config at initial loop is the current configuration: ["
          << desired_joints_position[0] << ", " << desired_joints_position[1]
          << ", " << desired_joints_position[2] << ", "
          << desired_joints_position[3] << ", " << desired_joints_position[4]
          << ", " << desired_joints_position[5] << ", "
          << desired_joints_position[6] << "]");

  while (start_flag.load() && rclcpp::ok()) {

    pos.header.stamp = this->now();

    // Update robot model
    //

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);

      // Update franka robot state
      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_config_vec[i] = current_joint_config->position[i];
        state.q[i] = current_joint_config->position[i];
      }

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_speed[i] = current_joint_config->velocity[i];
        state.dq[i] = current_joint_config->velocity[i];
      }
    }

    // Calculate quantities for control
    //

    // Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(
    //     this->panda_franka_model.value().mass(state).data());
    std::array<double, 49> mass_matrix_raw =
        this->panda_franka_model.value().mass(state);
    Eigen::Matrix<double, 7, 7> mass_matrix;
    for (size_t i = 0; i < 7; i++) {
      for (size_t j = 0; j < 7; j++) {
        mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
      }
    }

    // Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
    //     this->panda_franka_model.value().coriolis(state).data());
    std::array<double, 7> coriolis_raw =
        this->panda_franka_model.value().coriolis(state);
    Eigen::Vector<double, 7> coriolis;
    for (size_t i = 0; i < 7; i++) {
      coriolis(i) = coriolis_raw[i];
    }

    // Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(
    //     this->panda_franka_model.value().gravity(state).data());
    std::array<double, 7> gravity_raw =
        this->panda_franka_model.value().gravity(state);
    for (size_t i = 0; i < 7; i++) {
      gravity(i) = gravity_raw[i];
    }
    gravity = panda.getGravityVector(current_joints_config_vec);

    {
      std::lock_guard<std::mutex> lock(desired_cmd_mutex);
      err_pos = desired_joints_position - current_joints_config_vec;
      err_vel = desired_joints_velocity - current_joints_speed;
      y_accel = mass_matrix * Md * desired_joints_accelerations;

      if (compliance_mode.load()) {
        y = -KD * current_joints_speed;
      } else {
        y = Md * desired_joints_accelerations + KD * (err_vel) + KP * (err_pos);
      }
    }

    // Consider control input without the gravitational contribute
    control_input = mass_matrix * y + coriolis;

    // Clamping control input
    //
    // WARN: This section has to clamp the control signal WITHOUT the gravity
    // (and motor friction) contributions as per libfranka library logic on
    // commanded torque.
    if (clamp) {
      // clamp_control_speed(control_input,
      //                     (control_loop_rate->period().count() * 1e-9));
      clamp_control(control_input);
    }

    // Apply control
    //
    RCLCPP_DEBUG_STREAM(this->get_logger(), control_input);
    // Re-sum the gravitational effort contribute
    publish_efforts(control_input + gravity);
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Seconds from last control published: "
    //                        << (this->now() - last_control_cycle).seconds());
    // last_control_cycle = this->now();

    cmd.header.stamp = this->now();

    for (int i = 0; i < control_input.size(); i++) {
      cmd.effort_values[i] = control_input[i];
    }
    robot_joint_efforts_pub_debug->publish(cmd);

    for (int i = 0; i < gravity.size(); i++) {
      cmd.effort_values[i] = gravity[i];
    }
    gravity_contribute_debug->publish(cmd);

    for (int i = 0; i < y.size(); i++) {
      cmd.effort_values[i] = y[i];
    }
    y_contribute_debug->publish(cmd);

    for (int i = 0; i < y.size(); i++) {
      cmd.effort_values[i] = y_accel[i];
    }
    y_desired_accel_debug->publish(cmd);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = err_pos[i];
    }
    position_error_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = err_vel[i];
    }
    velocity_error_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = current_joints_config_vec[i];
    }
    current_position_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = current_joints_speed[i];
    }
    current_velocity_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_position[i];
    }
    desired_position_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_velocity[i];
    }
    desired_velocity_debug->publish(pos);

    for (int i = 0; i < err_pos.size(); i++) {
      pos.joint_values[i] = desired_joints_accelerations[i];
    }
    desired_acceleration_debug->publish(pos);

    mass.data = panda.getMassMatrix(current_joints_config_vec).norm();
    mass_matrix_val_debug->publish(mass);

    // Save last control input without the gravitational contribute
    //
    last_control_input = control_input;

    // Print current pose to topic
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.pose = panda.getPose(frame_id_name);
    current_pose.header.stamp = this->now();
    // robot_pose_pub->tryPublish(current_pose);

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
  auto node = std::make_shared<InverseDynamicsController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
