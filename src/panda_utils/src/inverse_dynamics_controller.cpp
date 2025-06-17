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
#include "sensor_msgs/msg/joint_state.hpp"
#include "spatial/explog-quaternion.hpp"
#include "spatial/fwd.hpp"
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
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
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

geometry_msgs::msg::Pose
convertMatrixToPose(const std::array<double, 16> &tf_matrix) {
  // Step 1: Map the array to an Eigen 4x4 matrix (column-major)
  Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(tf_matrix.data());

  // Step 2: Extract translation (last column)
  Eigen::Vector3d translation = T.block<3, 1>(0, 3);

  // Step 3: Extract rotation matrix
  Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);

  // Step 4: Convert to quaternion
  Eigen::Quaterniond quat(rotation);
  quat.normalize(); // Safety

  // Step 5: Fill geometry_msgs::msg::Pose
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
    this->declare_parameter<double>("Kp", 150.0);
    this->declare_parameter<double>("Kd", 30.0);
    this->declare_parameter<double>("Md", 1.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<bool>("clamp", true);
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<bool>("use_robot", false);

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();
    use_robot = this->get_parameter("use_robot").as_bool();

    if (use_robot) {
      panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Connected to robot with ip "
                             << this->get_parameter("robot_ip").as_string());
    }

    // Taking joint limits

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

    RCLCPP_INFO(this->get_logger(), "Getting velocity limits");
    velocity_limits = panda.getModel().velocityLimit;

    RCLCPP_INFO(this->get_logger(), "Getting acceleration limits");
    acceleration_limits.resize(panda.getModel().nq);
    for (int i = 0; i < panda.getModel().nq; i++) {
      acceleration_limits[i] = 10.0;
    }

    auto set_joint_state = [this](const JointState::SharedPtr msg) {
      current_joint_config = msg;
    };

    robot_joint_states_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_joint_state);

    auto set_joint_measured_torques =
        [this](const JointTorqueMeasureStamped::SharedPtr msg) {
          current_measured_joint_torques = msg;
        };

    robot_measured_torque_sub =
        this->create_subscription<JointTorqueMeasureStamped>(
            panda_interface_names::torque_sensor_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS,
            set_joint_measured_torques);

    auto set_desired_joints_command = [this](
                                          const JointsCommand::SharedPtr msg) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Received joint command");

      desired_joints_position =
          Eigen::VectorXd::Map(msg->positions.data(), msg->positions.size());

      desired_joints_velocity =
          Eigen::VectorXd::Map(msg->velocities.data(), msg->velocities.size());

      desired_joints_accelerations = Eigen::VectorXd::Map(
          msg->accelerations.data(), msg->accelerations.size());

      clamp_joint_config();
    };

    desired_joint_command_sub = this->create_subscription<JointsCommand>(
        panda_interface_names::panda_joint_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_desired_joints_command);

    robot_joint_efforts_pub = this->create_publisher<JointsEffort>(
        panda_interface_names::panda_effort_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

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

    robot_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        panda_interface_names::panda_pose_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

    joint_states_pub = this->create_publisher<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);
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
            << " with Kp = " << Kp << " and Kd = " << Kd);

    Eigen::Vector3d translation = Eigen::Vector3d{0, 0, 0.75};
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    T_0_b = pinocchio::SE3{orientation.toRotationMatrix(), translation};

    if (use_robot) {
      panda_franka_model = panda_franka.value().loadModel();
      joint_state_to_pub.position.resize(7);
      joint_state_to_pub.velocity.resize(7);
      joint_state_to_pub.effort.resize(7);

      robot_control_callback = [this](const franka::RobotState &state,
                                      franka::Duration dt) -> franka::Torques {
        // If the thread flag is changed or a shutdown has been requested,
        // always returns last command
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        // Publish current pose and joint state
        publish_robot_state_libfranka(state);

        // Get q and q_dot
        //
        Eigen::Vector<double, 7> current_joints_config_vec =
            Eigen::Vector<double, 7>::Map(state.q.data(), state.q.size());

        Eigen::Vector<double, 7> current_joints_speed =
            Eigen::Vector<double, 7>::Map(state.dq.data(), state.dq.size());

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
        Eigen::Matrix<double, 7, 7> mass_matrix =
            Eigen::Matrix<double, 7, 7>::Map(
                this->panda_franka_model.value().mass(state).data());

        // Calculate only the coriolis term for the n(q, q_dot) term
        Eigen::Vector<double, 7> coriolis = Eigen::Vector<double, 7>::Map(
            this->panda_franka_model.value().coriolis(state).data());

        Eigen::VectorXd y;
        // If compliance mode: Kp = 0
        if (compliance_mode.load()) {
          y = desired_joints_accelerations +
              Kd * (desired_joints_velocity - current_joints_speed);
        } else {
          y = desired_joints_accelerations +
              Kd * (desired_joints_velocity - current_joints_speed) +
              Kp * (desired_joints_position - current_joints_config_vec);
        }

        // Inverse dynamics torque commanded
        Eigen::VectorXd control_input_vec = mass_matrix * y + coriolis;

        // Clamping control input
        // Torque limits for fr3 indicated by libfranka lib
        //
        // https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3
        //
        // The value of last control input on first iteration is 0. Assuming a
        // fake cycle before the first one with 1 kHz freq
        //
        // Clamp tau_dot
        if (dt.toSec() == 0.0) {
          clamp_control_speed(control_input_vec, 1.0 / 1e3);
        } else {
          clamp_control_speed(control_input_vec, dt.toSec());
        }

        // Clamp tau
        clamp_control(control_input_vec);

        // Apply control
        //
        // RCLCPP_DEBUG_STREAM(this->get_logger(), control_input_vec);
        // publish_efforts(control_input_vec);

        last_control_input = control_input_vec;
        std::array<double, 7> tau;
        for (size_t i = 0; i < 7; ++i) {
          tau[i] = control_input_vec[i];
        }
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
      start_flag.store(true);

      // Try-catch version
      control_thread = std::thread{[this]() {
        try {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Starting control thread with real time robot");
          panda_franka->control(robot_control_callback);
        } catch (const franka::Exception &ex) {
          start_flag.store(false);
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
      }};

      // Not try-catch version
      // panda_franka->control(robot_control_callback);
      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with real time robot");
      return CallbackReturn::SUCCESS;
    } else {
      // Start control loop thread
      start_flag.store(true);
      control_thread =
          std::thread{std::bind(&InverseDynamicsController::control, this)};
      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with simulated robot");

      return CallbackReturn::SUCCESS;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    // Stop thread and join it
    start_flag.store(false);
    control_thread.join();
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
    control_thread.join();
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

  // Robot pose publisher
  Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub{};
  Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub{};

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  // Robot related variables
  panda::RobotModel panda;
  std::optional<franka::Robot> panda_franka;
  std::optional<franka::Model> panda_franka_model;
  bool use_robot;
  JointState::SharedPtr current_joint_config{nullptr};
  JointState joint_state_to_pub{};
  JointTorqueMeasureStamped::SharedPtr current_measured_joint_torques{nullptr};
  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      robot_control_callback;

  Eigen::VectorXd desired_joints_position;
  Eigen::VectorXd desired_joints_velocity;
  Eigen::VectorXd desired_joints_accelerations;

  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd effort_speed_limits{};
  Eigen::VectorXd joint_min_limits{};
  Eigen::VectorXd joint_max_limits{};
  Eigen::VectorXd velocity_limits{};
  Eigen::VectorXd acceleration_limits{};

  const std::string frame_id_name{"fr3_link8"};

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

  void clamp_control(Eigen::VectorXd &control_input) {

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
      if (abs(control_input[i] - last_control_input[i]) / dt) {
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

  void clamp_joint_config() {

    clamp_vec(desired_joints_position, joint_min_limits, joint_max_limits);

    clamp_vec(desired_joints_velocity, -velocity_limits, velocity_limits);

    clamp_vec(desired_joints_accelerations, -acceleration_limits,
              acceleration_limits);
  }

  void publish_robot_state_libfranka(const franka::RobotState &state) {

    // Publish robot pose stamped
    geometry_msgs::msg::PoseStamped current_pose;
    // WARN: Verify that this is the right frame considered in simulation
    current_pose.pose = convertMatrixToPose(
        panda_franka_model.value().pose(franka::Frame::kJoint7, state));
    // current_pose.pose = convertMatrixToPose(state.O_T_EE);
    current_pose.header.stamp = this->now();
    robot_pose_pub->publish(current_pose);

    // Publish joint state
    joint_state_to_pub.header.stamp = this->now();
    for (size_t i = 0; i < 7; i++) {
      joint_state_to_pub.position[i] = state.q[i];
      joint_state_to_pub.position[i] = state.dq[i];
      joint_state_to_pub.position[i] = state.tau_J[i];
    }

    joint_states_pub->publish(joint_state_to_pub);
  }
};

void InverseDynamicsController::control() {
  // Inverse dynamics controller
  //
  //

  Eigen::VectorXd non_linear_effects;
  Eigen::VectorXd control_input;
  Eigen::VectorXd y;

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Current desired config at initial loop: ["
          << desired_joints_position[0] << ", " << desired_joints_position[1]
          << ", " << desired_joints_position[2] << ", "
          << desired_joints_position[3] << ", " << desired_joints_position[4]
          << ", " << desired_joints_position[5] << ", "
          << desired_joints_position[6] << "]");

  while (start_flag.load() && rclcpp::ok()) {

    // Update robot model
    //
    Eigen::VectorXd current_joints_config_vec =
        Eigen::VectorXd::Map(current_joint_config->position.data(),
                             current_joint_config->position.size());

    Eigen::VectorXd current_joints_speed =
        Eigen::VectorXd::Map(current_joint_config->velocity.data(),
                             current_joint_config->velocity.size());

    panda.computeAll(current_joints_config_vec, current_joints_speed);

    // Calculate quantities for control
    //

    // Doesn't account for friction
    non_linear_effects = panda.getNonLinearEffects(current_joints_config_vec,
                                                   current_joints_speed);

    if (compliance_mode.load()) {
      y = Md * desired_joints_accelerations +
          Kd * (desired_joints_velocity - current_joints_speed);
    } else {
      y = Md * desired_joints_accelerations +
          Kd * (desired_joints_velocity - current_joints_speed) +
          Kp * (desired_joints_position - current_joints_config_vec);
    }

    control_input =
        panda.getMassMatrix(current_joints_config_vec) * y + non_linear_effects;

    // Clamping control input
    //
    if (clamp) {
      clamp_control_speed(control_input,
                          (control_loop_rate->period().count() * 1e-9));
      clamp_control(control_input);
    }

    // Apply control
    //
    RCLCPP_DEBUG_STREAM(this->get_logger(), control_input);
    publish_efforts(control_input);

    // Save last control input
    //
    last_control_input = control_input;

    // Print current pose to topic
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.pose = panda.getPose(frame_id_name);
    current_pose.header.stamp = this->now();
    robot_pose_pub->publish(current_pose);

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
