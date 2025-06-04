#include "algorithm/jacobian.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
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

class LowPassFilterVector {
private:
  double alpha;
  Eigen::VectorXd prev_y;
  bool initialized;

public:
  LowPassFilterVector(double alpha_, int size)
      : alpha(alpha_), prev_y(Eigen::VectorXd::Zero(size)), initialized(false) {
  }

  Eigen::VectorXd filter(const Eigen::VectorXd &x) {
    if (!initialized) {
      prev_y = x;
      initialized = true;
      return x;
    }
    prev_y = alpha * x + (1.0 - alpha) * prev_y;
    return prev_y;
  }
};
auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort;

class InverseDynamicsController : public rclcpp_lifecycle::LifecycleNode {

public:
  InverseDynamicsController(
      const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::inverse_dynamics_controller_node_name),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 250.0);
    this->declare_parameter<double>("Kd", 50.0);
    this->declare_parameter<double>("Md", 100.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<bool>("clamp", true);

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();

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
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
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

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {

    RCLCPP_INFO(get_logger(), "Activating...");
    // Set desired pose to current pose
    desired_joints_position.resize(panda.getModel().nq);
    desired_joints_velocity.resize(panda.getModel().nq);
    desired_joints_accelerations.resize(panda.getModel().nq);
    if (desired_joints_position.size() == 0 && current_joint_config) {
      for (size_t i = 0; i < current_joint_config->name.size(); i++) {
        desired_joints_position[i] = current_joint_config->position[i];
        desired_joints_velocity[i] = 0.0;
        desired_joints_accelerations[i] = 0.0;
      }
    }

    clamp_joint_config();

    last_control_input.resize(panda.getModel().nq);
    for (int i = 0; i < last_control_input.size(); i++) {
      last_control_input[i] = 0.0;
    }

    // Start control loop thread
    start_flag.store(true);
    control_thread =
        std::thread{std::bind(&InverseDynamicsController::control, this)};
    RCLCPP_INFO(this->get_logger(), "Started control thread");

    return CallbackReturn::SUCCESS;
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

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  // Robot related variables
  panda::RobotModel panda;
  JointState::SharedPtr current_joint_config{nullptr};
  JointTorqueMeasureStamped::SharedPtr current_measured_joint_torques{nullptr};

  Eigen::VectorXd desired_joints_position;
  Eigen::VectorXd desired_joints_velocity;
  Eigen::VectorXd desired_joints_accelerations;

  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd effort_speed_limits{};
  Eigen::VectorXd joint_min_limits{};
  Eigen::VectorXd joint_max_limits{};
  Eigen::VectorXd velocity_limits{};
  Eigen::VectorXd acceleration_limits{};

  const std::string frame_id_name{"fr3_hand_tcp"};

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

  void clamp_control_speed(Eigen::VectorXd &control_input) {

    double dt = (control_loop_rate->period().count() * 1e-9);

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
      clamp_control_speed(control_input);
      clamp_control(control_input);
    }

    // Apply control
    //
    RCLCPP_DEBUG_STREAM(this->get_logger(), control_input);
    publish_efforts(control_input);

    // Save last control input
    //
    last_control_input = control_input;

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
