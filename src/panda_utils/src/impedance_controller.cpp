#include "algorithm/jacobian.hpp"
#include "franka/control_types.h"
#include "franka/exception.h"
#include "franka/rate_limiting.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
#include <cmath>
#include <cstdlib>
#include <exception>
#include <franka/model.h>
#include <franka/robot.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <ratio>
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

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort_no_table;

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

class ImpedanceController : public rclcpp_lifecycle::LifecycleNode {

public:
  ImpedanceController(const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::inverse_dynamics_controller_node_name),
        panda_mine(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
                   OrientationConfiguration::UNIT_QUATERNION),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 50.0);
    this->declare_parameter<double>("Kd", 10.0);
    this->declare_parameter<double>("Md", 1.0);
    this->declare_parameter<double>("lambda", 1e-2);
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
    }

    // Taking joint limits

    RCLCPP_INFO(this->get_logger(), "Getting effort speed limits");
    effort_limits = panda.getModel().effortLimit;

    RCLCPP_INFO(this->get_logger(), "Getting effort speed limits");
    effort_speed_limits.resize(panda.getModel().nq);
    for (int i = 0; i < panda.getModel().nq; i++) {
      effort_speed_limits[i] = 1000.0;
    }

    auto set_joint_state = [this](const JointState::SharedPtr msg) {
      current_joint_config = msg;
    };

    robot_joint_states_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_joint_state);

    auto set_desired_pose = [this](const Pose::SharedPtr msg) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Received desired pose");
      desired_pose = msg;
    };
    auto set_desired_twist = [this](const Twist::SharedPtr msg) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Received desired twist");
      desired_twist = msg;
    };
    auto set_desired_accel = [this](const Accel::SharedPtr msg) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Received desired accel");
      desired_accel = msg;
    };

    desired_pose_sub = this->create_subscription<Pose>(
        panda_interface_names::panda_pose_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_desired_pose);

    desired_twist_sub = this->create_subscription<Twist>(
        panda_interface_names::panda_twist_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_desired_twist);

    desired_accel_sub = this->create_subscription<Accel>(
        panda_interface_names::panda_accel_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_desired_accel);

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

    min_singular_val_pub = this->create_publisher<std_msgs::msg::Float64>(
        panda_interface_names::min_singular_value_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

    // DEBUGGING
    //

    robot_joint_efforts_pub_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/effort_no_gravity",
        panda_interface_names::DEFAULT_TOPIC_QOS);

    gravity_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/gravity", panda_interface_names::DEFAULT_TOPIC_QOS);

    pose_error_debug = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "debug/error/pose", panda_interface_names::DEFAULT_TOPIC_QOS);

    velocity_error_debug = this->create_publisher<TwistStamped>(
        "debug/error/velocity", panda_interface_names::DEFAULT_TOPIC_QOS);

    desired_pose_debug = this->create_publisher<PoseStamped>(
        "debug/desired_pose", panda_interface_names::DEFAULT_TOPIC_QOS);

    desired_velocity_debug = this->create_publisher<TwistStamped>(
        "debug/desired_velocity", panda_interface_names::DEFAULT_TOPIC_QOS);

    desired_acceleration_debug = this->create_publisher<AccelStamped>(
        "debug/desired_acceleration", panda_interface_names::DEFAULT_TOPIC_QOS);

    current_pose_debug = this->create_publisher<PoseStamped>(
        "debug/current_pose", panda_interface_names::DEFAULT_TOPIC_QOS);

    current_velocity_debug = this->create_publisher<TwistStamped>(
        "debug/current_velocity", panda_interface_names::DEFAULT_TOPIC_QOS);

    y_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/y_contribute", panda_interface_names::DEFAULT_TOPIC_QOS);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {

    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();
    lambda = this->get_parameter("lambda").as_double();
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
            << " with Kp = " << Kp << ", Kd = " << Kd << " and Md = " << Md);

    if (use_robot) {
      panda_franka_model = panda_franka.value().loadModel();

      robot_control_callback = [this](const franka::RobotState &state,
                                      franka::Duration dt) -> franka::Torques {
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }
        //
        // // Publish robot pose stamped
        // geometry_msgs::msg::PoseStamped current_pose;
        // current_pose.pose = convertMatrixToPose(state.O_T_EE);
        // current_pose.header.stamp = this->now();
        // robot_pose_pub->publish(current_pose);
        //
        // Eigen::VectorXd control_input_vec;
        // Eigen::VectorXd y;
        //
        // // Get q and q_dot
        // //
        // Eigen::Vector<double, 7> current_joints_config_vec =
        //     Eigen::Vector<double, 7>::Map(state.q.data(), state.q.size());
        //
        // Eigen::Vector<double, 7> current_joints_speed =
        //     Eigen::Vector<double, 7>::Map(state.q_d.data(),
        //     state.q_d.size());
        //
        // // Calculate quantities for control
        // // Calculated through libfranka lib for better accuracy
        // // WARN: the libfranka lib considers the torque command sent to the
        // // robot as part of the real torque commanded.
        // // The torque commanded is the sum of 3 terms:
        // // - tau_d = user commanded torque
        // // - tua_g = torque required for gravity compensation
        // // - tau_f = torque required to compensate motor friction
        // //
        // // B(q)
        // Eigen::Matrix<double, 7, 7> mass_matrix =
        //     Eigen::Matrix<double, 7, 7>::Map(
        //         this->panda_franka_model.value().mass(state).data());
        //
        // // Calculate only the coriolis term for the n(q, q_dot) term
        // Eigen::Vector<double, 7> coriolis = Eigen::Vector<double, 7>::Map(
        //     this->panda_franka_model.value().coriolis(state).data());
        //
        // // If compliance mode: Kp = 0
        // if (compliance_mode.load()) {
        //   y = desired_joints_accelerations +
        //       Kd * (desired_joints_velocity - current_joints_speed);
        // } else {
        //   y = desired_joints_accelerations +
        //       Kd * (desired_joints_velocity - current_joints_speed) +
        //       Kp * (desired_joints_position - current_joints_config_vec);
        // }
        //
        // // Inverse dynamics torque commanded
        // control_input_vec = mass_matrix * y + coriolis;
        //
        // // Clamping control input
        // // Torque limits for fr3 indicated by libfranka lib
        // //
        // //
        // https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3
        // //
        // // Clamp signal except on first iteration e.g. Duration == 0
        // if (clamp && dt.toSec() != 0.0) {
        //   clamp_control_speed(control_input_vec, dt.toSec());
        //   clamp_control(control_input_vec);
        // }
        //
        // // Apply control
        // //
        // RCLCPP_DEBUG_STREAM(this->get_logger(), control_input_vec);
        // publish_efforts(control_input_vec);
        //
        // std::array<double, 7> tau;
        // for (size_t i = 0; i < 7; ++i) {
        //   tau[i] = control_input_vec[i];
        // }
        // return franka::Torques(tau);
      };
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

    // Wait for desired pose
    // WARN: should check if pose is reachable
    std::thread([this]() -> void {
      wait_for_pose();

      if (use_robot) {
        start_flag.store(true);

        // Try-catch version
        control_thread = std::thread{[this]() {
          try {
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
      }

      // Start control loop thread
      start_flag.store(true);
      control_thread =
          std::thread{std::bind(&ImpedanceController::control, this)};
      RCLCPP_INFO(this->get_logger(), "Started control thread");
    }).detach();

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
  rclcpp::Subscription<Pose>::SharedPtr desired_pose_sub{};
  rclcpp::Subscription<Twist>::SharedPtr desired_twist_sub{};
  rclcpp::Subscription<Accel>::SharedPtr desired_accel_sub{};

  // Commands publisher
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub{};

  // Robot pose publisher and debug
  Publisher<PoseStamped>::SharedPtr robot_pose_pub{};
  Publisher<std_msgs::msg::Float64>::SharedPtr min_singular_val_pub{};
  Publisher<PoseStamped>::SharedPtr pose_error_debug{};
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub_debug{};
  Publisher<JointsEffort>::SharedPtr gravity_contribute_debug{};
  Publisher<JointsEffort>::SharedPtr y_contribute_debug{};
  Publisher<TwistStamped>::SharedPtr velocity_error_debug{};
  Publisher<PoseStamped>::SharedPtr desired_pose_debug{};
  Publisher<TwistStamped>::SharedPtr desired_velocity_debug{};
  Publisher<AccelStamped>::SharedPtr desired_acceleration_debug{};
  Publisher<PoseStamped>::SharedPtr current_pose_debug{};
  Publisher<TwistStamped>::SharedPtr current_velocity_debug{};

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  // Robot related variables
  Robot<7> panda_mine;
  panda::RobotModel panda;
  std::optional<franka::Robot> panda_franka;
  std::optional<franka::Model> panda_franka_model;
  Eigen::Quaterniond old_quaternion;
  bool use_robot;
  JointState::SharedPtr current_joint_config{nullptr};
  JointTorqueMeasureStamped::SharedPtr current_measured_joint_torques{nullptr};
  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      robot_control_callback;
  double lambda;

  Pose::SharedPtr desired_pose;
  Twist::SharedPtr desired_twist = std::make_shared<Twist>();
  Accel::SharedPtr desired_accel = std::make_shared<Accel>();

  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd effort_speed_limits{};
  Eigen::VectorXd joint_min_limits{};
  Eigen::VectorXd joint_max_limits{};
  Eigen::VectorXd velocity_limits{};
  Eigen::VectorXd acceleration_limits{};

  const std::string frame_id_name{"fr3_link8"};

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

  void wait_for_pose() {
    using namespace std::chrono_literals;
    desired_pose = nullptr;
    while (!desired_pose) {
      RCLCPP_INFO(this->get_logger(), "Waiting for desired robot pose");
      std::this_thread::sleep_for(1s);
    }
    old_quaternion = Eigen::Quaterniond{};
    old_quaternion.w() = desired_pose->orientation.w;
    old_quaternion.x() = desired_pose->orientation.x;
    old_quaternion.y() = desired_pose->orientation.y;
    old_quaternion.z() = desired_pose->orientation.z;
    old_quaternion.normalize();
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
};

void ImpedanceController::control() {
  // Impedance controller

  Eigen::Vector<double, 7> non_linear_effects;
  Eigen::Vector<double, 7> control_input;
  Eigen::VectorXd gravity;
  Eigen::Vector<double, 7> y;
  PoseStamped pose_debug;
  TwistStamped twist_debug;
  AccelStamped accel_debug;
  std_msgs::msg::Float64 sigma;
  JointsEffort cmd;

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Desired Position: ["
          << desired_pose->position.x << ", " << desired_pose->position.y
          << ", " << desired_pose->position.z << "]; Orientation: [] "
          << desired_pose->orientation.w << ", " << desired_pose->orientation.x
          << ", " << desired_pose->orientation.y << ", "
          << desired_pose->orientation.z << "]");

  // Coefficient for dynamic lambda damping
  double epsilon = 1e-1;
  double k_max = 0.7;
  double alpha = 30.0;
  

  Eigen::Vector<double, 6> KP_{Kp, Kp, Kp, Kp, Kp, Kp};
  Eigen::Matrix<double, 6, 6> KP = Eigen::Matrix<double, 6, 6>::Identity();
  KP.diagonal() = KP_;

  Eigen::Vector<double, 6> KD_{Kd, Kd, Kd, Kd, Kd, Kd};
  Eigen::Matrix<double, 6, 6> KD = Eigen::Matrix<double, 6, 6>::Identity();
  KD.diagonal() = KD_;

  Eigen::Vector<double, 6> MD_{Md, Md, Md, Md, Md, Md};
  Eigen::Matrix<double, 6, 6> MD = Eigen::Matrix<double, 6, 6>::Identity();
  MD.diagonal() = MD_;
  auto MD_1 = MD.inverse();

  while (start_flag.load() && rclcpp::ok()) {

    // RCLCPP_INFO(this->get_logger(), "Entered cycle");
    pose_debug.header.stamp = this->now();
    twist_debug.header.stamp = this->now();
    accel_debug.header.stamp = this->now();

    Eigen::VectorXd current_joints_config_vec;
    current_joints_config_vec.resize(current_joint_config->position.size());
    for (size_t i = 0; i < current_joint_config->position.size(); i++) {
      current_joints_config_vec[i] = current_joint_config->position[i];
    }

    Eigen::VectorXd current_joints_speed;
    current_joints_speed.resize(current_joint_config->position.size());
    for (size_t i = 0; i < current_joint_config->position.size(); i++) {
      current_joints_speed[i] = current_joint_config->velocity[i];
    }
    // RCLCPP_INFO(this->get_logger(), "Computed terms");
    panda.computeAll(current_joints_config_vec, current_joints_speed);

    // double current_joints_config_doubles[7];
    // for (size_t i = 0; i < 7; i++) {
    //   current_joints_config_doubles[i] = current_joint_config->position[i];
    // }

    panda.computeAll(current_joints_config_vec, current_joints_speed);

    // Get current pose
    Pose current_pose_tmp = panda.getPose(frame_id_name);
    // Pose current_pose_tmp = panda_mine.pose(current_joints_config_doubles);
    Eigen::Quaterniond current_quat{};
    current_quat.w() = current_pose_tmp.orientation.w;
    current_quat.x() = current_pose_tmp.orientation.x;
    current_quat.y() = current_pose_tmp.orientation.y;
    current_quat.z() = current_pose_tmp.orientation.z;
    current_quat.normalize();
    current_quat = quaternionContinuity(current_quat, old_quaternion);
    old_quaternion = current_quat;

    // Get desired pose
    Eigen::Quaterniond desired_quat{};
    desired_quat.w() = desired_pose->orientation.w;
    desired_quat.x() = desired_pose->orientation.x;
    desired_quat.y() = desired_pose->orientation.y;
    desired_quat.z() = desired_pose->orientation.z;
    desired_quat.normalize();

    // Calculate pose error
    Eigen::Quaterniond error_quat{};
    error_quat = desired_quat * current_quat.inverse();
    Eigen::Vector<double, 6> error_pose_vec{};
    error_pose_vec(0) = desired_pose->position.x - current_pose_tmp.position.x;
    error_pose_vec(1) = desired_pose->position.y - current_pose_tmp.position.y;
    error_pose_vec(2) = desired_pose->position.z - current_pose_tmp.position.z;
    error_pose_vec(3) = error_quat.x();
    error_pose_vec(4) = error_quat.y();
    error_pose_vec(5) = error_quat.z();

    //
    // Get desired twist
    Eigen::Vector<double, 6> desired_twist_vec{};
    desired_twist_vec(0) = desired_twist->linear.x;
    desired_twist_vec(1) = desired_twist->linear.y;
    desired_twist_vec(2) = desired_twist->linear.z;
    desired_twist_vec(3) = desired_twist->angular.x;
    desired_twist_vec(4) = desired_twist->angular.y;
    desired_twist_vec(5) = desired_twist->angular.z;
    //
    // Get desired accel
    Eigen::Vector<double, 6> desired_accel_vec{};
    desired_accel_vec(0) = desired_accel->linear.x;
    desired_accel_vec(1) = desired_accel->linear.y;
    desired_accel_vec(2) = desired_accel->linear.z;
    desired_accel_vec(3) = desired_accel->angular.x;
    desired_accel_vec(4) = desired_accel->angular.y;
    desired_accel_vec(5) = desired_accel->angular.z;
    // RCLCPP_INFO(this->get_logger(), "Got desired quantities");

    // Update robot model
    //

    // Calculate quantities for control
    //
    Eigen::Matrix<double, 6, 7> jacobian =
        panda.getGeometricalJacobian(frame_id_name);
    // Eigen::Matrix<double, 6, 7> jacobian =
    //     panda_mine.geometrical_jacobian(current_joints_config_doubles);

    // Calculate jacobian SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);
    double sigma_min = svd.singularValues().tail(1)(0);

    // RCLCPP_INFO(this->get_logger(), "Calculated jacob");
    // Doesn't account for friction
    non_linear_effects = panda.getNonLinearEffects(current_joints_config_vec,
                                                   current_joints_speed);
    gravity = panda.getGravityVector(current_joints_config_vec);
    auto mass_matrix = panda.getMassMatrix(current_joints_config_vec);

    Eigen::Matrix<double, 7, 6> jacobian_pinv;
    // STATIC LAMBDA
    // jacobian_pinv = jacobian.transpose() *
    //                 (jacobian * jacobian.transpose() +
    //                  lambda * Eigen::Matrix<double, 6, 6>::Identity())
    //                     .inverse();

    // DYNAMIC LAMBDA BASED ON CONDITION NUMBER
    // double lambda = sigma_min <= epsilon
    //                     ? k_max * (pow(sigma_min, 2) / pow(epsilon, 2))
    //                     : 0.0;
    double lambda = std::exp(-alpha * sigma_min);
    jacobian_pinv = jacobian.transpose() *
                    (jacobian * jacobian.transpose() +
                     lambda * Eigen::Matrix<double, 6, 6>::Identity())
                        .inverse();

    // Calculate J_dot * q_dot
    auto current_twist = jacobian * current_joints_speed;
    auto error_twist = desired_twist_vec - current_twist;

    // RCLCPP_INFO(this->get_logger(), "Calculating control");
    if (compliance_mode.load()) {
      y = jacobian_pinv * (1 / Md) *
          (

              Md * desired_accel_vec +
              KD * (desired_twist_vec - jacobian * current_joints_speed)
              // - panda.computeHessianTimesQDot(current_joints_config_vec,
              //                                     current_joints_speed,
              //                                     frame_id_name)

          );
    } else {

      y = jacobian_pinv * MD_1 *
          (

              MD * desired_accel_vec + KD * error_twist + KP * error_pose_vec -
              MD * panda.computeHessianTimesQDot(current_joints_config_vec,
                                                 current_joints_speed,
                                                 frame_id_name)

          );
    }

    // RCLCPP_INFO(this->get_logger(), "Calculating control input");
    control_input = mass_matrix * y + non_linear_effects - gravity;

    // Clamping control input
    //
    if (clamp) {
      // clamp_control_speed(control_input,
      //                     (control_loop_rate->period().count() * 1e-9));
      clamp_control(control_input);
    }

    // Apply control
    //
    publish_efforts(control_input + gravity);

    // Save last control input
    //
    last_control_input = control_input;

    pose_debug.pose = panda.getPose(frame_id_name);
    robot_pose_pub->publish(pose_debug);

    // DEBUG
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

    // POSE

    current_pose_debug->publish(pose_debug);

    pose_debug.pose = *desired_pose;
    desired_pose_debug->publish(pose_debug);

    // VELOCITY

    twist_debug.twist.linear.x = current_twist[0];
    twist_debug.twist.linear.y = current_twist[1];
    twist_debug.twist.linear.z = current_twist[2];

    twist_debug.twist.angular.x = current_twist[3];
    twist_debug.twist.angular.y = current_twist[4];
    twist_debug.twist.angular.z = current_twist[5];

    current_velocity_debug->publish(twist_debug);

    twist_debug.twist.linear.x = error_twist[0];
    twist_debug.twist.linear.y = error_twist[1];
    twist_debug.twist.linear.z = error_twist[2];

    twist_debug.twist.angular.x = error_twist[3];
    twist_debug.twist.angular.y = error_twist[4];
    twist_debug.twist.angular.z = error_twist[5];

    velocity_error_debug->publish(twist_debug);

    twist_debug.twist = *desired_twist;
    desired_velocity_debug->publish(twist_debug);

    accel_debug.accel = *desired_accel;
    desired_acceleration_debug->publish(accel_debug);

    // Print current pose to topic
    // current_pose.pose = panda_mine.pose(current_joints_config_doubles);

    // Publish minimum singular value
    sigma.data = sigma_min;
    min_singular_val_pub->publish(sigma);

    // Publish error on pose
    geometry_msgs::msg::PoseStamped error;
    error.pose.position.x = error_pose_vec(0);
    error.pose.position.y = error_pose_vec(1);
    error.pose.position.z = error_pose_vec(2);
    error.pose.orientation.w = error_quat.w();
    error.pose.orientation.x = error_quat.x();
    error.pose.orientation.y = error_quat.y();
    error.pose.orientation.z = error_quat.z();
    error.header.stamp = this->now();
    pose_error_debug->publish(error);

    // Debug printing
    // Lambda dynamic
    RCLCPP_INFO_STREAM(this->get_logger(), "Lambda: " << lambda);

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
