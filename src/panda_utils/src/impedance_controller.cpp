#include "algorithm/jacobian.hpp"
#include "franka/control_types.h"
#include "franka/exception.h"
#include "franka/lowpass_filter.h"
#include "franka/robot_state.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "multibody/fwd.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
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
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
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
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
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

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort_no_table;

// struct debug_data {
//   std::mutex mut;
//   bool has_data;
//   std::array<double, 7> tau_d_last;
//   franka::RobotState robot_state;
//   std::array<double, 7> gravity;
// };

enum class Mode {
  // Simulation only mode
  sim,
  // Simulation mode with franka library for robot model
  sim_franka,
  // Real mode with franka panda library and communication
  franka
};

double low_pass_filter_alpha(double frequency, double sampletime) {
  double time_constant = 1.0 / (2.0 * M_PI * frequency);
  return sampletime / (time_constant + sampletime);
}

struct robot_state {
  std::optional<franka::RobotState> state;
  rclcpp::Time state_time;
  std::mutex mut;
};

/// The quaternion represents a rotation
Eigen::Matrix<double, 3, 3> s_operator(const Eigen::Quaterniond &quat) {
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Zero();
  mat(0, 1) = -quat.z();
  mat(0, 2) = quat.y();
  mat(1, 0) = quat.z();
  mat(1, 2) = -quat.x();
  mat(2, 0) = quat.y();
  mat(2, 1) = quat.x();
  return mat;
}

Eigen::Matrix<double, 7, 6>
compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian,
                        const double &lambda) {
  return jacobian.transpose() *
         (jacobian * jacobian.transpose() +
          lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity())
             .inverse();
}

Eigen::Matrix<double, 6, 7>
get_j_dot(std::function<Eigen::Matrix<double, 6, 7>(Eigen::Vector<double, 7>)>
              get_jacobian_func,
          const Eigen::Vector<double, 7> &current_joint_pos,
          const Eigen::Vector<double, 7> &current_joint_speed) {
  // Compute Jdot as finite difference
  // J_forw - J_back / (2 delta_time) = J_dot
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

geometry_msgs::msg::Pose
convertMatrixToPose(const std::array<double, 16> &tf_matrix) {
  Eigen::Map<const Eigen::Matrix4d> eigen_matrix(tf_matrix.data());
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = eigen_matrix(0, 3);
  pose_msg.position.y = eigen_matrix(1, 3);
  pose_msg.position.z = eigen_matrix(2, 3);
  Eigen::Matrix3d rotation_matrix = eigen_matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(rotation_matrix);
  quaternion.normalize();
  pose_msg.orientation.x = quaternion.x();
  pose_msg.orientation.y = quaternion.y();
  pose_msg.orientation.z = quaternion.z();
  pose_msg.orientation.w = quaternion.w();
  return pose_msg;
}

Pose get_pose(const std::array<double, 16> &transform_mat) {
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(transform_mat.data()));
  // Eigen::Map<const Eigen::Matrix4d, Eigen::ColMajor> eigen_matrix(
  //     transform_mat.data());

  geometry_msgs::msg::Pose pose_msg;

  // pose_msg.position.x = eigen_matrix(0, 3); // Tx
  // pose_msg.position.y = eigen_matrix(1, 3); // Ty
  // pose_msg.position.z = eigen_matrix(2, 3); // Tz
  //
  pose_msg.position.x = initial_transform.translation().x(); // Tx
  pose_msg.position.y = initial_transform.translation().y(); // Ty
  pose_msg.position.z = initial_transform.translation().z(); // Tz

  // Eigen::Matrix3d rotation_matrix = eigen_matrix.block<3, 3>(0, 0);

  Eigen::Quaterniond quaternion(initial_transform.rotation());
  quaternion.normalize();

  pose_msg.orientation.x = quaternion.x();
  pose_msg.orientation.y = quaternion.y();
  pose_msg.orientation.z = quaternion.z();
  pose_msg.orientation.w = quaternion.w();

  return pose_msg;
}

Eigen::Matrix<double, 6, 7>
get_jacobian(const std::array<double, 42> jacob_raw) {
  Eigen::Map<const Eigen::Matrix<double, 6, 7>, Eigen::ColMajor> jacobian(
      jacob_raw.data());
  // Eigen::Matrix<double, 6, 7> jacobian;
  // for (size_t i = 0; i < 7; i++) {
  //   for (size_t j = 0; j < 6; j++) {
  //     jacobian(j, i) = jacob_raw[i * 7 + j];
  //   }
  // }
  return jacobian;
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
  Pose current_pose = get_pose(model.pose(franka::Frame::kFlange, state));
  RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position: ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");

  current_pose = get_pose(state.O_T_EE);
  RCLCPP_INFO_STREAM_ONCE(
      logger, "Current position (with O_T_EE): ["
                  << current_pose.position.x << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << "]; Orientation: [] "
                  << current_pose.orientation.w << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << "]");
  auto jacobian =
      get_jacobian(model.zeroJacobian(franka::Frame::kFlange, state));
  RCLCPP_INFO_STREAM_ONCE(logger, "Current jacobian: [" << jacobian << "]");

  // B(q)
  std::array<double, 49> mass_matrix_raw = model.mass(state);
  Eigen::Matrix<double, 7, 7> mass_matrix;
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < 7; j++) {
      mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
    }
  }

  // Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
  //     this->panda_franka_model.value().coriolis(state).data());
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

    return get_jacobian(model.zeroJacobian(franka::Frame::kFlange, state));
  };

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                      Eigen::ComputeThinV);
  double sigma_min = svd.singularValues().tail(1)(0);

  // DYNAMIC LAMBDA BASED ON MINIMUM SINGULAR VALUE
  double k_max = 1.0;
  double eps = 0.1;
  double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
  lambda = (sigma_min >= eps ? 0.0 : lambda);
  Eigen::Matrix<double, 7, 6> jacobian_pinv =
      compute_jacob_pseudoinv(jacobian, lambda);

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
        panda_mine(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
                   OrientationConfiguration::UNIT_QUATERNION),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 50.0);
    this->declare_parameter<double>("Kd", 10.0);
    this->declare_parameter<double>("Md", 1.0);
    this->declare_parameter<double>("alpha", 30.0);
    this->declare_parameter<double>("lambda", 1e-2);
    this->declare_parameter<double>("k_max", 2.0);
    this->declare_parameter<double>("eps", 0.1);
    this->declare_parameter<double>("task_gain", 1.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<double>("safe_joint_speed", 0.3);
    this->declare_parameter<double>("safe_effort_perc", 0.6);
    this->declare_parameter<double>("safe_error_pose_norm", 0.035);
    this->declare_parameter<bool>("clamp", true);
    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<bool>("use_robot", false);
    this->declare_parameter<bool>("use_franka_sim", false);
    // World to base link transform with quaternion (w, x, y, z)
    this->declare_parameter<std::vector<double>>(
        "world_base_link",
        std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    Md = this->get_parameter("Md").as_double();

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

    // Define mode of operation
    if (use_robot) {
      mode = Mode::franka;
      RCLCPP_INFO_STREAM(this->get_logger(), "Using mode franka");
    } else if (use_franka_sim) {
      mode = Mode::sim_franka;
      RCLCPP_INFO_STREAM(this->get_logger(), "Using mode sim_franka");
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
        panda_interface_names::CONTROLLER_SUBSCRIBER_QOS(), set_joint_state);

    auto set_cartesian_cmd =
        [this](
            const panda_interfaces::msg::CartesianCommand::ConstSharedPtr msg) {
          RCLCPP_INFO_ONCE(this->get_logger(), "Entered callback");
          desired_cartesian_box.try_set(*msg);
          // std::cout << "im here";
          // std::lock_guard<std::mutex> lock(desired_pose_mutex);
          // desired_pose = std::make_shared<Pose>(msg);
          // desired_pose_raw = msg;
        };
    auto set_desired_pose = [](const Pose::SharedPtr msg) {
      // std::cout << "im here";
      // std::lock_guard<std::mutex> lock(desired_pose_mutex);
      // desired_pose = std::make_shared<Pose>(msg);
      // desired_pose_raw = msg;
    };
    auto set_desired_twist = [](const Twist::SharedPtr msg) {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Received desired twist");
      // std::lock_guard<std::mutex> lock(desired_twist_mutex);
      // desired_twist = msg;
      // desired_twist_vec[0] = msg.linear.x;
      // desired_twist_vec[1] = msg.linear.y;
      // desired_twist_vec[2] = msg.linear.z;
      // desired_twist_vec[3] = msg.angular.x;
      // desired_twist_vec[4] = msg.angular.y;
      // desired_twist_vec[5] = msg.angular.z;
    };
    auto set_desired_accel = [](const Accel::SharedPtr msg) {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Received desired accel");
      // std::lock_guard<std::mutex> lock(desired_accel_mutex);
      // desired_accel = msg;
      // desired_accel_vec[0] = msg.linear.x;
      // desired_accel_vec[1] = msg.linear.y;
      // desired_accel_vec[2] = msg.linear.z;
      // desired_accel_vec[3] = msg.angular.x;
      // desired_accel_vec[4] = msg.angular.y;
      // desired_accel_vec[5] = msg.angular.z;
    };

    auto set_external_tau_cb = [this](const JointTorqueMeasureStamped msg) {
      for (int i = 0; i < 7; i++) {
        extern_tau[i] = msg.measures.torque[i];
      }
    };

    cartesian_cmd_sub =
        this->create_subscription<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd", panda_interface_names::DEFAULT_TOPIC_QOS(),
            set_cartesian_cmd);

    // desired_pose_sub = this->create_subscription<Pose>(
    //     panda_interface_names::panda_pose_cmd_topic_name,
    //     panda_interface_names::DEFAULT_TOPIC_QOS(), set_desired_pose);
    //
    // desired_twist_sub = this->create_subscription<Twist>(
    //     panda_interface_names::panda_twist_cmd_topic_name,
    //     panda_interface_names::DEFAULT_TOPIC_QOS(), set_desired_twist);
    //
    // desired_accel_sub = this->create_subscription<Accel>(
    //     panda_interface_names::panda_accel_cmd_topic_name,
    //     panda_interface_names::DEFAULT_TOPIC_QOS(), set_desired_accel);

    external_tau_sub = this->create_subscription<JointTorqueMeasureStamped>(
        panda_interface_names::torque_sensor_topic_name,
        panda_interface_names::CONTROLLER_SUBSCRIBER_QOS(),
        set_external_tau_cb);

    robot_joint_efforts_pub = this->create_publisher<JointsEffort>(
        panda_interface_names::panda_effort_cmd_topic_name,
        // Leave this as reliable otherwise the effort could not be delivered
        // to gz bridge interface
        panda_interface_names::DEFAULT_TOPIC_QOS());

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

    auto wrist_contact_cb =
        [this](
            const panda_interfaces::srv::WristContact_Request::SharedPtr
                request,
            panda_interfaces::srv::WristContact_Response::SharedPtr response) {
          std_msgs::msg::String wrist;

          if (!request->contact) {
            wrist_contact_frame = std::nullopt;
            wrist.data = "";
            wrist_contact_index_pub->publish(wrist);
            RCLCPP_INFO_STREAM(this->get_logger(), "Currently unset contact");
          } else {
            wrist = request->wrist;
            wrist_contact_frame = std::string(request->wrist.data);
            wrist_contact_index_pub->publish(wrist);
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Currently set wrist in touch to "
                                   << wrist.data);
          }
          response->set__success(true);
        };

    wrist_contact_server =
        this->create_service<panda_interfaces::srv::WristContact>(
            panda_interface_names::set_wrist_contact_service_name,
            wrist_contact_cb);

    wrist_contact_index_pub = this->create_publisher<std_msgs::msg::String>(
        panda_interface_names::wrist_contact_index_topic_name,
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

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

    min_singular_val_pub =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            panda_interface_names::min_singular_value_topic_name,
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    // DEBUGGING
    //

    robot_joint_efforts_pub_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/effort_no_gravity",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    gravity_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/gravity", panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    pose_error_debug = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "debug/error/pose", panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    velocity_error_debug = this->create_publisher<TwistStamped>(
        "debug/error/velocity",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    desired_pose_debug = this->create_publisher<PoseStamped>(
        "debug/desired_pose",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    desired_velocity_debug = this->create_publisher<TwistStamped>(
        "debug/desired_velocity",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    desired_acceleration_debug = this->create_publisher<AccelStamped>(
        "debug/desired_acceleration",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    current_pose_debug = this->create_publisher<PoseStamped>(
        "debug/current_pose",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    current_velocity_debug = this->create_publisher<TwistStamped>(
        "debug/current_velocity",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    y_contribute_debug = this->create_publisher<JointsEffort>(
        "debug/cmd/y_contribute",
        panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    y_cartesian_contribute_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/cmd/y_cartesian_contribute",
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    lamda_dls_debug =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "debug/lambda", panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    manipulability_index_debug =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "debug/manipulability_index",
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    manipulability_index_grad_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/manipulability_index_grad",
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    joint_limits_index_debug =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "debug/joint_limits_index",
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    joint_limits_index_grad_debug =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/joint_limits_index_grad",
            panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer, this);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {

    debug_pub.create_pubs(shared_from_this(),
                          panda_interface_names::CONTROLLER_PUBLISHER_QOS());

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
      double load = 0.553455;
      std::array F_x_Cload{-0.010328, 0.000068, 0.148159};
      std::array load_inertia{0.02001,        0.000006527121, -0.0004590,
                              0.000006527121, 0.01936,        0.000003371038,
                              -0.0004590,     0.000003371038, 0.002245};
      // panda_franka->setLoad(load, F_x_Cload, load_inertia);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Set load on real robot");
      panda_franka_model = panda_franka.value().loadModel();
      // Debug prints before activation
      print_initial_franka_state(panda_franka->readOnce(),
                                 panda_franka_model.value(),
                                 this->get_logger());

      // Getting initial pose as the current one
      desired_pose_raw = get_pose(panda_franka_model.value().pose(
          franka::Frame::kFlange, panda_franka->readOnce()));
      desired_pose = std::make_shared<Pose>(desired_pose_raw);

      desired_cartesian_cmd.pose = desired_pose_raw;
      desired_twist = desired_cartesian_cmd.twist;
      desired_accel = desired_cartesian_cmd.accel;

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

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Desired pose is: ["
                             << desired_pose->position.x << ", "
                             << desired_pose->position.y << ", "
                             << desired_pose->position.z << "], ["
                             << desired_pose->orientation.w << ", "
                             << desired_pose->orientation.x << ", "
                             << desired_pose->orientation.y << ", "
                             << desired_pose->orientation.z << "]");

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Desired accel is: ["
              << desired_accel.linear.x << ", " << desired_accel.linear.y
              << ", " << desired_accel.linear.z << "], ["
              << desired_accel.angular.x << ", " << desired_accel.angular.y
              << ", " << desired_accel.angular.z << "]");

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Desired twist is: ["
              << desired_twist.linear.x << ", " << desired_twist.linear.y
              << ", " << desired_twist.linear.z << "], ["
              << desired_twist.angular.x << ", " << desired_twist.angular.y
              << ", " << desired_twist.angular.z << "]");

      Eigen::Vector<double, 6> KP_{Kp, Kp, Kp, Kp, Kp, Kp};
      // Eigen::Vector<double, 6> KP_{Kp, Kp, Kp, Kp, Kp * 3, Kp * 3, Kp * 4};
      Eigen::Matrix<double, 6, 6> KP = Eigen::Matrix<double, 6, 6>::Identity();
      KP.diagonal() = KP_;

      Eigen::Vector<double, 6> KD_{Kd, Kd, Kd, Kd, Kd, Kd};
      // Eigen::Vector<double, 6> KD_{Kd, Kd, Kd, Kd, Kd * 3, Kd * 3, Kd * 4};
      Eigen::Matrix<double, 6, 6> KD = Eigen::Matrix<double, 6, 6>::Identity();
      KD.diagonal() = KD_;

      Eigen::Vector<double, 6> MD_{Md, Md, Md, Md, Md, Md};
      Eigen::Matrix<double, 6, 6> MD = Eigen::Matrix<double, 6, 6>::Identity();
      MD.diagonal() = MD_;
      Eigen::Matrix<double, 6, 6> MD_1 = MD.inverse();
      RCLCPP_INFO_STREAM(this->get_logger(), MD_1);

      // Coefficient for dynamic lambda damping
      double alpha = this->get_parameter("alpha").as_double();
      double k_max = this->get_parameter("k_max").as_double();
      double eps = this->get_parameter("eps").as_double();

      Eigen::Affine3d ident = Eigen::Affine3d::Identity();
      Eigen::Matrix4d matrix_ident = ident.matrix();
      std::array<double, 16> ident_transform;
      Eigen::Map<Eigen::Matrix4d>(ident_transform.data()) = matrix_ident;

      // Get jacobian function
      auto get_jacob =
          [this](const Eigen::Vector<double, 7> &current_joint_pos) {
            auto state = franka::RobotState{};
            for (size_t i = 0; i < state.q.size(); i++) {
              state.q[i] = current_joint_pos[i];
            }

            return get_jacobian(this->panda_franka_model->zeroJacobian(
                franka::Frame::kFlange, state));
          };
      Eigen::Vector<double, 6> current_twist;
      Eigen::Vector<double, 6> error_twist;
      Eigen::Vector<double, 7> y;
      Eigen::Vector<double, 7> last_y = Eigen::Vector<double, 7>::Zero();
      Eigen::Vector<double, 7> tau_ext;
      Eigen::Vector<double, 6> h_e;
      Pose current_pose;
      // Eigen::Matrix<double, 6, 7> jacobian =
      //     Eigen::Matrix<double, 6, 7>::Zero();
      Eigen::Vector<double, 7> current_joints_config_vec;
      Eigen::Quaterniond error_quat{};
      Eigen::Vector<double, 6> error_pose_vec{};
      std::array<double, 49> mass_matrix_raw;
      std::array<double, 7> coriolis_raw;
      Pose desired_pose;
      Eigen::Vector<double, 6> local_current_twist_vec;
      Eigen::Vector<double, 6> local_current_accel_vec;

      joint_state_to_pub.position.resize(7);
      joint_state_to_pub.velocity.resize(7);
      joint_state_to_pub.effort.resize(7);
      joint_state_to_pub.name = std::vector<std::string>{
          "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

      // Low pass filter alpha
      double low_pass_alpha = low_pass_filter_alpha(100.0, 0.001);
      RCLCPP_INFO(this->get_logger(), "low pass alpha: %f", low_pass_alpha);

      robot_control_callback =
          [this, KP, KD, MD, MD_1, k_max, eps, ident_transform, get_jacob,
           &current_twist, &error_twist, &y, &current_pose, &error_quat,
           &error_pose_vec, &mass_matrix_raw, low_pass_alpha,
           &last_y](const franka::RobotState &state,
                    franka::Duration dt) -> franka::Torques {
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }

        // Access the optional RealtimeThreadSafeBox
        // If it has something use the new value, else use the latest value
        // available
        desired_cartesian_cmd_opt = desired_cartesian_box.try_get();
        if (desired_cartesian_cmd_opt.has_value()) {
          desired_cartesian_cmd = desired_cartesian_cmd_opt.value();

          desired_pose_raw = desired_cartesian_cmd.pose;
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

        auto now = this->now();

        // Get q and q_dot
        //
        Eigen::Map<const Eigen::Vector<double, 7>> current_joints_config_vec(
            state.q.data());

        // Eigen::Map<const Eigen::Vector<double, 7>> current_joints_speed(
        //     state.dq.data());
        if (dt.toSec() != 0.0) {
          for (int i = 0; i < 7; i++) {
            current_joints_speed[i] = franka::lowpassFilter(
                dt.toSec(), state.dq[i], current_joints_speed[i], 15.0);
          }
        } else {
          for (int i = 0; i < 7; i++) {
            current_joints_speed[i] = state.dq[i];
          }
        }
        // last_joints_speed = current_joints_speed;

        // Get current pose and jacobian according to stiffness or flange
        // frame
        if (compliance_mode.load() && EE_to_K_transform.has_value()) {
          // I have to get Kstiffness frame pose if also EE_to_K has value
          try {
            panda_franka->setK(EE_to_K_transform.value());
          } catch (const franka::Exception &ex) {
          }

          panda_franka->setK(ident_transform);
          current_pose = get_pose(
              panda_franka_model->pose(franka::Frame::kStiffness, state));
          jacobian = get_jacobian(panda_franka_model->zeroJacobian(
              franka::Frame::kStiffness, state));
        } else {
          // panda_franka->setK(ident_transform);
          current_pose =
              get_pose(panda_franka_model->pose(franka::Frame::kFlange, state));
          jacobian = get_jacobian(
              panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
        }

        // Getting current desired quantities with mutex access

        Eigen::Quaterniond current_quat{};
        current_quat.w() = current_pose.orientation.w;
        current_quat.x() = current_pose.orientation.x;
        current_quat.y() = current_pose.orientation.y;
        current_quat.z() = current_pose.orientation.z;
        current_quat.normalize();
        current_quat = quaternionContinuity(current_quat, old_quaternion);
        old_quaternion = current_quat;

        // Calculate pose error
        {
          Eigen::Quaterniond desired_quat{};
          desired_quat.w() = desired_pose_raw.orientation.w;
          desired_quat.x() = desired_pose_raw.orientation.x;
          desired_quat.y() = desired_pose_raw.orientation.y;
          desired_quat.z() = desired_pose_raw.orientation.z;

          desired_quat.normalize();
          error_quat = desired_quat * current_quat.inverse();
          error_quat.normalize();

          error_pose_vec(0) =
              desired_pose_raw.position.x - current_pose.position.x;
          error_pose_vec(1) =
              desired_pose_raw.position.y - current_pose.position.y;
          error_pose_vec(2) =
              desired_pose_raw.position.z - current_pose.position.z;
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
        double lambda = k_max * (1.0 - pow(sigma_min, 2) / pow(eps, 2));
        lambda = (sigma_min >= eps ? 0.0 : lambda);
        Eigen::Matrix<double, 7, 6> jacobian_pinv =
            compute_jacob_pseudoinv(jacobian, lambda);
        // Eigen::Matrix<double, 7, 6> jacobian_pinv =
        //     jacobian.completeOrthogonalDecomposition().pseudoInverse();

        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                "jacob pinv: " << jacobian_pinv);

        Eigen::Map<const Eigen::Vector<double, 7>> tau_ext(
            state.tau_ext_hat_filtered.data());

        Eigen::Map<const Eigen::Vector<double, 6>> h_e(
            state.O_F_ext_hat_K.data());

        // Calculate quantities for control
        // Calculated through libfranka lib for better accuracy
        // WARN: the libfranka lib considers the torque command sent to the
        // robot as part of the real torque commanded.
        // The torque commanded is the sum of 3 terms:
        // - tau_d = user commanded torque
        // - tua_g = torque required for gravity compensation
        // - tau_f = torque required to compensate motor friction
        //

        {
          // std::lock_guard<std::mutex> lock(desired_twist_mutex);
          // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(),
          //                             1000.0, "Accessing desired twist");
          current_twist = jacobian * current_joints_speed;
          error_twist = desired_twist_vec - current_twist;
        }

        {
          // std::lock_guard<std::mutex> lock(desired_accel_mutex);
          // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(),
          //                             1000.0, "Accessing desired accel");
          if (compliance_mode.load()) {

            y = jacobian_pinv * MD_1 *
                (

                    -KD * current_twist -
                    MD *
                        get_j_dot(get_jacob, current_joints_config_vec,
                                  current_joints_speed) *
                        current_joints_speed
                    // - h_e

                );
          } else {

            y = jacobian_pinv * MD_1 *
                (

                    MD * desired_accel_vec + KD * error_twist +
                    KP * error_pose_vec
                    // -
                    //   MD *
                    //       get_j_dot(get_jacob, current_joints_config_vec,
                    //                 current_joints_speed) *
                    //       current_joints_speed
                    // - h_e

                );
          }
        }

        for (int i = 0; i < y.size(); i++) {
          if (y[i] == 0.0) {
            y[i] = last_y[i];
          }
        }
        last_y = y;

        // Eigen::Vector<double, 7> control_input_vec = mass_matrix * y +
        // coriolis;
        Eigen::Vector<double, 7> control_input_vec =
            mass_matrix * y + coriolis + tau_ext;

        // Clamp tau
        clamp_control(control_input_vec);

        // Safety checks
        if (dt.toSec() == 0.0) {
          for (int i = 0; i < control_input_vec.size(); i++) {
            control_input_vec[i] = 0.0;
          }
        } else {

          clamp_control_speed(control_input_vec, dt.toSec());

          RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Running safety checks");
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
              } else if (abs(state.dq[i]) >= 0.5) {
                RCLCPP_INFO_STREAM_ONCE(
                    this->get_logger(),
                    "Running safety check: joint limit speed");
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Joint velocity over the safety value 0.5");
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
                  << ", Desired pose: " << desired_pose_raw.position.x << ", "
                  << desired_pose_raw.position.y << ", "
                  << desired_pose_raw.position.z
                  << ", Desired twist: " << desired_twist_vec
                  << ", Desired accel: " << desired_accel_vec
                  << ", Jacobian pinv: " << jacobian_pinv << ", error twist: "
                  << error_twist << ", error pose: " << error_pose_vec
                  << ", tau_ext: " << tau_ext << ", coriolis: " << coriolis
                  << ", joint vel: " << current_joints_speed
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

        // if (panda_franka_state.mut.try_lock()) {
        //   panda_franka_state.state = state;
        //   panda_franka_state.state_time = now;
        // }

        if (debug_pub.data().mut.try_lock()) {

          try {
            // debug_pub.data().h_e = h_e;
            debug_pub.data().tau_ext = tau_ext;
            // debug_pub.data().lambda = lambda;
            // debug_pub.data().sigma_min = sigma_min;
            debug_pub.data().current_twist = current_twist;
            debug_pub.data().des_twist = desired_twist;
            // debug_pub.data().des_pose = desired_pose_raw;
            // debug_pub.data().des_accel = desired_accel;
            debug_pub.data().gravity = panda_franka_model->gravity(state);
            debug_pub.data().coriolis = coriolis;
            debug_pub.data().filtered_joints_vec = current_joints_speed;
            debug_pub.data().error_pose_vec.head(3) = error_pose_vec.head(3);
            debug_pub.data().error_pose_vec.tail(3) = error_pose_vec.tail(3);
            // w value of the pose message in a vector<7>
            debug_pub.data().error_pose_vec(3) = 1.0;
            debug_pub.data().tau_d_calculated = tau;
            debug_pub.data().tau_d_last = state.tau_J_d;
            debug_pub.data().y = mass_matrix * y;

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

        // PoseStamped pose_stamp;
        // pose_stamp.header.stamp = this->now();
        // pose_stamp.pose = current_pose;
        // robot_pose_pub->try_publish(pose_stamp);
        //
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
    case Mode::sim_franka: {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Configuring robot with simulation and franka library");
      panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
      double load = 0.553455;
      std::array F_x_Cload{-0.010328, 0.000068, 0.148159};
      std::array load_inertia{0.02001,        0.000006527121, -0.0004590,
                              0.000006527121, 0.01936,        0.000003371038,
                              -0.0004590,     0.000003371038, 0.002245};
      // panda_franka->setLoad(load, F_x_Cload, load_inertia);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Set load on real robot");
      panda_franka_model = panda_franka->loadModel();
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Loaded robot model library with libfranka");
      break;
    }
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

      geometry_msgs::msg::TransformStamped world_to_base_transform_static;
      world_to_base_transform_static.header.stamp = this->now();
      world_to_base_transform_static.header.frame_id = "world";
      world_to_base_transform_static.child_frame_id = "fr3_link0";

      // Assuming world_base_link parameter order is x, y, z, w, x, y, z for
      // quaternion
      world_to_base_transform_static.transform.translation.x =
          world_base_link[0];
      world_to_base_transform_static.transform.translation.y =
          world_base_link[1];
      world_to_base_transform_static.transform.translation.z =
          world_base_link[2];

      world_to_base_transform_static.transform.rotation.w = world_base_link[3];
      world_to_base_transform_static.transform.rotation.x = world_base_link[4];
      world_to_base_transform_static.transform.rotation.y = world_base_link[5];
      world_to_base_transform_static.transform.rotation.z = world_base_link[6];

      static_tf_broadcaster->sendTransform(world_to_base_transform_static);
      RCLCPP_INFO(this->get_logger(),
                  "Published static world -> fr3_link0 transform.");

      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(this->get_logger(),
                     "The robot thread has no real time kernel, shutting down");
        start_flag.store(false);
        rclcpp::shutdown();
      }
      // Getting initial pose as the current one
      desired_pose_raw = get_pose(panda_franka_model.value().pose(
          franka::Frame::kFlange, panda_franka->readOnce()));

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Desired pose is: ["
                             << desired_pose->position.x << ", "
                             << desired_pose->position.y << ", "
                             << desired_pose->position.z << "], ["
                             << desired_pose->orientation.w << ", "
                             << desired_pose->orientation.x << ", "
                             << desired_pose->orientation.y << ", "
                             << desired_pose->orientation.z << "]");

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

      old_quaternion.w() = desired_pose->orientation.w;
      old_quaternion.x() = desired_pose->orientation.x;
      old_quaternion.y() = desired_pose->orientation.y;
      old_quaternion.z() = desired_pose->orientation.z;
      old_quaternion.normalize();

      if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return CallbackReturn::FAILURE;
      }

      start_flag.store(true);

      // Try-catch version
      control_thread = std::thread{[this]() {
        // Configuring real time thread
        try {

          // EE to K transform thread
          // std::thread{[this]() {
          //   RCLCPP_INFO_STREAM(this->get_logger(),
          //                      "Started EE_to_K (wrist in touch)
          //                      thread");
          //   while (start_flag.load() && rclcpp::ok()) {
          //     try {
          //       if (wrist_contact_frame.has_value()) {
          //         auto transform = tf_buffer->lookupTransform(
          //             frame_id_name, wrist_contact_frame.value(),
          //             tf2::TimePointZero);
          //
          //         Eigen::Affine3d eigen_ee_to_wrist;
          //         eigen_ee_to_wrist =
          //             tf2::transformToEigen(transform.transform);
          //
          //         Eigen::Matrix4d matrix_ee_to_wrist =
          //             eigen_ee_to_wrist.matrix();
          //
          //         std::array<double, 16> temp_array;
          //         Eigen::Map<Eigen::Matrix4d>(temp_array.data()) =
          //             matrix_ee_to_wrist;
          //         EE_to_K_transform = temp_array;
          //       } else {
          //         EE_to_K_transform = std::nullopt;
          //       }
          //     } catch (tf2::LookupException &ex) {
          //       RCLCPP_ERROR_STREAM(
          //           this->get_logger(),
          //           "Could not lookup transform (LookupException): "
          //               << ex.what());
          //       EE_to_K_transform = std::nullopt;
          //     }
          //   }
          //   RCLCPP_INFO_STREAM(this->get_logger(),
          //                      "Stopped EE_to_K (wrist in touch)
          //                      thread");
          // }}.detach();

          // TFs publisher thread according to fr3 link names
          // std::thread{[this]() {
          //   if (realtime_tools::configure_sched_fifo(95)) {
          //     RCLCPP_INFO(this->get_logger(),
          //                 "Set real time priority for publisher thread");
          //   } else {
          //     RCLCPP_ERROR(this->get_logger(),
          //                  "Real time priority not set for publisher thread,
          //                  " "shutting down");
          //     start_flag.store(false);
          //     rclcpp::shutdown();
          //   }
          //   RCLCPP_INFO_STREAM(this->get_logger(),
          //                      "Started TFs publisher thread");
          //   while (start_flag.load() && rclcpp::ok()) {
          //     if (panda_franka_state.mut.try_lock() &&
          //         panda_franka_model.has_value()) {
          //       franka::RobotState current_state;
          //       rclcpp::Time now;
          //       if (panda_franka_state.state.has_value()) {
          //         current_state = panda_franka_state.state.value();
          //         now = panda_franka_state.state_time;
          //         panda_franka_state.state = std::nullopt;
          //       } else {
          //         RCLCPP_ERROR(this->get_logger(), "No available state");
          //         continue;
          //       }
          //
          //       int i = 1;
          //       franka::Frame frame = franka::Frame::kJoint1;
          //       geometry_msgs::msg::TransformStamped tf_stamped;
          //       tf2::Transform tf;
          //       tf_stamped.header.frame_id =
          //           panda_interface_names::panda_link_names[0];
          //       while (frame != franka::Frame::kEndEffector) {
          //         auto transform =
          //             panda_franka_model->pose(frame, current_state);
          //         auto pose = get_pose(transform);
          //         tf2::fromMsg(pose, tf);
          //         tf_stamped.transform = tf2::toMsg(tf);
          //         tf_stamped.child_frame_id =
          //             panda_interface_names::panda_link_names[i];
          //         tf_broadcaster->sendTransform(tf_stamped);
          //         frame++;
          //         i++;
          //       }
          //     } else {
          //       RCLCPP_ERROR(this->get_logger(),
          //                    "No robot model or couldnt lock state");
          //     }
          //     std::this_thread::sleep_for(5ms);
          //   }
          //   RCLCPP_INFO_STREAM(this->get_logger(),
          //                      "Stopped TFs publisher thread");
          // }}.detach();

          // Debug publisher thread
          std::thread{[this]() {
            using namespace std::chrono_literals;
            // if (realtime_tools::configure_sched_fifo(95)) {
            //   RCLCPP_INFO(this->get_logger(),
            //               "Set real time priority for publisher thread");
            // } else {
            //   RCLCPP_ERROR(this->get_logger(),
            //                "Real time priority not set for publisher thread "
            //                "shutting down");
            //   start_flag.store(false);
            //   rclcpp::shutdown();
            // }
            JointsEffort cmd;
            PoseStamped pose_debug;
            TwistStamped twist_debug;
            AccelStamped accel_debug;
            panda_interfaces::msg::DoubleStamped double_stamped;
            panda_interfaces::msg::DoubleArrayStamped arr_stamped;
            arr_stamped.data.resize(7);

            RCLCPP_INFO(this->get_logger(), "Started print control thread");
            while (start_flag.load() && rclcpp::ok()) {
              std::this_thread::sleep_for(5ms);
              try {

                debug_pub.data().mut.lock();
                // JOINT VALUES PUBLISHER
                //////////////////////////////////////////
                // if (debug_pub.data().robot_state.has_value()) {
                //   publish_robot_state_libfranka(
                //       debug_pub.data().robot_state.value());
                // }
                //////////////////////////////////////////

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
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Starting control thread with real time robot");
          panda_franka->control(robot_control_callback);
        } catch (const franka::Exception &ex) {
          start_flag.store(false);
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
          rclcpp::shutdown();
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
    case Mode::sim_franka: {

      // Start control loop thread
      start_flag.store(true);
      control_thread = std::thread{
          std::bind(&ImpedanceController::control_libfranka_sim, this)};
      RCLCPP_INFO(
          this->get_logger(),
          "Started control thread with simulated robot and libfranka lib");
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
  // Mode
  Mode mode = Mode::sim;
  // Subscribers
  rclcpp::Subscription<JointState>::SharedPtr robot_joint_states_sub{};
  rclcpp::Subscription<panda_interfaces::msg::CartesianCommand>::SharedPtr
      cartesian_cmd_sub{};
  rclcpp::Subscription<JointTorqueMeasureStamped>::SharedPtr external_tau_sub{};

  // Commands publisher
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub{};

  // Robot pose publisher and debug
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      robot_pose_pub{};
  realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_pub{};
  Publisher<std_msgs::msg::String>::SharedPtr wrist_contact_index_pub{};

  // DEBUG PUBLISHERS
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

  // Services
  rclcpp::Service<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_server;

  rclcpp::Service<panda_interfaces::srv::WristContact>::SharedPtr
      wrist_contact_server;

  // Robot related variables
  Robot<7> panda_mine;
  panda::RobotModel panda;
  std::optional<franka::Robot> panda_franka;
  std::optional<franka::Model> panda_franka_model;
  robot_state panda_franka_state;
  std::optional<std::string> wrist_contact_frame{std::nullopt};
  std::optional<std::array<double, 16>> EE_to_K_transform{std::nullopt};
  Eigen::Quaterniond old_quaternion;
  Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();

  std::mutex joint_state_mutex;
  JointState::SharedPtr current_joint_config{nullptr};
  Eigen::Vector<double, 7> extern_tau;
  std::vector<double> world_base_link;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      robot_control_callback;
  JointState joint_state_to_pub{};
  double lambda;
  debug_data print_debug;

  std::mutex desired_pose_mutex;
  Pose::SharedPtr desired_pose;
  Pose desired_pose_raw{};
  std::mutex desired_twist_mutex;
  std::mutex desired_accel_mutex;
  Eigen::Vector<double, 6> desired_twist_vec = Eigen::Vector<double, 6>::Zero();
  Eigen::Vector<double, 6> desired_accel_vec = Eigen::Vector<double, 6>::Zero();
  Twist desired_twist{};
  Accel desired_accel{};

  Eigen::Vector<double, 7> last_joints_speed = Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 7> current_joints_speed =
      Eigen::Vector<double, 7>::Zero();

  panda_interfaces::msg::CartesianCommand desired_cartesian_cmd{};
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

  // Control loop related variables
  rclcpp::Rate::SharedPtr control_loop_rate;
  double Kp{};
  double Kd{};
  double Md{};
  double joint_speed_safe_limit{};
  double percentage_effort_safe_limit{};
  double error_pose_norm_safe_limit{};
  std::thread control_thread;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> compliance_mode{false};
  bool clamp;
  Eigen::VectorXd last_control_input;

  Eigen::Vector<double, 7>
  joint_limit_index_grad(const Eigen::Vector<double, 7> &current_joint_config,
                         double step = 1e-3) {
    double index_0;
    double index_forwarded;
    Eigen::Vector<double, 7> limit_index_grad{};
    Eigen::Vector<double, 7> forward_joint_config{};

    // Calculating index 0
    index_0 = joint_limit_index(current_joint_config);

    // Calculating gradient
    for (int i = 0; i < forward_joint_config.size(); i++) {

      forward_joint_config = current_joint_config;
      forward_joint_config[i] = forward_joint_config[i] + step;
      index_forwarded = joint_limit_index(forward_joint_config);
      limit_index_grad(i) = (index_forwarded - index_0) / step;
    }
    return limit_index_grad;
  }

  double
  joint_limit_index(const Eigen::Vector<double, 7> &current_joint_config) {
    double index = 0;
    for (int i = 0; i < current_joint_config.size(); i++) {
      index = index + 1 / (current_joint_config[i] - joint_min_limits[i]) +
              1 / (joint_max_limits[i] - current_joint_config[i]);
    }
    return index;
  }

  Eigen::Vector<double, 7>
  manip_grad(const Eigen::VectorXd &current_joint_config, double step = 1e-3) {
    double index_0;
    double index_forwarded;
    Eigen::Vector<double, 7> manip_grad;
    Eigen::Matrix<double, 6, 7> jacob;
    franka::RobotState current_state;
    for (int i = 0; i < current_joint_config.size(); i++) {
      current_state.q[i] = current_joint_config[i];
    }

    if (panda_franka_model.has_value()) {
      // Calculating index 0
      auto jacob_raw = panda_franka_model.value().zeroJacobian(
          franka::Frame::kFlange, current_state);
      for (size_t col = 0; col < 7; col++) {
        for (size_t row = 0; row < 6; row++) {
          jacob(row, col) = jacob_raw[col * 7 + row];
        }
      }
      index_0 = manip_index(jacob);

      // Calculating gradient
      for (int i = 0; i < 7; i++) {
        franka::RobotState forward_state = current_state;
        forward_state.q[i] = forward_state.q[i] + step;
        auto jacob_raw = panda_franka_model.value().zeroJacobian(
            franka::Frame::kFlange, current_state);
        for (size_t col = 0; col < 7; col++) {
          for (size_t row = 0; row < 6; row++) {
            jacob(row, col) = jacob_raw[col * 7 + row];
          }
        }
        index_forwarded = manip_index(jacob);
        manip_grad(i) = (index_forwarded - index_0) / step;
      }
    } else {

      // Calculating index 0
      panda.computeAll(current_joint_config, current_joint_config);
      jacob = panda.getGeometricalJacobian(frame_id_name);
      index_0 = manip_index(jacob);

      // Calculating gradient
      for (int i = 0; i < 7; i++) {
        Eigen::VectorXd forward_joint_config = current_joint_config;
        forward_joint_config[i] = forward_joint_config[i] + step;
        panda.computeAll(forward_joint_config, forward_joint_config);
        jacob = panda.getGeometricalJacobian(frame_id_name);
        index_forwarded = manip_index(jacob);
        manip_grad(i) = (index_forwarded - index_0) / step;
      }
    }
    return manip_grad;
  }

  double manip_index(const Eigen::Matrix<double, 6, 7> &jacob) {
    return std::sqrt((jacob * jacob.transpose()).determinant());
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

  void clamp_control_speed(Eigen::Vector<double, 7> &control_input,
                           const double dt) {

    for (int i = 0; i < control_input.size(); i++) {
      if ((abs(control_input[i] - last_control_input[i]) / dt) >
          effort_speed_limits[i]) {
        RCLCPP_INFO_ONCE(this->get_logger(), "CLAMPED INPUT TORQUE VALUE");
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

  std::array<double, 16>
  get_transform_matrix(const geometry_msgs::msg::Pose &pose_msg) {
    std::array<double, 16> transform_mat;

    // Convert quaternion to rotation matrix
    Eigen::Quaterniond q(pose_msg.orientation.w, pose_msg.orientation.x,
                         pose_msg.orientation.y, pose_msg.orientation.z);
    q.normalize();

    Eigen::Matrix3d rotation = q.toRotationMatrix();

    // Create 4x4 transformation matrix
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = rotation;
    mat(0, 3) = pose_msg.position.x;
    mat(1, 3) = pose_msg.position.y;
    mat(2, 3) = pose_msg.position.z;

    // Map to std::array in **column-major** layout
    Eigen::Map<Eigen::Matrix4d>(transform_mat.data()) = mat;

    return transform_mat;
  }

  Eigen::Matrix<double, 7, 6>
  compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian,
                          const double &lambda) {
    return jacobian.transpose() *
           (jacobian * jacobian.transpose() +
            lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity())
               .inverse();
  }

  void publish_robot_state_libfranka(franka::RobotState state) {

    // Publish robot pose stamped
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.pose = get_pose(
        panda_franka_model.value().pose(franka::Frame::kFlange, state));
    current_pose.header.stamp = this->now();
    // robot_pose_pub->try_publish(current_pose);

    // Publish joint state
    joint_state_to_pub.header.stamp = this->now();
    for (size_t i = 0; i < joint_state_to_pub.position.size(); i++) {
      joint_state_to_pub.position[i] = state.q[i];
      joint_state_to_pub.velocity[i] = state.dq[i];
      joint_state_to_pub.effort[i] = state.tau_J[i];
    }

    joint_states_pub->try_publish(joint_state_to_pub);
  }

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

  rclcpp::Subscription<Pose>::SharedPtr desired_pose_sub{};
  rclcpp::Subscription<Twist>::SharedPtr desired_twist_sub{};
  rclcpp::Subscription<Accel>::SharedPtr desired_accel_sub{};
  DebugPublisher debug_pub;
};

void ImpedanceController::control() {
  // Impedance controller

  Eigen::Vector<double, 7> non_linear_effects;
  Eigen::Vector<double, 7> control_input;
  Eigen::Vector<double, 6> h_e;
  Eigen::VectorXd gravity;
  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 6> y_cartesian;
  Eigen::Matrix<double, 7, 6> jacobian_pinv;
  Eigen::Vector<double, 7> current_joints_config_vec;
  Eigen::Vector<double, 7> current_joints_speed =
      Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 7> last_joints_speed = Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 6> current_twist;
  Eigen::Vector<double, 6> error_twist;
  Pose current_pose_tmp;
  Eigen::Quaterniond current_quat;
  Eigen::Quaterniond error_quat{};
  Eigen::Vector<double, 6> error_pose_vec{};
  Eigen::Quaterniond desired_quat{};
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
  }

  // Get current pose in simulation environment
  current_joint_config = nullptr;
  while (!current_joint_config) {
  }
  for (size_t i = 0; i < current_joint_config->position.size(); i++) {
    current_joints_config_vec[i] = current_joint_config->position[i];
  }
  panda.computeAll(current_joints_config_vec, Eigen::Vector<double, 7>::Zero());
  desired_pose = std::make_shared<Pose>(panda.getPose(frame_id_name));

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Desired Position: ["
          << desired_pose->position.x << ", " << desired_pose->position.y
          << ", " << desired_pose->position.z << "]; Orientation: [] "
          << desired_pose->orientation.w << ", " << desired_pose->orientation.x
          << ", " << desired_pose->orientation.y << ", "
          << desired_pose->orientation.z << "]");

  // Coefficient for dynamic lambda damping
  double alpha = this->get_parameter("alpha").as_double();
  double k_max = this->get_parameter("k_max").as_double();
  double eps = this->get_parameter("eps").as_double();

  double task_gain = this->get_parameter("task_gain").as_double();

  double orient_scale = 1.0;
  Eigen::Vector<double, 6> KP_{
      Kp, Kp, Kp, Kp * orient_scale, Kp * orient_scale, Kp * orient_scale};
  Eigen::Matrix<double, 6, 6> KP = Eigen::Matrix<double, 6, 6>::Identity();
  KP.diagonal() = KP_;

  Eigen::Vector<double, 6> KD_{
      Kd, Kd, Kd, Kd * orient_scale, Kd * orient_scale, Kd * orient_scale};
  Eigen::Matrix<double, 6, 6> KD = Eigen::Matrix<double, 6, 6>::Identity();
  KD.diagonal() = KD_;

  Eigen::Vector<double, 6> MD_{Md, Md, Md, Md, Md, Md};
  Eigen::Matrix<double, 6, 6> MD = Eigen::Matrix<double, 6, 6>::Identity();
  MD.diagonal() = MD_;
  auto MD_1 = MD.inverse();

  RCLCPP_INFO_STREAM(this->get_logger(), "MD matrix and inverse: " << std::endl
                                                                   << MD
                                                                   << std::endl
                                                                   << MD_1);

  while (start_flag.load() && rclcpp::ok()) {

    // RCLCPP_INFO(this->get_logger(), "Entered cycle");
    pose_debug.header.stamp = this->now();
    twist_debug.header.stamp = this->now();
    accel_debug.header.stamp = this->now();
    arr_stamped.header.stamp = this->now();
    double_stamped.header.stamp = this->now();
    y_cartesian_stamped.header.stamp = this->now();
    sigma.header.stamp = this->now();

    {

      std::lock_guard<std::mutex> lock(joint_state_mutex);
      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_config_vec[i] = current_joint_config->position[i];
      }

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        // current_joints_speed[i] = current_joint_config->velocity[i];
        current_joints_speed[i] =
            franka::lowpassFilter(0.001, current_joint_config->velocity[i],
                                  current_joints_speed[i], 20.0);
      }
    }

    panda.computeAll(current_joints_config_vec, current_joints_speed);

    // Get current pose
    current_pose_tmp = panda.getPose(frame_id_name);
    current_quat.w() = current_pose_tmp.orientation.w;
    current_quat.x() = current_pose_tmp.orientation.x;
    current_quat.y() = current_pose_tmp.orientation.y;
    current_quat.z() = current_pose_tmp.orientation.z;
    current_quat.normalize();
    current_quat = quaternionContinuity(current_quat, old_quaternion);
    old_quaternion = current_quat;

    // Calculate pose error
    {
      std::lock_guard<std::mutex> lock(desired_pose_mutex);

      // Get desired pose
      desired_quat.w() = desired_pose->orientation.w;
      desired_quat.x() = desired_pose->orientation.x;
      desired_quat.y() = desired_pose->orientation.y;
      desired_quat.z() = desired_pose->orientation.z;
      desired_quat.normalize();

      error_quat = desired_quat * current_quat.inverse();
      error_quat.normalize();

      error_pose_vec(0) =
          desired_pose->position.x - current_pose_tmp.position.x;
      error_pose_vec(1) =
          desired_pose->position.y - current_pose_tmp.position.y;
      error_pose_vec(2) =
          desired_pose->position.z - current_pose_tmp.position.z;

      // Quaternion vec error
      error_pose_vec(3) = error_quat.x();
      error_pose_vec(4) = error_quat.y();
      error_pose_vec(5) = error_quat.z();

      // error_pose_vec(3) = 2 * error_quat.x();
      // error_pose_vec(4) = 2 * error_quat.y();
      // error_pose_vec(5) = 2 * error_quat.z();
    }

    // Update robot model
    //

    // Calculate quantities for control
    //
    if (compliance_mode.load() && wrist_contact_frame.has_value()) {
      jacobian = panda.getGeometricalJacobian("fr3_joint4");
    } else {
      jacobian = panda.getGeometricalJacobian(frame_id_name);
    }

    Eigen::Matrix<double, 6, 6> T_a;
    // {
    //   Eigen::Vector3d euler_angles =
    //   current_quat.toRotationMatrix().eulerAngles(2, 1, 2);
    //   auto T = computeTMatrix(euler_angles);
    //   T_a.setZero();
    //   T_a.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    //   T_a.block<3, 3>(3, 3) = T.inverse();
    //   jacobian = T_a.inverse() * jacobian;
    // }

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

    // DYNAMIC LAMBDA BASED ON MINIMUM SINGULAR VALUE
    //
    // Exponential
    // double lambda = std::exp(-alpha * sigma_min);
    // lambda = (lambda < 1e-4 ? 0.0 : lambda);

    // Quadratic
    double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
    lambda = (sigma_min >= eps ? 0.0 : lambda);

    jacobian_pinv = compute_jacob_pseudoinv(jacobian, lambda);

    B_a = (jacobian * mass_matrix.inverse() * jacobian.transpose() +
           lambda * Eigen::Matrix<double, 6, 6>::Identity())
              .inverse();

    jacobian_cap = mass_matrix.inverse() * jacobian.transpose() * B_a;

    tau_projector = Eigen::Matrix<double, 7, 7>::Identity() -
                    jacobian.transpose() * jacobian_cap.transpose();

    {
      std::lock_guard<std::mutex> lock(desired_twist_mutex);
      current_twist = jacobian * current_joints_speed;
      error_twist = desired_twist_vec - current_twist;
    }

    {
      std::lock_guard<std::mutex> lock(desired_accel_mutex);
      // h_e = jacobian_pinv.transpose() * extern_tau;

      if (compliance_mode.load()) {

        y = jacobian_pinv * MD_1 *
            (

                -KD * current_twist -
                MD * panda.computeHessianTimesQDot(current_joints_config_vec,
                                                   current_joints_speed,
                                                   frame_id_name)
                // -
                //       h_e

            );
        // control_input =
        //     mass_matrix * y + non_linear_effects + extern_tau - gravity;
        control_input = mass_matrix * y + non_linear_effects - gravity;
      } else {
        y_cartesian =
            MD_1 *
            (MD * desired_accel_vec + KD * error_twist + KP * error_pose_vec
             // -
             //    MD *
             //        get_j_dot(get_jacob, current_joints_config_vec,
             //                  current_joints_speed) *
             //        current_joints_speed
             // MD *
             // panda.computeHessianTimesQDot(current_joints_config_vec,
             //                                    current_joints_speed,
             //                                    frame_id_name)
             // -
             //    h_e
            );

        y = jacobian_pinv * y_cartesian;
        control_input = mass_matrix * y + non_linear_effects - gravity;
      }
    }

    control_input =
        mass_matrix * y + non_linear_effects - gravity +
        task_gain * tau_projector * (manip_grad(current_joints_config_vec));

    // control_input = mass_matrix * y + non_linear_effects + extern_tau -
    //                 gravity +
    //                 task_gain * tau_projector *
    //                     (-joint_limit_index_grad(current_joints_config_vec));

    // Clamping control input
    //
    clamp_control(control_input);

    clamp_control_speed(control_input,
                        (this->now() - last_control_cycle).seconds());

    // Apply control
    //
    publish_efforts(control_input + gravity);

    // Save last control input
    //
    last_control_input = control_input;
    last_control_cycle = this->now();

    pose_debug.pose = current_pose_tmp;
    // robot_pose_pub->tryPublish(pose_debug);

    if (debug_pub.data().mut.try_lock()) {

      debug_pub.data().h_e = h_e;
      debug_pub.data().tau_ext = extern_tau;
      debug_pub.data().lambda = lambda;
      debug_pub.data().sigma_min = sigma_min;
      debug_pub.data().current_twist = current_twist;
      debug_pub.data().current_pose = current_pose_tmp;
      debug_pub.data().des_twist = desired_twist;
      debug_pub.data().des_pose = *desired_pose;
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
          get_transform_matrix(current_pose_tmp);

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

void ImpedanceController::control_libfranka_sim() {
  // Impedance controller

  Eigen::Vector<double, 7> non_linear_effects;
  Eigen::Vector<double, 7> control_input;
  Eigen::Vector<double, 7> gravity;
  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 6> y_cartesian;
  Eigen::Matrix<double, 7, 6> jacobian_pinv;
  Eigen::Vector<double, 7> last_joints_speed = Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 7> tau_ext;
  Eigen::Vector<double, 7> current_joints_config_vec;
  Eigen::Vector<double, 7> current_joints_speed =
      Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 6> current_twist;
  Eigen::Vector<double, 6> error_twist;
  Eigen::Vector<double, 6> h_e;
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

  Eigen::Affine3d ident = Eigen::Affine3d::Identity();
  Eigen::Matrix4d matrix_ident = ident.matrix();
  std::array<double, 16> ident_transform;
  Eigen::Map<Eigen::Matrix4d>(ident_transform.data()) = matrix_ident;

  franka::RobotState state;

  RCLCPP_INFO(this->get_logger(), "Waiting for simulation to start");
  rclcpp::Time last_control_cycle = this->now();
  while (rclcpp::Time{current_joint_config->header.stamp} - this->now() ==
         rclcpp::Duration{0, 0}) {
  }

  // Get current pose in simulation environment
  current_joint_config = nullptr;
  while (!current_joint_config) {
  }
  for (size_t i = 0; i < current_joint_config->position.size(); i++) {
    state.q[i] = current_joint_config->position[i];
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Joint " << i + 1 << ": " << state.q[i]);
  }
  desired_pose = std::make_shared<Pose>(
      get_pose(panda_franka_model->pose(franka::Frame::kFlange, state)));

  auto get_jacob = [this](const Eigen::Vector<double, 7> &current_joint_pos) {
    auto state = franka::RobotState{};
    for (size_t i = 0; i < state.q.size(); i++) {
      state.q[i] = current_joint_pos[i];
    }

    return get_jacobian(
        this->panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
  };

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Desired Position: ["
          << desired_pose->position.x << ", " << desired_pose->position.y
          << ", " << desired_pose->position.z << "]; Orientation: [] "
          << desired_pose->orientation.w << ", " << desired_pose->orientation.x
          << ", " << desired_pose->orientation.y << ", "
          << desired_pose->orientation.z << "]");

  // Coefficient for dynamic lambda damping
  double alpha = this->get_parameter("alpha").as_double();
  double k_max = this->get_parameter("k_max").as_double();
  double eps = this->get_parameter("eps").as_double();

  double task_gain = this->get_parameter("task_gain").as_double();

  double orient_scale = 5;
  Eigen::Vector<double, 6> KP_{
      Kp, Kp, Kp, Kp * orient_scale, Kp * orient_scale, Kp * orient_scale};
  Eigen::Matrix<double, 6, 6> KP = Eigen::Matrix<double, 6, 6>::Identity();
  KP.diagonal() = KP_;

  Eigen::Vector<double, 6> KD_{
      Kd, Kd, Kd, Kd * orient_scale, Kd * orient_scale, Kd * orient_scale};
  Eigen::Matrix<double, 6, 6> KD = Eigen::Matrix<double, 6, 6>::Identity();
  KD.diagonal() = KD_;

  Eigen::Vector<double, 6> MD_{Md, Md, Md, Md, Md, Md};
  Eigen::Matrix<double, 6, 6> MD = Eigen::Matrix<double, 6, 6>::Identity();
  MD.diagonal() = MD_;
  Eigen::Matrix<double, 6, 6> MD_1 = MD.inverse();

  RCLCPP_INFO_STREAM(this->get_logger(), "MD matrix and inverse: " << std::endl
                                                                   << MD
                                                                   << std::endl
                                                                   << MD_1);

  if (realtime_tools::configure_sched_fifo(99)) {
    RCLCPP_INFO(this->get_logger(), "Set real time priority");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Real time priority not set, shutting down");
    start_flag.store(false);
    rclcpp::shutdown();
  }

  while (start_flag.load() && rclcpp::ok()) {

    pose_debug.header.stamp = this->now();

    {

      std::lock_guard<std::mutex> lock(joint_state_mutex);
      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        current_joints_config_vec[i] = current_joint_config->position[i];
        state.q[i] = current_joint_config->position[i];
      }

      for (size_t i = 0; i < current_joint_config->position.size(); i++) {
        // current_joints_speed[i] = current_joint_config->velocity[i];
        current_joints_speed[i] =
            franka::lowpassFilter(0.001, current_joint_config->velocity[i],
                                  current_joints_speed[i], 50.0);
        state.dq[i] = current_joint_config->velocity[i];
      }
    }

    ///////////////////////////////////
    // {
    //
    //   // Using franka robot measured state
    //   //
    //   state = panda_franka->readOnce();
    //
    //   std::lock_guard<std::mutex> lock(joint_state_mutex);
    //   for (size_t i = 0; i < current_joint_config->position.size(); i++) {
    //     current_joints_config_vec[i] = state.q[i];
    //   }
    //
    //   for (size_t i = 0; i < current_joint_config->position.size(); i++) {
    //     current_joints_speed[i] = state.dq[i];
    //   }
    // }
    ///////////////////////////////////

    // Get current pose
    Pose current_pose;
    Eigen::Matrix<double, 6, 7> jacobian;
    if (compliance_mode.load() && EE_to_K_transform.has_value()) {
      // I have to get Kstiffness frame pose if also EE_to_K has value
      panda_franka->setK(EE_to_K_transform.value());
      current_pose =
          get_pose(panda_franka_model->pose(franka::Frame::kStiffness, state));
      jacobian = get_jacobian(
          panda_franka_model->zeroJacobian(franka::Frame::kStiffness, state));
    } else {
      // panda_franka->setK(ident_transform);
      current_pose =
          get_pose(panda_franka_model->pose(franka::Frame::kFlange, state));
      RCLCPP_INFO_STREAM_ONCE(
          this->get_logger(),
          "Current position: ["
              << current_pose.position.x << ", " << current_pose.position.y
              << ", " << current_pose.position.z << "]; Orientation: [] "
              << current_pose.orientation.w << ", "
              << current_pose.orientation.x << ", "
              << current_pose.orientation.y << ", "
              << current_pose.orientation.z << "]");
      jacobian = get_jacobian(
          panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                              "Current jacobian: [" << jacobian << "]");
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
    Eigen::Quaterniond error_quat{};
    Eigen::Quaterniond quat_des_dot{};
    Eigen::Quaterniond current_quat_dot{};
    Eigen::Vector<double, 6> error_pose_vec{};
    {
      std::lock_guard<std::mutex> lock(desired_pose_mutex);

      // Get desired pose
      Eigen::Quaterniond desired_quat{};
      desired_quat.w() = desired_pose_raw.orientation.w;
      desired_quat.x() = desired_pose_raw.orientation.x;
      desired_quat.y() = desired_pose_raw.orientation.y;
      desired_quat.z() = desired_pose_raw.orientation.z;
      desired_quat.normalize();

      error_quat = desired_quat * current_quat.inverse();
      error_quat.normalize();

      error_pose_vec(0) = desired_pose_raw.position.x - current_pose.position.x;
      error_pose_vec(1) = desired_pose_raw.position.y - current_pose.position.y;
      error_pose_vec(2) = desired_pose_raw.position.z - current_pose.position.z;

      // Axis angle error
      error_pose_vec(3) = error_quat.x();
      error_pose_vec(4) = error_quat.y();
      error_pose_vec(5) = error_quat.z();
    }

    // RCLCPP_INFO(this->get_logger(), "Calculated jacob");
    // Doesn't account for friction
    // B(q)
    std::array<double, 49> mass_matrix_raw =
        this->panda_franka_model.value().mass(state);
    Eigen::Matrix<double, 7, 7> mass_matrix;
    for (size_t i = 0; i < 7; i++) {
      for (size_t j = 0; j < 7; j++) {
        mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
      }
    }
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                            "Current mass matrxi: [" << mass_matrix << "]");

    // Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
    //     this->panda_franka_model.value().coriolis(state).data());
    std::array<double, 7> coriolis_raw =
        this->panda_franka_model.value().coriolis(state);
    Eigen::Vector<double, 7> coriolis;
    for (size_t i = 0; i < coriolis_raw.size(); i++) {
      coriolis(i) = coriolis_raw[i];
    }
    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000.0,
        "Current coriolis: [" << coriolis[0] << ", " << coriolis[1] << ", "
                              << coriolis[2] << ", " << coriolis[3] << ", "
                              << coriolis[4] << ", " << coriolis[5] << ", "
                              << coriolis[6] << "]");

    std::array<double, 7> gravity_raw =
        this->panda_franka_model.value().gravity(state);
    for (size_t i = 0; i < gravity_raw.size(); i++) {
      gravity(i) = gravity_raw[i];
    }
    gravity = panda.getGravityVector(current_joints_config_vec);
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                            "Current gravity: ["
                                << gravity[0] << ", " << gravity[1] << ", "
                                << gravity[2] << ", " << gravity[3] << ", "
                                << gravity[4] << ", " << gravity[5] << ", "
                                << gravity[6] << "]");

    // Calculate jacobian SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);
    double sigma_min = svd.singularValues().tail(1)(0);
    // DYNAMIC LAMBDA BASED ON MINIMUM SINGULAR VALUE
    //

    // Quadratic
    double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
    lambda = (sigma_min >= eps ? 0.0 : lambda);

    jacobian_pinv = compute_jacob_pseudoinv(jacobian, lambda);

    RCLCPP_INFO_STREAM_ONCE(
        this->get_logger(),
        "Current error: [" << error_pose_vec[0] << ", " << error_pose_vec[1]
                           << ", " << error_pose_vec[2] << ", "
                           << error_pose_vec[3] << ", " << error_pose_vec[4]
                           << ", " << error_pose_vec[5] << "]");

    {
      std::lock_guard<std::mutex> lock(desired_twist_mutex);
      current_twist = jacobian * current_joints_speed;
      error_twist = desired_twist_vec - current_twist;
    }

    auto current_state = panda_franka->readOnce();
    // Get external forces acting on robot
    for (int i = 0; i < tau_ext.size(); i++) {
      tau_ext[i] = current_state.tau_ext_hat_filtered[i];
    }

    for (int i = 0; i < h_e.size(); i++) {
      h_e[i] = current_state.O_F_ext_hat_K[i];
    }

    {
      std::lock_guard<std::mutex> lock(desired_accel_mutex);
      if (compliance_mode.load()) {

        y = jacobian_pinv * MD_1 *
            (
                // The robot should not accept trajectory, so the desired
                // commands are ignored
                -KD * current_twist - MD *
                                          get_j_dot(get_jacob,
                                                    current_joints_config_vec,
                                                    current_joints_speed) *
                                          current_joints_speed

                // - h_e
            );
      } else {
        y_cartesian =
            MD_1 *
            (MD * desired_accel_vec + KD * error_twist + KP * error_pose_vec
             // - MD *
             //        get_j_dot(get_jacob, current_joints_config_vec,
             //                  current_joints_speed) *
             //        current_joints_speed
             // -
             //    h_e
            );

        y = jacobian_pinv * y_cartesian;
      }
    }

    // control_input = mass_matrix * y + coriolis + tau_ext;
    control_input = mass_matrix * y + coriolis;

    // Clamping control input
    //
    clamp_control(control_input);

    // Apply control
    //
    publish_efforts(control_input + gravity);

    // Save last control input
    //
    last_control_input = control_input;
    std::array<double, 7> tau;
    for (size_t i = 0; i < control_input.size(); ++i) {
      tau[i] = control_input[i];
    }

    pose_debug.pose = current_pose;
    // robot_pose_pub->tryPublish(pose_debug);

    // Fill struct for debug prints
    // if (print_debug.mut.try_lock()) {
    //   print_debug.current_twist = current_twist;
    //   print_debug.has_data = true;
    //   // Deep copy
    //   print_debug.robot_state = franka::RobotState(state);
    //   print_debug.sigma_min = sigma_min;
    //   print_debug.tau_d_last = tau;
    //   print_debug.gravity = panda_franka_model.value().gravity(state);
    //   print_debug.tau_ext = tau_ext;
    //   print_debug.h_e = h_e;
    //   print_debug.mut.unlock();
    // }

    if (debug_pub.data().mut.try_lock()) {

      debug_pub.data().h_e = h_e;
      debug_pub.data().tau_ext = extern_tau;
      debug_pub.data().lambda = lambda;
      debug_pub.data().sigma_min = sigma_min;
      debug_pub.data().current_pose = current_pose;
      debug_pub.data().current_twist = current_twist;
      debug_pub.data().current_j_dot_q_dot =
          get_j_dot(get_jacob, current_joints_config_vec,
                    current_joints_speed) *
          current_joints_speed;
      debug_pub.data().des_twist = desired_twist;
      debug_pub.data().des_pose = *desired_pose;
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
          get_transform_matrix(current_pose);

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
