#include "panda_interfaces/action/cart_traj.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <memory>
#include <random>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_wait_set_mask.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>
#include <string>
#include <thread>

using CartTraj = panda_interfaces::action::CartTraj;
using GoalHandleCartMove = rclcpp_action::ServerGoalHandle<CartTraj>;
using namespace std::chrono_literals;

double qintic(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return q_i;
  if (t >= t_f)
    return q_f;

  double tau = t / t_f;
  double q_cap =
      6 * std::pow(tau, 5) - 15 * std::pow(tau, 4) + 10 * std::pow(tau, 3);
  return q_i + (q_f - q_i) * q_cap;
}

double qintic_velocity(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap =
      30 * std::pow(tau, 4) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 2);
  return (q_f - q_i) * q_cap / t_f;
}

double qintic_accel(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap = 120 * std::pow(tau, 3) - 180 * std::pow(tau, 2) + 60 * tau;
  return (q_f - q_i) * q_cap / std::pow(t_f, 2);
}

class CartTrajectory : public rclcpp::Node {

public:
  CartTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node(panda_interface_names::cart_traj_node_name, opt) {

    this->declare_parameter<double>("loop_rate_freq", 1000.0);

    loop_rate_freq = this->get_parameter("loop_rate_freq").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loop rate frequency: " << loop_rate_freq);

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const CartTraj::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial pose: Position: ["
                             << goal->initial_pose.position.x << ", "
                             << goal->initial_pose.position.y << ", "
                             << goal->initial_pose.position.z
                             << "] Orientation(w, x, y, z): ["
                             << goal->initial_pose.orientation.w << ", "
                             << goal->initial_pose.orientation.x << ", "
                             << goal->initial_pose.orientation.y << ", "
                             << goal->initial_pose.orientation.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Final pose: Position: ["
                             << goal->desired_pose.position.x << ", "
                             << goal->desired_pose.position.y << ", "
                             << goal->desired_pose.position.z
                             << "] Orientation(w, x, y, z): ["
                             << goal->desired_pose.orientation.w << ", "
                             << goal->desired_pose.orientation.x << ", "
                             << goal->desired_pose.orientation.y << ", "
                             << goal->desired_pose.orientation.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(), "In " << goal->total_time << "s");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](const std::shared_ptr<
                                rclcpp_action::ServerGoalHandle<CartTraj>>
                                    goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      // CartTraj::Result::SharedPtr result =
      //     std::make_shared<CartTraj::Result>();
      // result->completed = false;
      // goal_handle->canceled(result);
      (void)goal_handle;
      std::thread{[this]() {
        RCLCPP_INFO(this->get_logger(), "Running exponential traj");
        std::map<int, decay_laws> translation_laws;
        std::map<int, decay_laws> orientation_laws;
        geometry_msgs::msg::Pose initial_pose = cmd_cartesian.pose;
        geometry_msgs::msg::Twist initial_velocity = cmd_cartesian.twist;
        geometry_msgs::msg::Accel initial_acceleration = cmd_cartesian.accel;

        translation_laws.emplace(0, decay_laws(1.5, initial_pose.position.x,
                                               initial_velocity.linear.x,
                                               initial_acceleration.linear.x));
        translation_laws.emplace(1, decay_laws(1.5, initial_pose.position.y,
                                               initial_velocity.linear.y,
                                               initial_acceleration.linear.y));
        translation_laws.emplace(2, decay_laws(1.5, initial_pose.position.z,
                                               initial_velocity.linear.z,
                                               initial_acceleration.linear.z));

        orientation_laws.emplace(0, decay_laws(1.5, 0.0,
                                               initial_velocity.angular.x,
                                               initial_acceleration.angular.x));
        orientation_laws.emplace(1, decay_laws(1.5, 0.0,
                                               initial_velocity.angular.y,
                                               initial_acceleration.angular.y));
        orientation_laws.emplace(2, decay_laws(1.5, 0.0,
                                               initial_velocity.angular.z,
                                               initial_acceleration.angular.z));
        Eigen::Quaterniond current_quat{
            initial_pose.orientation.w, initial_pose.orientation.x,
            initial_pose.orientation.y, initial_pose.orientation.z};
        current_quat.normalize();
        rclcpp::Time t0 = this->now();
        rclcpp::Duration t = rclcpp::Duration::from_seconds(0.0);
        rclcpp::Rate loop_rate(loop_rate_freq, this->get_clock());
        while (rclcpp::ok() && t.seconds() < 1.5) {
          t = this->get_clock()->now() - t0;
          RCLCPP_DEBUG(this->get_logger(), "Time now %f", t.seconds());

          RCLCPP_DEBUG(this->get_logger(), "Uploading next pose");

          // Calculating position
          cmd_cartesian.pose.position.x =
              translation_laws.at(0).exponential_decay_position(t.seconds());
          cmd_cartesian.pose.position.y =
              translation_laws.at(1).exponential_decay_position(t.seconds());
          cmd_cartesian.pose.position.z =
              translation_laws.at(2).exponential_decay_position(t.seconds());

          cmd_cartesian.twist.linear.x =
              translation_laws.at(0).exponential_decay_velocity(t.seconds());
          cmd_cartesian.twist.linear.y =
              translation_laws.at(1).exponential_decay_velocity(t.seconds());
          cmd_cartesian.twist.linear.z =
              translation_laws.at(2).exponential_decay_velocity(t.seconds());

          cmd_cartesian.accel.linear.x =
              translation_laws.at(0).exponential_decay_acceleration(
                  t.seconds());
          cmd_cartesian.accel.linear.y =
              translation_laws.at(1).exponential_decay_acceleration(
                  t.seconds());
          cmd_cartesian.accel.linear.z =
              translation_laws.at(2).exponential_decay_acceleration(
                  t.seconds());

          // Orientation
          // For the orientation we apply the decay laws to the twist and the
          // accel and we calculate a theta angle as the integral of the twist:
          // based on this theta we calculate a delta quaternion to apply to the
          // current quaternion
          cmd_cartesian.twist.angular.x =
              orientation_laws.at(0).exponential_decay_velocity(t.seconds());
          cmd_cartesian.twist.angular.y =
              orientation_laws.at(1).exponential_decay_velocity(t.seconds());
          cmd_cartesian.twist.angular.z =
              orientation_laws.at(2).exponential_decay_velocity(t.seconds());

          Eigen::Quaterniond delta_quat{
              1.0, 0.5 * cmd_cartesian.twist.angular.x / loop_rate_freq,
              0.5 * cmd_cartesian.twist.angular.y / loop_rate_freq,
              0.5 * cmd_cartesian.twist.angular.z / loop_rate_freq};
          delta_quat.normalize();

          current_quat = current_quat * delta_quat;
          current_quat.normalize();

          cmd_cartesian.pose.orientation.x = current_quat.x();
          cmd_cartesian.pose.orientation.y = current_quat.y();
          cmd_cartesian.pose.orientation.z = current_quat.z();
          cmd_cartesian.pose.orientation.w = current_quat.w();

          cmd_cartesian.accel.angular.x =
              orientation_laws.at(0).exponential_decay_acceleration(
                  t.seconds());
          cmd_cartesian.accel.angular.y =
              orientation_laws.at(1).exponential_decay_acceleration(
                  t.seconds());
          cmd_cartesian.accel.angular.z =
              orientation_laws.at(2).exponential_decay_acceleration(
                  t.seconds());

          RCLCPP_DEBUG_ONCE(this->get_logger(), "Publish command");

          cartesian_cmd_pub->try_publish(cmd_cartesian);

          // Sleep
          //
          loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Finished running exponential traj");
      }}.detach();
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleCartMove> goal_handle) {
          using namespace std::placeholders;

          std::thread{std::bind(&CartTrajectory::execute, this, _1),
                      goal_handle}
              .detach();
        };

    this->action_traj_server = rclcpp_action::create_server<CartTraj>(
        this, panda_interface_names::panda_cart_move_action_name, handle_goal,
        handle_cancel, handle_accepted);

    cartesian_cmd_pub = std::make_shared<realtime_tools::RealtimePublisher<
        panda_interfaces::msg::CartesianCommand>>(
        this->create_publisher<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd",
            panda_interface_names::DEFAULT_TOPIC_QOS()));
  }

private:

  realtime_tools::RealtimePublisherSharedPtr<
      panda_interfaces::msg::CartesianCommand>
      cartesian_cmd_pub;
    panda_interfaces::msg::CartesianCommand cmd_cartesian;

  rclcpp_action::Server<CartTraj>::SharedPtr action_traj_server;

  double loop_rate_freq{};

  void execute(const std::shared_ptr<GoalHandleCartMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    if (realtime_tools::has_realtime_kernel() &&
        !this->get_parameter("use_sim_time").as_bool()) {
      if (realtime_tools::has_realtime_kernel()) {
        if (!realtime_tools::configure_sched_fifo(96)) {
          RCLCPP_WARN(this->get_logger(),
                      "Execute thread: Could not set SCHED_FIFO."
                      " Running with default scheduler.");
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Execute thread: Set SCHED_FIFO priority.");
        }
      } else if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation: realtime not requested");
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Execute thread: No real-time kernel detected.");
      }
    }

    rclcpp::Rate loop_rate(1000.0);

    RCLCPP_INFO(this->get_logger(), "Getting goal");
    const auto goal = goal_handle->get_goal();

    auto node = std::make_shared<rclcpp::Node>("wait_pose");
    geometry_msgs::msg::PoseStamped initial_pose;
    rclcpp::wait_for_message<geometry_msgs::msg::PoseStamped>(
        initial_pose, node, panda_interface_names::panda_pose_state_topic_name,
        10s, panda_interface_names::CONTROLLER_PUBLISHER_QOS());
    // initial_pose.pose = goal->initial_pose;

    // Initial orientation
    Eigen::Quaterniond initial_quat{
        initial_pose.pose.orientation.w, initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y, initial_pose.pose.orientation.z};
    initial_quat.normalize();
    Eigen::Matrix3d initial_rot{initial_quat};

    // Desired orientation
    Eigen::Quaterniond desired_quat{
        goal->desired_pose.orientation.w, goal->desired_pose.orientation.x,
        goal->desired_pose.orientation.y, goal->desired_pose.orientation.z};
    desired_quat.normalize();
    Eigen::Quaterniond relative_quat = initial_quat.inverse() * desired_quat;
    Eigen::AngleAxisd angle_axis_relative = Eigen::AngleAxisd{relative_quat};
    double theta_f = angle_axis_relative.angle();
    Eigen::Vector3d axis = angle_axis_relative.axis();

    auto feedback = std::make_shared<CartTraj::Feedback>();
    auto result = std::make_shared<CartTraj::Result>();

    rclcpp::Time t0 = this->get_clock()->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration =
        rclcpp::Duration::from_seconds(goal->total_time);
    geometry_msgs::msg::Pose cmd_pose;
    geometry_msgs::msg::Twist cmd_twist;
    geometry_msgs::msg::Accel cmd_accel;

    RCLCPP_DEBUG(this->get_logger(), "Entering while");
    while (rclcpp::ok() && t < traj_duration) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<CartTraj::Result>();
        result->completed = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      t = this->get_clock()->now() - t0;
      RCLCPP_DEBUG(this->get_logger(), "Time now %f", t.seconds());

      RCLCPP_DEBUG(this->get_logger(), "Uploading next pose");

      // Calculating position
      cmd_pose.position.x =
          qintic(initial_pose.pose.position.x, goal->desired_pose.position.x,
                 t.seconds(), goal->total_time);
      cmd_pose.position.y =
          qintic(initial_pose.pose.position.y, goal->desired_pose.position.y,
                 t.seconds(), goal->total_time);
      cmd_pose.position.z =
          qintic(initial_pose.pose.position.z, goal->desired_pose.position.z,
                 t.seconds(), goal->total_time);

      cmd_twist.linear.x = qintic_velocity(initial_pose.pose.position.x,
                                           goal->desired_pose.position.x,
                                           t.seconds(), goal->total_time);
      cmd_twist.linear.y = qintic_velocity(initial_pose.pose.position.y,
                                           goal->desired_pose.position.y,
                                           t.seconds(), goal->total_time);
      cmd_twist.linear.z = qintic_velocity(initial_pose.pose.position.z,
                                           goal->desired_pose.position.z,
                                           t.seconds(), goal->total_time);

      cmd_accel.linear.x = qintic_accel(initial_pose.pose.position.x,
                                        goal->desired_pose.position.x,
                                        t.seconds(), goal->total_time);
      cmd_accel.linear.y = qintic_accel(initial_pose.pose.position.y,
                                        goal->desired_pose.position.y,
                                        t.seconds(), goal->total_time);
      cmd_accel.linear.z = qintic_accel(initial_pose.pose.position.z,
                                        goal->desired_pose.position.z,
                                        t.seconds(), goal->total_time);

      // Calculating orientation
      double theta = qintic(0, theta_f, t.seconds(), goal->total_time);
      double theta_dot =
          qintic_velocity(0, theta_f, t.seconds(), goal->total_time);
      double theta_ddot =
          qintic_accel(0, theta_f, t.seconds(), goal->total_time);

      Eigen::Vector3d epsilon = axis * sin(theta / 2);
      Eigen::Quaterniond current_quat{cos(theta / 2), epsilon[0], epsilon[1],
                                      epsilon[2]};
      current_quat.normalize();
      // Orientation
      current_quat = initial_quat * current_quat;
      cmd_pose.orientation.w = current_quat.w();
      cmd_pose.orientation.x = current_quat.x();
      cmd_pose.orientation.y = current_quat.y();
      cmd_pose.orientation.z = current_quat.z();

      // Angular velocity
      Eigen::Vector3d angular_vel =
          current_quat.toRotationMatrix() * axis * theta_dot;
      // Eigen::Vector3d angular_vel = initial_rot * axis * theta_dot;
      cmd_twist.angular.x = angular_vel.x();
      cmd_twist.angular.y = angular_vel.y();
      cmd_twist.angular.z = angular_vel.z();

      // Angular acceleration
      Eigen::Vector3d angular_accel =
          current_quat.toRotationMatrix() * axis * theta_ddot;
      // Eigen::Vector3d angular_accel = initial_rot * axis * theta_ddot;
      cmd_accel.angular.x = angular_accel.x();
      cmd_accel.angular.y = angular_accel.y();
      cmd_accel.angular.z = angular_accel.z();

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Assigning time left");
      feedback->time_left = (traj_duration - t).seconds();

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Publishing feedback");
      goal_handle->publish_feedback(feedback);

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Publish command");


      cmd_cartesian.pose = cmd_pose;
      cmd_cartesian.twist = cmd_twist;
      cmd_cartesian.accel = cmd_accel;
      cartesian_cmd_pub->try_publish(cmd_cartesian);

      // Sleep
      //
      RCLCPP_DEBUG_ONCE(this->get_logger(), "Sleep");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok() && goal_handle->is_active()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
