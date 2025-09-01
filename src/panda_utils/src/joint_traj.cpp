#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_wait_set_mask.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

using TrajMove = panda_interfaces::action::JointTraj;
using GoalHandleTrajMove = rclcpp_action::ServerGoalHandle<TrajMove>;
using sensor_msgs::msg::JointState;
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

class JointTrajectory : public rclcpp::Node {

public:
  JointTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node("joint_trajectory", opt) {

    auto save_joints_state = [this](const JointState::SharedPtr msg) {
      this->joint_state = msg;
    };

    joint_state_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS(), save_joints_state);

    cmd_pos_pub = std::make_shared<realtime_tools::RealtimePublisher<
        panda_interfaces::msg::JointsCommand>>(
        this->create_publisher<panda_interfaces::msg::JointsCommand>(
            panda_interface_names::panda_joint_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const TrajMove::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Joint desired config is: ["
                             << goal->desired_joint_cmd.positions[0] << ", "
                             << goal->desired_joint_cmd.positions[1] << ", "
                             << goal->desired_joint_cmd.positions[2] << ", "
                             << goal->desired_joint_cmd.positions[3] << ", "
                             << goal->desired_joint_cmd.positions[4] << ", "
                             << goal->desired_joint_cmd.positions[5] << ", "
                             << goal->desired_joint_cmd.positions[6]
                             << ", "
                                "]"
                             << " in " << goal->total_time);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajMove>>
                   goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          TrajMove::Result::SharedPtr result =
              std::make_shared<TrajMove::Result>();
          result->completed = false;
          goal_handle->canceled(result);
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleTrajMove> goal_handle) {
          using namespace std::placeholders;
          std::thread{std::bind(&JointTrajectory::execute, this, _1),
                      goal_handle}
              .detach();
        };

    this->action_traj_server = rclcpp_action::create_server<TrajMove>(
        this, panda_interface_names::panda_traj_move_action_name, handle_goal,
        handle_cancel, handle_accepted);
  }

private:
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub;
  realtime_tools::RealtimePublisherSharedPtr<
      panda_interfaces::msg::JointsCommand>
      cmd_pos_pub;
  rclcpp_action::Server<TrajMove>::SharedPtr action_traj_server;
  JointState::SharedPtr joint_state;

  void execute(const std::shared_ptr<GoalHandleTrajMove> goal_handle) {

    if (realtime_tools::has_realtime_kernel() &&
        !this->get_parameter("use_sim_time").as_bool()) {
      if (!realtime_tools::configure_sched_fifo(98)) {
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

    rclcpp::Rate loop_rate(1000.0);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrajMove::Feedback>();
    auto result = std::make_shared<TrajMove::Result>();

    // Get joint configuration now
    RCLCPP_INFO(this->get_logger(), "Waiting for current joints config");
    JointState q0;
    joint_state = nullptr;
    while (rclcpp::ok() && !joint_state) {
      std::this_thread::sleep_for(500ns);
      RCLCPP_INFO(this->get_logger(), "Still waiting current joint state");
    }
    q0 = *joint_state;

    rclcpp::Time t0 = this->get_clock()->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration = rclcpp::Duration(goal->total_time, 0);

    while (rclcpp::ok() && t < traj_duration) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<TrajMove::Result>();
        result->completed = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      t = this->get_clock()->now() - t0;
      // Get next JointState
      panda_interfaces::msg::JointsCommand cmd;

      for (size_t i = 0; i < 7; i++) {

        cmd.positions[i] =
            qintic(q0.position[i], goal->desired_joint_cmd.positions[i],
                   t.seconds(), goal->total_time);

        cmd.velocities[i] = qintic_velocity(
            q0.position[i], goal->desired_joint_cmd.positions[i], t.seconds(),
            goal->total_time);

        cmd.accelerations[i] =
            qintic_accel(q0.position[i], goal->desired_joint_cmd.positions[i],
                         t.seconds(), goal->total_time);
      }

      cmd.header.stamp = this->now();

      feedback->time_left = (traj_duration - t).seconds();

      goal_handle->publish_feedback(feedback);

      cmd_pos_pub->tryPublish(cmd);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok() && goal_handle->is_active()) {
      result->completed = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      result->completed = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JointTrajectory>();

  if (!realtime_tools::has_realtime_kernel()) {
    RCLCPP_ERROR(node->get_logger(), "No real time kernel");
  }
  if (!node->get_parameter("use_sim_time").as_bool()) {

    if (!realtime_tools::configure_sched_fifo(98)) {
      RCLCPP_ERROR(node->get_logger(),
                   "Couldn't configure real time priority for current node");
    } else {
      RCLCPP_INFO(node->get_logger(), "Set real time priority");
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Simulation: realtime not requested");
  }

  RCLCPP_INFO(node->get_logger(), "Node spinned");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
