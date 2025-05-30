#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_wait_set_mask.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>
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

class JointTrajectory : public rclcpp::Node {

public:
  JointTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node("joint_trajectory", opt) {

    auto save_joints_state = [this](const JointState::SharedPtr msg) {
      this->joint_state = msg;
    };

    joint_state_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, save_joints_state);

    cmd_pos_pub = this->create_publisher<panda_interfaces::msg::JointsPos>(
        panda_interface_names::panda_pos_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const TrajMove::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Joint desired config is: "
                             << goal->desired_joint_pos.data() << " in "
                             << goal->total_time);
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
  rclcpp::Publisher<panda_interfaces::msg::JointsPos>::SharedPtr cmd_pos_pub;
  rclcpp_action::Server<TrajMove>::SharedPtr action_traj_server;
  JointState::SharedPtr joint_state;

  void execute(const std::shared_ptr<GoalHandleTrajMove> goal_handle) {
    rclcpp::Rate loop_rate(100.0);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrajMove::Feedback>();
    auto result = std::make_shared<TrajMove::Result>();

    // Get joint configuration now
    RCLCPP_INFO(this->get_logger(), "Waiting for current joints config");
    JointState q0;
    joint_state = nullptr;
    while (rclcpp::ok() && !joint_state) {
      std::this_thread::sleep_for(500ns);
      RCLCPP_INFO(this->get_logger(), "Still waiting");
    }
    q0 = *joint_state;

    rclcpp::Time t0 = this->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration = rclcpp::Duration(goal->total_time, 0);

    using panda_interfaces::msg::JointsPos;
    while (rclcpp::ok() && t < traj_duration) {
      t = this->now() - t0;
      // Get next JointState
      JointsPos cmd;
      // cmd.position.resize(panda_interface_names::panda_joint_names.size());

      for (size_t i = 0; i < 7; i++) {
        // cmd.position[i] = qintic(q0.position[i], goal->desired_joint_pos[i],
        //                          t.seconds(), goal->total_time);
        cmd.joint_values[i] = qintic(q0.position[i], goal->desired_joint_pos[i],
                                 t.seconds(), goal->total_time);
      }

      // cmd.header.stamp = this->now();
      // cmd.name = panda_interface_names::panda_joint_names;

      feedback->time_left = (traj_duration - t).seconds();

      goal_handle->publish_feedback(feedback);

      cmd_pos_pub->publish(cmd);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
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
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
