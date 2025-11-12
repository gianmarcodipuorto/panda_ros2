#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "panda_interfaces/action/stop_traj.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <memory>
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
#include "panda_utils/utils_func.hpp"

using StopTraj = panda_interfaces::action::StopTraj;
using GoalHandleStopTraj = rclcpp_action::ServerGoalHandle<StopTraj>;
using namespace std::chrono_literals;

class StopTrajectory : public rclcpp::Node {

public:
  StopTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node(panda_interface_names::exponential_stop_traj_node_name, opt) {

    this->declare_parameter<double>("loop_rate_freq", 1000.0);

    loop_rate_freq = this->get_parameter("loop_rate_freq").as_double();

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const StopTraj::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      initial_pose = last_cartesian_cmd.pose;
      initial_velocity = last_cartesian_cmd.twist;
      initial_acceleration = last_cartesian_cmd.accel;
      (void)uuid;
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Desired pose: Position: ["
              << initial_pose.position.x << ", " << initial_pose.position.y
              << ", " << initial_pose.position.z
              << "] Orientation(w, x, y, z): [" << initial_pose.orientation.w
              << ", " << initial_pose.orientation.x << ", "
              << initial_pose.orientation.y << ", "
              << initial_pose.orientation.z << "]");

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial twist: Linear: ["
                             << initial_velocity.linear.x << ", "
                             << initial_velocity.linear.y << ", "
                             << initial_velocity.linear.z << "] Angular: ["
                             << initial_velocity.angular.x << ", "
                             << initial_velocity.angular.y << ", "
                             << initial_velocity.angular.z << "]");

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial accelaration: Linear: ["
                             << initial_acceleration.linear.x << ", "
                             << initial_acceleration.linear.y << ", "
                             << initial_acceleration.linear.z << "] Angular: ["
                             << initial_acceleration.angular.x << ", "
                             << initial_acceleration.angular.y << ", "
                             << initial_acceleration.angular.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(), "In " << goal->total_time << "s");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<StopTraj>>
                   goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleStopTraj> goal_handle) {
          using namespace std::placeholders;

          std::thread{std::bind(&StopTrajectory::execute, this, _1),
                      goal_handle}
              .detach();
        };

    this->action_traj_server = rclcpp_action::create_server<StopTraj>(
        this, panda_interface_names::panda_exponential_stop_action_name,
        handle_goal, handle_cancel, handle_accepted);

    cmd_pose_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Pose>>(
        this->create_publisher<geometry_msgs::msg::Pose>(
            panda_interface_names::panda_pose_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    cmd_twist_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
        this->create_publisher<geometry_msgs::msg::Twist>(
            panda_interface_names::panda_twist_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    cmd_accel_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Accel>>(
        this->create_publisher<geometry_msgs::msg::Accel>(
            panda_interface_names::panda_accel_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    auto last_cmd_cb =
        [this](const panda_interfaces::msg::CartesianCommand msg) {
          last_cartesian_cmd = msg;
        };

    cmd_cartesian_sub =
        this->create_subscription<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd", panda_interface_names::DEFAULT_TOPIC_QOS(),
            last_cmd_cb);

    cartesian_cmd_pub = std::make_shared<realtime_tools::RealtimePublisher<
        panda_interfaces::msg::CartesianCommand>>(
        this->create_publisher<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd",
            panda_interface_names::DEFAULT_TOPIC_QOS()));
  }

private:
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Pose>
      cmd_pose_pub;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Twist>
      cmd_twist_pub;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Accel>
      cmd_accel_pub;
  realtime_tools::RealtimePublisherSharedPtr<
      panda_interfaces::msg::CartesianCommand>
      cartesian_cmd_pub;

  rclcpp::Subscription<panda_interfaces::msg::CartesianCommand>::SharedPtr
      cmd_cartesian_sub;

  panda_interfaces::msg::CartesianCommand last_cartesian_cmd;

  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Twist initial_velocity;
  geometry_msgs::msg::Accel initial_acceleration;

  rclcpp_action::Server<StopTraj>::SharedPtr action_traj_server;

  double loop_rate_freq{};

  void execute(const std::shared_ptr<GoalHandleStopTraj> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

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
    rclcpp::Rate loop_rate(loop_rate_freq, this->get_clock());

    RCLCPP_DEBUG(this->get_logger(), "Getting goal");
    const auto goal = goal_handle->get_goal();

    // Initial orientation
    Eigen::Quaterniond initial_quat{
        initial_pose.orientation.w, initial_pose.orientation.x,
        initial_pose.orientation.y, initial_pose.orientation.z};
    initial_quat.normalize();
    Eigen::Quaterniond current_quat = initial_quat;
    Eigen::AngleAxisd axis_angle = Eigen::AngleAxisd{initial_quat};
    Eigen::Vector3d axis = axis_angle.axis();

    auto feedback = std::make_shared<StopTraj::Feedback>();
    auto result = std::make_shared<StopTraj::Result>();

    rclcpp::Time t0 = this->get_clock()->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration =
        rclcpp::Duration::from_seconds(goal->total_time);
    geometry_msgs::msg::Pose cmd_pose;
    geometry_msgs::msg::Twist cmd_twist;
    geometry_msgs::msg::Accel cmd_accel;
    panda_interfaces::msg::CartesianCommand cmd_cartesian;

    // Exponential decay laws
    std::map<int, decay_laws> translation_laws;
    std::map<int, decay_laws> orientation_laws;

    translation_laws.emplace(0, decay_laws(goal->total_time,
                                           initial_pose.position.x,
                                           initial_velocity.linear.x,
                                           initial_acceleration.linear.x));
    translation_laws.emplace(1, decay_laws(goal->total_time,
                                           initial_pose.position.y,
                                           initial_velocity.linear.y,
                                           initial_acceleration.linear.y));
    translation_laws.emplace(2, decay_laws(goal->total_time,
                                           initial_pose.position.z,
                                           initial_velocity.linear.z,
                                           initial_acceleration.linear.z));

    orientation_laws.emplace(0, decay_laws(goal->total_time, 0.0,
                                           initial_velocity.angular.x,
                                           initial_acceleration.angular.x));
    orientation_laws.emplace(1, decay_laws(goal->total_time, 0.0,
                                           initial_velocity.angular.y,
                                           initial_acceleration.angular.y));
    orientation_laws.emplace(2, decay_laws(goal->total_time, 0.0,
                                           initial_velocity.angular.z,
                                           initial_acceleration.angular.z));

    RCLCPP_DEBUG(this->get_logger(), "Entering while");
    while (rclcpp::ok() && t < traj_duration) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<StopTraj::Result>();
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
          translation_laws.at(0).exponential_decay_position(t.seconds());
      cmd_pose.position.y =
          translation_laws.at(1).exponential_decay_position(t.seconds());
      cmd_pose.position.z =
          translation_laws.at(2).exponential_decay_position(t.seconds());

      cmd_twist.linear.x =
          translation_laws.at(0).exponential_decay_velocity(t.seconds());
      cmd_twist.linear.y =
          translation_laws.at(1).exponential_decay_velocity(t.seconds());
      cmd_twist.linear.z =
          translation_laws.at(2).exponential_decay_velocity(t.seconds());

      cmd_accel.linear.x =
          translation_laws.at(0).exponential_decay_acceleration(t.seconds());
      cmd_accel.linear.y =
          translation_laws.at(1).exponential_decay_acceleration(t.seconds());
      cmd_accel.linear.z =
          translation_laws.at(2).exponential_decay_acceleration(t.seconds());

      // Orientation
      // For the orientation we apply the decay laws to the twist and the accel
      // and we calculate a theta angle as the integral of the twist: based on
      // this theta we calculate a delta quaternion to apply to the current
      // quaternion
      cmd_twist.angular.x =
          orientation_laws.at(0).exponential_decay_velocity(t.seconds());
      cmd_twist.angular.y =
          orientation_laws.at(1).exponential_decay_velocity(t.seconds());
      cmd_twist.angular.z =
          orientation_laws.at(2).exponential_decay_velocity(t.seconds());

      Eigen::Quaterniond delta_quat{1.0,
                                    0.5 * cmd_twist.angular.x / loop_rate_freq,
                                    0.5 * cmd_twist.angular.y / loop_rate_freq,
                                    0.5 * cmd_twist.angular.z / loop_rate_freq};
      delta_quat.normalize();

      current_quat = current_quat * delta_quat;
      current_quat.normalize();

      cmd_pose.orientation.x = current_quat.x();
      cmd_pose.orientation.y = current_quat.y();
      cmd_pose.orientation.z = current_quat.z();
      cmd_pose.orientation.w = current_quat.w();

      cmd_accel.angular.x =
          orientation_laws.at(0).exponential_decay_acceleration(t.seconds());
      cmd_accel.angular.y =
          orientation_laws.at(1).exponential_decay_acceleration(t.seconds());
      cmd_accel.angular.z =
          orientation_laws.at(2).exponential_decay_acceleration(t.seconds());

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
  auto node = std::make_shared<StopTrajectory>();

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

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
