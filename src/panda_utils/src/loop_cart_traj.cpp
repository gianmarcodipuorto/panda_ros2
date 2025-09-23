#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_utils/constants.hpp"
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

using LoopCartTraj = panda_interfaces::action::LoopCartTraj;
using GoalHandleLoopMove = rclcpp_action::ServerGoalHandle<LoopCartTraj>;
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

class LoopCartTrajectory : public rclcpp::Node {

public:
  LoopCartTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node(panda_interface_names::loop_cart_traj_node_name, opt) {

    this->declare_parameter<double>("loop_rate_freq", 1000.0);

    loop_rate_freq = this->get_parameter("loop_rate_freq").as_double();

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const LoopCartTraj::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial pose: Position: ["
                             << goal->desired_poses[0].position.x << ", "
                             << goal->desired_poses[0].position.y << ", "
                             << goal->desired_poses[0].position.z
                             << "] Orientation(w, x, y, z): ["
                             << goal->desired_poses[0].orientation.w << ", "
                             << goal->desired_poses[0].orientation.x << ", "
                             << goal->desired_poses[0].orientation.y << ", "
                             << goal->desired_poses[0].orientation.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(), "In " << goal->total_time << "s");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<LoopCartTraj>>
                goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          // LoopCartTraj::Result::SharedPtr result =
          //     std::make_shared<LoopCartTraj::Result>();
          // result->completed = false;
          // goal_handle->canceled(result);
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleLoopMove> goal_handle) {
          using namespace std::placeholders;

          std::thread{std::bind(&LoopCartTrajectory::execute, this, _1),
                      goal_handle}
              .detach();
        };

    this->action_traj_server = rclcpp_action::create_server<LoopCartTraj>(
        this, panda_interface_names::panda_cart_loop_action_name, handle_goal,
        handle_cancel, handle_accepted);

    // cmd_pose_pub = std::make_shared<
    //     realtime_tools::RealtimePublisher<geometry_msgs::msg::Pose>>(
    //     this->create_publisher<geometry_msgs::msg::Pose>(
    //         panda_interface_names::panda_pose_cmd_topic_name,
    //         panda_interface_names::DEFAULT_TOPIC_QOS()));
    //
    // cmd_twist_pub = std::make_shared<
    //     realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
    //     this->create_publisher<geometry_msgs::msg::Twist>(
    //         panda_interface_names::panda_twist_cmd_topic_name,
    //         panda_interface_names::DEFAULT_TOPIC_QOS()));
    //
    // cmd_accel_pub = std::make_shared<
    //     realtime_tools::RealtimePublisher<geometry_msgs::msg::Accel>>(
    //     this->create_publisher<geometry_msgs::msg::Accel>(
    //         panda_interface_names::panda_accel_cmd_topic_name,
    //         panda_interface_names::DEFAULT_TOPIC_QOS()));
    cartesian_cmd_pub = std::make_shared<realtime_tools::RealtimePublisher<
        panda_interfaces::msg::CartesianCommand>>(
        this->create_publisher<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd",
            panda_interface_names::DEFAULT_TOPIC_QOS()));
  }

private:
  // realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Pose>
  //     cmd_pose_pub;
  // realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Twist>
  //     cmd_twist_pub;
  // realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Accel>
  //     cmd_accel_pub;

  realtime_tools::RealtimePublisherSharedPtr<
      panda_interfaces::msg::CartesianCommand>
      cartesian_cmd_pub;

  rclcpp_action::Server<LoopCartTraj>::SharedPtr action_traj_server;

  double loop_rate_freq{};

  void execute(const std::shared_ptr<GoalHandleLoopMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    if (realtime_tools::has_realtime_kernel() &&
        !this->get_parameter("use_sim_time").as_bool()) {
      if (!realtime_tools::configure_sched_fifo(95)) {
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

    auto node = std::make_shared<rclcpp::Node>("wait_pose");
    geometry_msgs::msg::PoseStamped initial_pose;
    rclcpp::wait_for_message<geometry_msgs::msg::PoseStamped>(
        initial_pose, node, panda_interface_names::panda_pose_state_topic_name,
        10s, panda_interface_names::CONTROLLER_PUBLISHER_QOS());

    size_t index = 0;
    int num_poses = goal->desired_poses.size();
    geometry_msgs::msg::Pose first_des_pose = goal->desired_poses[0];

    Eigen::Quaterniond initial_quat{
        initial_pose.pose.orientation.w, initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y, initial_pose.pose.orientation.z};

    Eigen::Quaterniond first_des_orient{
        first_des_pose.orientation.w, first_des_pose.orientation.x,
        first_des_pose.orientation.y, first_des_pose.orientation.z};

    Eigen::Vector3d orient_err =
        (initial_quat.inverse() * first_des_orient).normalized().vec();
    double pos_error, pos_orientation;
    pos_error = std::sqrt(
        std::pow(initial_pose.pose.position.x - first_des_pose.position.x, 2) +
        std::pow(initial_pose.pose.position.x - first_des_pose.position.x, 2) +
        std::pow(initial_pose.pose.position.x - first_des_pose.position.x, 2));
    pos_orientation = std::sqrt(std::pow(orient_err.x() - orient_err.x(), 2) +
                                std::pow(orient_err.x() - orient_err.x(), 2) +
                                std::pow(orient_err.x() - orient_err.x(), 2));

    if (pos_error > 1e-1 || pos_orientation > 1e-3) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Robot not in first desired position, aborting: "
                              << pos_error << ", " << pos_orientation);
      LoopCartTraj::Result res;
      res.completed = false;
      goal_handle->abort(std::make_shared<LoopCartTraj::Result>(res));
      return;
    }

    geometry_msgs::msg::Pose desired_pose;
    while (rclcpp::ok()) {
      initial_pose.pose = goal->desired_poses[index];
      desired_pose =
          goal->desired_poses[(index + 1) % goal->desired_poses.size()];

      // Initial orientation
      Eigen::Quaterniond initial_quat{
          initial_pose.pose.orientation.w, initial_pose.pose.orientation.x,
          initial_pose.pose.orientation.y, initial_pose.pose.orientation.z};
      initial_quat.normalize();
      Eigen::Matrix3d initial_rot{initial_quat};

      // Desired orientation
      Eigen::Quaterniond desired_quat{
          desired_pose.orientation.w, desired_pose.orientation.x,
          desired_pose.orientation.y, desired_pose.orientation.z};
      desired_quat.normalize();
      Eigen::Quaterniond relative_quat = initial_quat.inverse() * desired_quat;
      Eigen::AngleAxisd angle_axis_relative = Eigen::AngleAxisd{relative_quat};
      double theta_f = angle_axis_relative.angle();
      Eigen::Vector3d axis = angle_axis_relative.axis();

      auto feedback = std::make_shared<LoopCartTraj::Feedback>();
      auto result = std::make_shared<LoopCartTraj::Result>();

      rclcpp::Time t0 = this->get_clock()->now();
      rclcpp::Duration t = rclcpp::Duration(0, 0);
      double segment_total_time = goal->total_time / goal->desired_poses.size();
      rclcpp::Duration traj_duration =
          rclcpp::Duration::from_seconds(segment_total_time);
      panda_interfaces::msg::CartesianCommand cmd_cartesian;
      geometry_msgs::msg::Pose cmd_pose;
      geometry_msgs::msg::Twist cmd_twist;
      geometry_msgs::msg::Accel cmd_accel;

      RCLCPP_DEBUG(this->get_logger(), "Entering while");
      while (rclcpp::ok() && t < traj_duration) {
        if (goal_handle->is_canceling()) {
          auto result = std::make_shared<LoopCartTraj::Result>();
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
            qintic(initial_pose.pose.position.x, desired_pose.position.x,
                   t.seconds(), segment_total_time);
        cmd_pose.position.y =
            qintic(initial_pose.pose.position.y, desired_pose.position.y,
                   t.seconds(), segment_total_time);
        cmd_pose.position.z =
            qintic(initial_pose.pose.position.z, desired_pose.position.z,
                   t.seconds(), segment_total_time);

        cmd_twist.linear.x = qintic_velocity(initial_pose.pose.position.x,
                                             desired_pose.position.x,
                                             t.seconds(), segment_total_time);
        cmd_twist.linear.y = qintic_velocity(initial_pose.pose.position.y,
                                             desired_pose.position.y,
                                             t.seconds(), segment_total_time);
        cmd_twist.linear.z = qintic_velocity(initial_pose.pose.position.z,
                                             desired_pose.position.z,
                                             t.seconds(), segment_total_time);

        cmd_accel.linear.x =
            qintic_accel(initial_pose.pose.position.x, desired_pose.position.x,
                         t.seconds(), segment_total_time);
        cmd_accel.linear.y =
            qintic_accel(initial_pose.pose.position.y, desired_pose.position.y,
                         t.seconds(), segment_total_time);
        cmd_accel.linear.z =
            qintic_accel(initial_pose.pose.position.z, desired_pose.position.z,
                         t.seconds(), segment_total_time);

        // Calculating orientation
        double theta = qintic(0, theta_f, t.seconds(), segment_total_time);
        double theta_dot =
            qintic_velocity(0, theta_f, t.seconds(), segment_total_time);
        double theta_ddot =
            qintic_accel(0, theta_f, t.seconds(), segment_total_time);

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

        // cmd_accel.angular.x = 0.0;
        // cmd_accel.angular.y = 0.0;
        // cmd_accel.angular.z = 0.0;

        RCLCPP_DEBUG_ONCE(this->get_logger(), "Assigning time left");
        feedback->time_left = (traj_duration - t).seconds();

        RCLCPP_DEBUG_ONCE(this->get_logger(), "Publishing feedback");
        goal_handle->publish_feedback(feedback);

        RCLCPP_DEBUG_ONCE(this->get_logger(), "Publish command");
        cmd_cartesian.pose = cmd_pose;
        cmd_cartesian.twist = cmd_twist;
        cmd_cartesian.accel = cmd_accel;
        cartesian_cmd_pub->try_publish(cmd_cartesian);

        // cmd_pose_pub->tryPublish(cmd_pose);
        // cmd_twist_pub->tryPublish(cmd_twist);
        // cmd_accel_pub->tryPublish(cmd_accel);

        // Sleep
        //
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Sleep");
        loop_rate.sleep();
      }

      if (index + 1 >= goal->desired_poses.size()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Terminated figure");
        index = 0;
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Terminated segment " << index + 1);
        index++;
      }
    }

    LoopCartTraj::Result res;
    res.completed = true;
    // Check if goal is done
    if (rclcpp::ok() && goal_handle->is_active()) {
      goal_handle->succeed(std::make_shared<LoopCartTraj::Result>(res));
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoopCartTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
