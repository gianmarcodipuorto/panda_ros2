#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_processing/constants.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_interfaces/action/stop_traj.hpp"
#include "panda_interfaces/msg/human_detected.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <Eigen/src/Core/PartialReduxEvaluator.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <panda_interfaces/msg/human_detected.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shared_mutex>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using geometry_msgs::msg::Accel;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using panda_interfaces::action::CartTraj;
using panda_interfaces::action::LoopCartTraj;
using panda_interfaces::action::StopTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;
template <typename ActionT>
using GoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;
template <typename ActionT>
using OptionalGoalHandle = typename std::optional<
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>;

enum class SceneState {
  // The robot has to be configured
  no_state,
  // The robot is doing whichever task has to do
  task,
  // The human enters the robot area and the robot has to enter compliance
  // mode
  transition_human,
  // The robot is in compliance mode: it can be freely moved by the human
  compliance,
  // The human leaves the robot area, allowing the robot to resume task
  transition_leave_human,

};

void publish_state(
    const rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pub,
    const SceneState &state) {

  std_msgs::msg::ColorRGBA color;

  switch (state) {

  case SceneState::no_state: {
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  }
  case SceneState::task: {
    color.r = 0.0;
    color.g = 255.0;
    color.b = 0.0;
    break;
  }
  case SceneState::transition_human: {
    color.r = 255.0;
    color.g = 255.0;
    color.b = 0.0;
    break;
  }
  case SceneState::compliance: {
    color.r = 255.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  }
  case SceneState::transition_leave_human: {
    color.r = 0.0;
    color.g = 255.0;
    color.b = 255.0;
    break;
  } break;
  }

  pub->publish(color);
}

struct human_presence {
  const double MAX_TIME = 1.0;
  std::shared_mutex mut;
  bool human_present = false;
  rclcpp::Duration time_present = rclcpp::Duration::from_seconds(0.0);
  std::optional<std::string> contact_joint = std::optional<std::string>{};

  void normalize_time() {
    if (time_present.seconds() > MAX_TIME) {
      time_present = rclcpp::Duration::from_seconds(MAX_TIME);
    } else if (time_present.seconds() < 0.0) {
      time_present = rclcpp::Duration::from_seconds(0.0);
    }
  }
};

void fill_pose_orientation(geometry_msgs::msg::Pose &pose,
                           const Eigen::Quaterniond &orientation) {

  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
}

void fill_pose_position(geometry_msgs::msg::Pose &pose, const double &x,
                        const double &y, const double &z) {

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
}

panda_interfaces::action::LoopCartTraj_Goal
generate_triangle_task(double x, double y, double z, double h, double l,
                       Eigen::Quaterniond orientation, double total_time) {

  using geometry_msgs::msg::Pose;
  panda_interfaces::action::LoopCartTraj_Goal goal;
  Pose initial;
  Pose right_corner;
  Pose left_corner;

  fill_pose_orientation(initial, orientation);
  fill_pose_orientation(right_corner, orientation);
  fill_pose_orientation(left_corner, orientation);

  fill_pose_position(initial, x, y, z);
  fill_pose_position(right_corner, x, y + l / 2, z - h);
  fill_pose_position(left_corner, x, y - l / 2, z - h);

  goal.desired_poses = std::vector{initial, right_corner, left_corner};
  goal.total_time = total_time;
  return goal;
}

void go_to_pose(
    const rclcpp::Node &node, rclcpp_action::Client<CartTraj> &client,
    const rclcpp_action::Client<CartTraj>::SendGoalOptions &cart_traj_options,
    const panda_interfaces::action::CartTraj_Goal &pose_goal) {}

struct LastRobotState {
  geometry_msgs::msg::Pose pose;
  std::array<double, 7> joint_config;
};

std::array<double, 7> home_joint_config;
Pose home_pose;
Pose initial_task_pose;
double triangle_height; // h
double triangle_base;   // l
double total_task_time;
const std::string robot_base_frame_name{"fr3_link0"};
const std::string robot_end_affector_frame{"fr3_joint7"};
const std::string world_frame{"world"};
const rclcpp::Duration max_tf_age = rclcpp::Duration::from_seconds(1.0 / 30.0);
const double robot_radius_area = 1.0;       // meters
const double min_distance_from_joint = 0.1; // meters

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto main_node = std::make_shared<rclcpp::Node>("task_node");
  bool use_sim_time;

  switch (argc) {
  case 5: {
    use_sim_time = (atoi(argv[1]) == 0) ? false : true;
    RCLCPP_INFO_STREAM(main_node->get_logger(), "x: " << argv[2]
                                                      << ", y: " << argv[3]
                                                      << ", z: " << argv[4]);
    home_pose.position.x = atof(argv[2]);
    home_pose.position.y = atof(argv[3]);
    home_pose.position.z = atof(argv[4]);
    break;
  }
  default: {
    RCLCPP_ERROR(main_node->get_logger(),
                 "Usage: <use_sim_time> <x> <y> <z> home pose position");
    rclcpp::shutdown();
    return 0;
  }
  }
  auto use_sim_time_param = rclcpp::Parameter("use_sim_time", use_sim_time);

  auto sub_node = std::make_shared<rclcpp::Node>("task_sub_node");
  sub_node->set_parameter(use_sim_time_param);
  main_node->set_parameter(use_sim_time_param);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(sub_node);
  executor.add_node(main_node);

  // Choosing home pose

  home_pose.orientation.w = 0.0;
  home_pose.orientation.x = 1.0;
  home_pose.orientation.y = 0.0;
  home_pose.orientation.z = 0.0;

  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose desired_pose;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  SceneState state{SceneState::no_state};
  human_presence presence_state;
  std::atomic<bool> threads_run{false};
  LastRobotState last_state;
  JointState joint_states;
  PoseStamped pose_state;
  // The goal handles for the 2 possible actions of robot, these needs to be
  // stopped in any error or shutdown state
  OptionalGoalHandle<CartTraj> cartesian_traj_handle;
  OptionalGoalHandle<LoopCartTraj> loop_cartesian_traj_handle;
  OptionalGoalHandle<StopTraj> stop_traj_handle;

  // Goal options
  GoalOptions<CartTraj> cart_traj_options;
  GoalOptions<LoopCartTraj> loop_cart_traj_options;
  GoalOptions<StopTraj> stop_traj_options;

  RCLCPP_INFO(main_node->get_logger(), "Defining Home goal");
  // Home goal definition
  panda_interfaces::action::CartTraj_Goal home_goal;
  home_goal.desired_pose = home_pose;
  home_goal.total_time = 3.0;

  // Stop trajectory goal definition
  panda_interfaces::action::StopTraj_Goal stop_traj_goal;
  stop_traj_goal.total_time = 1.0;

  RCLCPP_INFO(main_node->get_logger(), "Defining Task goal: triangle");
  // Task goal definition: triangle
  Eigen::Quaterniond triangle_orient{
      home_pose.orientation.w, home_pose.orientation.x, home_pose.orientation.y,
      home_pose.orientation.z};
  panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal =
      generate_triangle_task(home_pose.position.x, home_pose.position.y,
                             home_pose.position.z, 0.1, 0.1,
                             triangle_orient.normalized(), 4.5);

  // Topic to read for scene infos

  auto joint_states_cb = [&joint_states](const JointState msg) {
    joint_states = msg;
  };

  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub =
      sub_node->create_subscription<JointState>(
          panda_interface_names::joint_state_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS, joint_states_cb);

  auto pose_state_cb = [&pose_state](const PoseStamped msg) {
    pose_state = msg;
  };

  rclcpp::Subscription<PoseStamped>::SharedPtr pose_state_sub =
      sub_node->create_subscription<PoseStamped>(
          panda_interface_names::panda_pose_state_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS, pose_state_cb);

  tf_buffer = std::make_unique<tf2_ros::Buffer>(sub_node->get_clock());

  std::unique_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer, sub_node);

  // Compliance mode service client
  rclcpp::Client<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_client =
          main_node->create_client<panda_interfaces::srv::SetComplianceMode>(
              panda_interface_names::set_compliance_mode_service_name);
  panda_interfaces::srv::SetComplianceMode_Request compliance_request;

  // Publisher for the update of desired commands when switching to compliance
  // mode

  rclcpp::Publisher<Pose>::SharedPtr pose_cmd_pub =
      main_node->create_publisher<Pose>(
          panda_interface_names::panda_pose_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);
  rclcpp::Publisher<Twist>::SharedPtr twist_cmd_pub =
      main_node->create_publisher<Twist>(
          panda_interface_names::panda_twist_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);
  rclcpp::Publisher<Accel>::SharedPtr accel_cmd_pub =
      main_node->create_publisher<Accel>(
          panda_interface_names::panda_accel_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);

  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr state_publisher =
      main_node->create_publisher<std_msgs::msg::ColorRGBA>(
          panda_interface_names::demo_state_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);

  auto update_state = [&state_publisher, &state]() {
    publish_state(state_publisher, state);
  };

  auto send_current_pose_as_cmd = [&pose_cmd_pub, &twist_cmd_pub,
                                   &accel_cmd_pub](const Pose desired_pose) {
    Twist twist;
    Accel accel;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    accel.linear.x = 0.0;
    accel.linear.y = 0.0;
    accel.linear.z = 0.0;
    accel.angular.x = 0.0;
    accel.angular.y = 0.0;
    accel.angular.z = 0.0;

    twist_cmd_pub->publish(twist);
    accel_cmd_pub->publish(accel);
    pose_cmd_pub->publish(desired_pose);
  };

  // Action clients for trajectories (actions)
  rclcpp_action::Client<CartTraj>::SharedPtr cart_traj_action_client =
      rclcpp_action::create_client<CartTraj>(
          main_node, panda_interface_names::panda_cart_move_action_name);

  rclcpp_action::Client<LoopCartTraj>::SharedPtr loop_traj_action_client =
      rclcpp_action::create_client<LoopCartTraj>(
          main_node, panda_interface_names::panda_cart_loop_action_name);

  rclcpp_action::Client<StopTraj>::SharedPtr stop_traj_action_client =
      rclcpp_action::create_client<StopTraj>(
          main_node, panda_interface_names::panda_exponential_stop_action_name);

  // Management of action nodes for the 2 types of trajectories
  auto cancel_actions = [&cartesian_traj_handle, &loop_cartesian_traj_handle,
                         &stop_traj_handle, &stop_traj_action_client,
                         &cart_traj_action_client, &loop_traj_action_client,
                         &main_node]() {
    if (cartesian_traj_handle.has_value()) {
      cart_traj_action_client->async_cancel_goal(cartesian_traj_handle.value());
      RCLCPP_INFO(main_node->get_logger(),
                  "Requested cancel of cartesian trajectory action");
    }

    if (loop_cartesian_traj_handle.has_value()) {
      loop_traj_action_client->async_cancel_goal(
          loop_cartesian_traj_handle.value());
      RCLCPP_INFO(main_node->get_logger(),
                  "Requested cancel of loop cartesian trajectory action");
    }

    if (stop_traj_handle.has_value()) {
      stop_traj_action_client->async_cancel_goal(stop_traj_handle.value());
      RCLCPP_INFO(main_node->get_logger(),
                  "Requested cancel of exponential stop action");
    }
  };

  // Definition of SendGoalOptions struct
  {
    auto cart_result_callback =
        [&cartesian_traj_handle](
            const rclcpp_action::ClientGoalHandle<CartTraj>::WrappedResult &) {
          cartesian_traj_handle = std::nullopt;
        };
    cart_traj_options.result_callback = cart_result_callback;

    auto cart_goal_response_callback =
        [&cartesian_traj_handle](
            rclcpp_action::ClientGoalHandle<CartTraj>::SharedPtr goal_handle) {
          cartesian_traj_handle = goal_handle;
        };
    cart_traj_options.goal_response_callback = cart_goal_response_callback;

    auto loop_cart_goal_response_callback =
        [&loop_cartesian_traj_handle](
            rclcpp_action::ClientGoalHandle<LoopCartTraj>::SharedPtr
                goal_handle) { loop_cartesian_traj_handle = goal_handle; };
    loop_cart_traj_options.goal_response_callback =
        loop_cart_goal_response_callback;

    auto loop_result_callback =
        [&loop_cartesian_traj_handle](
            const rclcpp_action::ClientGoalHandle<LoopCartTraj>::WrappedResult
                &) { loop_cartesian_traj_handle = std::nullopt; };
    loop_cart_traj_options.result_callback = loop_result_callback;

    auto stop_result_callback =
        [&stop_traj_handle](
            const rclcpp_action::ClientGoalHandle<StopTraj>::WrappedResult &) {
          stop_traj_handle = std::nullopt;
        };
    stop_traj_options.result_callback = stop_result_callback;

    auto stop_goal_response_callback =
        [&stop_traj_handle](
            rclcpp_action::ClientGoalHandle<StopTraj>::SharedPtr goal_handle) {
          stop_traj_handle = goal_handle;
        };
    stop_traj_options.goal_response_callback = stop_goal_response_callback;
  }

  RCLCPP_INFO(main_node->get_logger(), "Waiting for servers...");

  RCLCPP_INFO(main_node->get_logger(), "Waiting for cartesian trajectory");
  cart_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(), "Waiting for loop cartesian trajectory");
  loop_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(),
              "Waiting for exponential stop trajectory");
  stop_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(), "Waiting for compliance mode server");
  compliance_mode_client->wait_for_service(10s);

  RCLCPP_INFO(main_node->get_logger(), "Servers UP");

  // Go to initial pose
  // Begin task: cancel when a human enters the area
  // Enter in compliance mode: exit when human leaves the area, staying in
  // current pose Go back to last task state Resume task

  std::thread info_thread([&executor]() { executor.spin(); });

  std::thread safe_keeper_thread{[cancel_actions, &main_node, &presence_state,
                                  &state, &threads_run]() {
    while (!threads_run.load()) {
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO_STREAM(main_node->get_logger(), "Started thread 'safe_keeper'");

    while (threads_run.load()) {
      std::shared_lock<std::shared_mutex> mutex(presence_state.mut);
      if (presence_state.human_present &&
          state != SceneState::transition_human &&
          state != SceneState::compliance) {
        RCLCPP_INFO_STREAM(
            main_node->get_logger(),
            "Human present in scene, cancelling actions and transitioning");
        cancel_actions();
        state = SceneState::transition_human;
      } else if (!presence_state.human_present &&
                 state == SceneState::compliance) {
        RCLCPP_INFO_STREAM(main_node->get_logger(),
                           "Human left scene, transitioning");
        state = SceneState::transition_leave_human;
      }

      std::this_thread::sleep_for(5ms);
    }
  }};

  std::thread calculate_proximity{[&tf_buffer, &sub_node, &presence_state,
                                   &threads_run] {
    // TODO: handle 2 things:
    // Check if base robot link exists
    std::map<std::string, geometry_msgs::msg::TransformStamped>
        robot_links_body_keypoints_tfs;
    std::map<std::string, geometry_msgs::msg::TransformStamped>
        robot_base_joints_tfs;

    while (!threads_run.load()) {
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO_STREAM(sub_node->get_logger(),
                       "Calculate proximity thread started");

    rclcpp::Time last_time{sub_node->now()};
    rclcpp::Time last_time_print{sub_node->now()};

    while (threads_run.load() && rclcpp::ok()) {

      bool print_every_half_sec =
          (sub_node->now() - last_time_print).seconds() > 0.5;

      if (tf_buffer->canTransform(robot_base_frame_name, world_frame,
                                  tf2::TimePointZero)) {
        // Robot base exists, now get the transforms of the body keypoints
        // wrt the base
        auto now = sub_node->now();
        // Clear the map to avoid getting false data
        robot_links_body_keypoints_tfs.clear();
        // Cycle through all the body keypoints frame: get the transform if the
        // body frame is relatively new in the tf2 system and obvoiusly exists
        for (auto keypoint_frame_name : image_constants::coco_keypoints) {
          try {
            auto keypoint_frame = tf_buffer->lookupTransform(
                keypoint_frame_name, world_frame, tf2::TimePointZero);

            if (now - keypoint_frame.header.stamp >= max_tf_age) {
              continue;
            }
          } catch (const tf2::TransformException &ex) {
            continue;
          }

          for (auto robot_frame_name :
               panda_interface_names::panda_link_names) {
            try {
              auto tf = tf_buffer->lookupTransform(
                  robot_frame_name, keypoint_frame_name, tf2::TimePointZero);
              robot_links_body_keypoints_tfs[robot_frame_name + "_to_" +
                                             keypoint_frame_name] = tf;
            } catch (const tf2::TransformException &ex) {
              // Tf does not exists
            }
          }
        }

        if (print_every_half_sec) {

          RCLCPP_INFO_STREAM(sub_node->get_logger(),
                             robot_links_body_keypoints_tfs.size()
                                 << " tfs looked");
        }

        // Now will check for proximity with the entire robot
        bool human_in_area = false;
        for (std::pair<std::string, geometry_msgs::msg::TransformStamped>
                 named_tf : robot_links_body_keypoints_tfs) {
          double dist = geom_utils::distance(named_tf.second);
          if (dist <= robot_radius_area) {
            human_in_area = true;
            break;
          }
        }
        // Increase the time the human have been seen inside the area or
        // decrease if he's not there
        {
          std::lock_guard<std::shared_mutex> mutex(presence_state.mut);
          if (human_in_area) {
            presence_state.time_present += (sub_node->now() - last_time);
          } else {
            presence_state.time_present -= (sub_node->now() - last_time);
          }
          // Normalize the time according to limit
          presence_state.normalize_time();
          if (print_every_half_sec) {

            RCLCPP_INFO_STREAM(sub_node->get_logger(),
                               "Presence time: "
                                   << presence_state.time_present.seconds()
                                   << "[s]");
          }
          if (human_in_area &&
              presence_state.time_present.seconds() >=
                  presence_state.MAX_TIME &&
              !presence_state.human_present) {
            // If human is inside by max time at least then he's surely in
            // area
            RCLCPP_INFO_STREAM(sub_node->get_logger(),
                               "The human entered the robot area");
            presence_state.human_present = true;
          } else if (!human_in_area &&
                     presence_state.time_present.seconds() == 0.0 &&
                     presence_state.human_present) {
            RCLCPP_INFO_STREAM(sub_node->get_logger(),
                               "The human left the robot area");
            presence_state.human_present = false;
          }
          // We leave the mutex property here because we need to recall
          // other tfs
        }

        // Now, if the human is in the area, we have to check if he's
        // touching a joint Don't check if every frame exists, we'll let
        // throw the exception if doesn't still exist and continue
        if (presence_state.human_present) {

          std::optional<std::string> joint_touched;

          for (auto joint_name : panda_interface_names::panda_joint_names) {
            try {
              auto tf = tf_buffer->lookupTransform(
                  image_constants::left_wrist, joint_name, tf2::TimePointZero);
              if (now - tf.header.stamp <= max_tf_age) {
                if (geom_utils::distance(tf) <= min_distance_from_joint) {
                  joint_touched = joint_name;
                  break;
                }
              }

            } catch (const tf2::TransformException &ex) {
              // Tf does not exists
            }

            try {
              auto tf = tf_buffer->lookupTransform(
                  image_constants::right_wrist, joint_name, tf2::TimePointZero);
              if (now - tf.header.stamp <= max_tf_age) {
                if (geom_utils::distance(tf) <= min_distance_from_joint) {
                  joint_touched = joint_name;
                  break;
                }
              }

            } catch (const tf2::TransformException &ex) {
              // Tf does not exists
            }
          }

          // Finally, if the joint_touched contains any value we update the
          // contact joint var
          if (joint_touched.has_value()) {
            std::shared_lock<std::shared_mutex> mutex(presence_state.mut);
            presence_state.contact_joint = joint_touched;
          }
        }
      }
      // - Proximity value as bool, depending on distance from working area
      // - Minimum distance between each joint of the robot and the human
      // wrist, if they are in scene

      last_time = sub_node->now();
      if (print_every_half_sec) {
        last_time_print = sub_node->now();
      }
      std::this_thread::sleep_for(10ms);
    }
  }};

  {
    RCLCPP_INFO(main_node->get_logger(), "Press enter to start demo");
    std::cin.ignore();
    threads_run.store(true);
    RCLCPP_INFO(main_node->get_logger(), "Threads running");
  }

  auto start = rclcpp::Time(0, 0, main_node->get_clock()->get_clock_type());

  // Resetting compliance mode if set in controller
  // Firstly, update the desired pose and other commands to the read one
  send_current_pose_as_cmd(pose_state.pose);

  compliance_request.cmd = false;
  auto compliance_future = compliance_mode_client->async_send_request(
      std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
          compliance_request));
  bool stopped = false;
  while (!stopped) {
    auto res = compliance_future.wait_for(10ms);
    if (res == std::future_status::ready) {
      RCLCPP_INFO(main_node->get_logger(), "Robot in non compliance mode");
      stopped = true;
    } else {
      RCLCPP_INFO(main_node->get_logger(), "Robot exiting compliance mode");
    }
  }

  update_state();

  while (rclcpp::ok()) {
    switch (state) {
    case SceneState::no_state: {

      update_state();

      // Ensure the robot is up and running
      // Going to pose
      RCLCPP_INFO(main_node->get_logger(), "Robot has no known state");

      if (!cartesian_traj_handle.has_value()) {

        if (!rclcpp::ok()) {
          break;
        }

        RCLCPP_INFO(main_node->get_logger(), "Sending robot home pose");
        cart_traj_action_client->async_send_goal(home_goal, cart_traj_options);
        while (!cartesian_traj_handle.has_value()) {
          RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
          if (!rclcpp::ok()) {
            break;
          }
          rclcpp::sleep_for(2s);
        }

        while (cartesian_traj_handle.has_value()) {
          RCLCPP_INFO(main_node->get_logger(),
                      "Waiting robot to reach home pose");
          if (!rclcpp::ok()) {
            break;
          }
          rclcpp::sleep_for(2s);
        }

        // Check if action has been interrupted from the human entering the area
        if (state != SceneState::no_state || !rclcpp::ok()) {
          break;
        }

        RCLCPP_INFO(main_node->get_logger(),
                    "Robot reached home pose, transitioning to task state");
        state = SceneState::task;
        break;
      }
      break;
    }
    case SceneState::task: {

      update_state();

      if (!loop_cartesian_traj_handle.has_value()) {
        loop_traj_action_client->async_send_goal(triangle_task_goal,
                                                 loop_cart_traj_options);
        while (!loop_cartesian_traj_handle.has_value()) {
          RCLCPP_INFO(main_node->get_logger(),
                      "Waiting for loop goal confirmation");
          if (!rclcpp::ok()) {
            break;
          }
          rclcpp::sleep_for(2s);
        }
      }
      if ((main_node->now() - start).seconds() > 2.0) {
        RCLCPP_INFO(main_node->get_logger(), "In task state");
        if (loop_cartesian_traj_handle.has_value()) {
          RCLCPP_INFO(main_node->get_logger(),
                      "Still executing loop trajectory");
        }
        start = main_node->now();
      }
      break;
      // Read the human_state and handle the task accordingly; when human
      // enter
      // -> go to transition_human
    }
    case SceneState::transition_human: {

      update_state();

      //
      // Stop the task, save the state and enter compliance mode -> go to
      // compliance
      //
      RCLCPP_INFO(main_node->get_logger(), "In transition_human state");
      // Issue exponential decay of velocity and acceleration at current read
      // pose

      RCLCPP_INFO(main_node->get_logger(),
                  "Sending velocity and acceleration to 0 exponentially");
      stop_traj_action_client->async_send_goal(stop_traj_goal,
                                               stop_traj_options);
      while (!stop_traj_handle.has_value()) {
        RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::sleep_for(2s);
      }

      while (stop_traj_handle.has_value()) {
        RCLCPP_INFO(main_node->get_logger(),
                    "Waiting robot to reach 0 velocity and acceleration");
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::sleep_for(2s);
      }

      // Check if action has been interrupted from the human entering the area
      if (state != SceneState::transition_human || !rclcpp::ok()) {
        break;
      }
      compliance_request.cmd = true;
      auto compliance_future = compliance_mode_client->async_send_request(
          std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
              compliance_request));
      bool stopped = false;
      while (!stopped) {

        auto res = compliance_future.wait_for(10ms);
        while (res != std::future_status::ready && rclcpp::ok()) {
          res = compliance_future.wait_for(10ms);
          RCLCPP_INFO(main_node->get_logger(),
                      "Robot entering compliance mode");
        }
        if (res == std::future_status::ready) {
          RCLCPP_INFO(main_node->get_logger(),
                      "Robot stopped and passed in compliance mode");
          stopped = true;
          state = SceneState::compliance;
        }
      }
      break;
    }
    case SceneState::compliance: {
      update_state();
      RCLCPP_INFO(main_node->get_logger(), "In compliance state");
      std::this_thread::sleep_for(2s);
      break;
      // Read the human_state and handle the compliance mode accordingly; when
      // human leaves -> go to transition_leave_human
    }
    case SceneState::transition_leave_human: {

      update_state();
      RCLCPP_INFO(main_node->get_logger(), "In transition_leave_human state");

      send_current_pose_as_cmd(pose_state.pose);

      compliance_request.cmd = false;
      auto compliance_future = compliance_mode_client->async_send_request(
          std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
              compliance_request));
      bool stopped = false;
      while (!stopped) {

        auto res = compliance_future.wait_for(10ms);
        if (res == std::future_status::ready) {
          RCLCPP_INFO(main_node->get_logger(),
                      "Robot passed in non compliance mode");
          stopped = true;
          state = SceneState::task;
        } else {
          RCLCPP_INFO(main_node->get_logger(), "Robot exiting compliance mode");
        }
      }

      RCLCPP_INFO(main_node->get_logger(), "Sending robot home pose");
      cart_traj_action_client->async_send_goal(home_goal, cart_traj_options);
      while (!cartesian_traj_handle.has_value()) {
        RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::sleep_for(2s);
      }

      while (cartesian_traj_handle.has_value()) {
        RCLCPP_INFO(main_node->get_logger(),
                    "Waiting robot to reach home pose");
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::sleep_for(2s);
      }

      // Check if action has been interrupted from the human entering the area
      if (state != SceneState::transition_leave_human || !rclcpp::ok()) {
        break;
      }

      state = SceneState::task;
      break;
      // Exit from compliance mode staying in current pose, return to last
      // state pose, resume the task -> go to task
    } break;
    }
  }
  RCLCPP_INFO(main_node->get_logger(), "Requested shutdown");

  cancel_actions();
  executor.cancel();
  threads_run.store(false);
  info_thread.join();
  calculate_proximity.join();
  safe_keeper_thread.join();

  rclcpp::shutdown();
}
