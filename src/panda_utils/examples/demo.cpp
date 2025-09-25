#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_processing/constants.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_interfaces/action/stop_traj.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_interfaces/msg/human_detected.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_interfaces/srv/wrist_contact.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
#include <rclcpp/executors.hpp>
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
#include <rclcpp_action/exceptions.hpp>
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
// const rclcpp::Duration max_tf_age = rclcpp::Duration::from_seconds(1.0
// / 30.0);
const rclcpp::Duration max_wrist_tfs_age =
    rclcpp::Duration::from_seconds(1.0 / 5.0);
// const double robot_radius_area = 1.0;       // meters
const double min_distance_from_joint = 1.0; // meters

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

  // home_pose.orientation.w = 0.0;
  // home_pose.orientation.x = 1.0;
  // home_pose.orientation.y = 0.0;
  // home_pose.orientation.z = 0.0;

  home_pose.orientation.w = 0.173648;
  home_pose.orientation.x = 0.984808;
  home_pose.orientation.y = 0.0;
  home_pose.orientation.z = 0.0;

  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose desired_pose;
  panda_interfaces::msg::CartesianCommand cartesian_cmd;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  SceneState state{SceneState::no_state};
  human_presence::HumanPresentState presence_state;
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
  home_goal.total_time = 5.0;

  // Stop trajectory goal definition
  panda_interfaces::action::StopTraj_Goal stop_traj_goal;
  stop_traj_goal.total_time = 1.5;

  RCLCPP_INFO(main_node->get_logger(), "Defining Task goal: triangle");
  // Task goal definition: triangle
  Eigen::Quaterniond triangle_orient{
      home_pose.orientation.w, home_pose.orientation.x, home_pose.orientation.y,
      home_pose.orientation.z};
  panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal =
      generate_triangle_task(home_pose.position.x, home_pose.position.y,
                             home_pose.position.z, 0.1, 0.1,
                             triangle_orient.normalized(), 15.0);

  // Topic to read for scene infos

  auto joint_states_cb = [&joint_states](const JointState msg) {
    joint_states = msg;
  };

  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub =
      sub_node->create_subscription<JointState>(
          panda_interface_names::joint_state_topic_name,
          panda_interface_names::CONTROLLER_PUBLISHER_QOS(), joint_states_cb);

  auto pose_state_cb = [&pose_state](const PoseStamped msg) {
    pose_state = msg;
  };

  rclcpp::Subscription<PoseStamped>::SharedPtr pose_state_sub =
      sub_node->create_subscription<PoseStamped>(
          panda_interface_names::panda_pose_state_topic_name,
          panda_interface_names::CONTROLLER_PUBLISHER_QOS(), pose_state_cb);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_present_sub =
      sub_node->create_subscription<std_msgs::msg::Bool>(
          panda_interface_names::human_presence_topic,
          panda_interface_names::DEFAULT_TOPIC_QOS(),
          [&presence_state](const std_msgs::msg::Bool present) {
            presence_state.human_present = present.data;
          });

  tf_buffer = std::make_unique<tf2_ros::Buffer>(sub_node->get_clock());

  std::unique_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer, sub_node);

  // Compliance mode service client
  rclcpp::Client<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_client =
          main_node->create_client<panda_interfaces::srv::SetComplianceMode>(
              panda_interface_names::set_compliance_mode_service_name);
  panda_interfaces::srv::SetComplianceMode_Request compliance_request;

  // Human presence service client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr human_presence_enabler =
      main_node->create_client<std_srvs::srv::SetBool>(
          panda_interface_names::enable_human_presence_service_name);
  std_srvs::srv::SetBool_Request::SharedPtr enable_human_presence_request =
      std::make_shared<std_srvs::srv::SetBool_Request>();

  // Contact joint service
  rclcpp::Client<panda_interfaces::srv::WristContact>::SharedPtr
      wrist_contact_index_client =
          main_node->create_client<panda_interfaces::srv::WristContact>(
              panda_interface_names::set_wrist_contact_service_name);

  // Publisher for the update of desired commands when switching to compliance
  // mode

  rclcpp::Publisher<Pose>::SharedPtr pose_cmd_pub =
      main_node->create_publisher<Pose>(
          panda_interface_names::panda_pose_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS());
  rclcpp::Publisher<Twist>::SharedPtr twist_cmd_pub =
      main_node->create_publisher<Twist>(
          panda_interface_names::panda_twist_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS());
  rclcpp::Publisher<Accel>::SharedPtr accel_cmd_pub =
      main_node->create_publisher<Accel>(
          panda_interface_names::panda_accel_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS());
  rclcpp::Publisher<panda_interfaces::msg::CartesianCommand>::SharedPtr
      cartesian_cmd_pub =
          main_node->create_publisher<panda_interfaces::msg::CartesianCommand>(
              "/panda/cartesian_cmd",
              panda_interface_names::DEFAULT_TOPIC_QOS());

  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr state_publisher =
      main_node->create_publisher<std_msgs::msg::ColorRGBA>(
          panda_interface_names::demo_state_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS());

  auto update_state = [&state_publisher, &state]() {
    publish_state(state_publisher, state);
  };

  auto send_current_pose_as_cmd = [&cartesian_cmd_pub,
                                   &cartesian_cmd](const Pose desired_pose) {
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

    cartesian_cmd.twist = twist;
    cartesian_cmd.accel = accel;
    cartesian_cmd.pose = desired_pose;

    cartesian_cmd_pub->publish(cartesian_cmd);
    // twist_cmd_pub->publish(twist);
    // accel_cmd_pub->publish(accel);
    // pose_cmd_pub->publish(desired_pose);
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

  rclcpp::Node::SharedPtr confirmation_node =
      std::make_shared<rclcpp::Node>("actions_confirmation_node");

  // Management of action nodes for the 2 types of trajectories
  auto cancel_actions = [&cartesian_traj_handle, &loop_cartesian_traj_handle,
                         &stop_traj_handle, &stop_traj_action_client,
                         &cart_traj_action_client, &loop_traj_action_client,
                         &main_node]() {
    if (cartesian_traj_handle.has_value()) {
      try {
        cart_traj_action_client->async_cancel_goal(
            cartesian_traj_handle.value());
        RCLCPP_INFO(main_node->get_logger(),
                    "Requested cancel of cartesian trajectory action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(main_node->get_logger(),
                           "Goal handle has null value or terminated");
      }
    }

    if (loop_cartesian_traj_handle.has_value()) {
      try {
        loop_traj_action_client->async_cancel_goal(
            loop_cartesian_traj_handle.value());
        RCLCPP_INFO(main_node->get_logger(),
                    "Requested cancel of loop cartesian trajectory action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(main_node->get_logger(),
                           "Goal handle has null value or terminated");
      }
    }

    if (stop_traj_handle.has_value()) {
      try {
        stop_traj_action_client->async_cancel_goal(stop_traj_handle.value());
        RCLCPP_INFO(main_node->get_logger(),
                    "Requested cancel of exponential stop action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(main_node->get_logger(),
                           "Goal handle has null value or terminated");
      }
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

  RCLCPP_INFO(main_node->get_logger(), "Waiting for human presence server");
  human_presence_enabler->wait_for_service(10s);

  RCLCPP_INFO(main_node->get_logger(), "Servers UP");

  // Go to initial pose
  // Begin task: cancel when a human enters the area
  // Enter in compliance mode: exit when human leaves the area, staying in
  // current pose Go back to last task state Resume task

  std::thread info_thread([&executor]() { executor.spin(); });

  std::thread safe_keeper_thread{[cancel_actions, &main_node, &presence_state,
                                  &state, &threads_run,
                                  &wrist_contact_index_client]() {
    while (!threads_run.load()) {
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO_STREAM(main_node->get_logger(), "Started thread 'safe_keeper'");

    bool wrist_in_contact = false;
    panda_interfaces::srv::WristContact_Request::SharedPtr
        wrist_contact_request =
            std::make_shared<panda_interfaces::srv::WristContact_Request>();
    auto send_req_wrist_contact =
        [&wrist_contact_index_client, &main_node](
            panda_interfaces::srv::WristContact_Request::SharedPtr req) {
          auto future = wrist_contact_index_client->async_send_request(req);
          bool stopped = false;
          while (!stopped) {
            auto res = future.wait_for(10ms);
            if (res == std::future_status::ready) {
              stopped = true;
            } else {
              rclcpp::sleep_for(1ms);
            }
          }
        };

    while (threads_run.load()) {
      {
        std::shared_lock<std::shared_mutex> mutex(presence_state.mut);
        if (!wrist_in_contact && presence_state.contact_wrist.has_value()) {
          // Send request to alert the contact
          RCLCPP_INFO(main_node->get_logger(), "Wrist contact alerted");
          wrist_contact_request->contact = true;
          wrist_contact_request->wrist.data =
              presence_state.contact_wrist.value();
          send_req_wrist_contact(wrist_contact_request);

          wrist_in_contact = true;
        } else if (wrist_in_contact &&
                   !presence_state.contact_wrist.has_value()) {
          // Send request to alert the loss of contact
          RCLCPP_INFO(main_node->get_logger(), "Wrist uncontact alerted");
          wrist_contact_request->contact = false;
          wrist_contact_request->wrist.data = "";
          send_req_wrist_contact(wrist_contact_request);

          wrist_in_contact = false;
        }

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
      }

      std::this_thread::sleep_for(5ms);
    }
  }};

  // std::thread calculate_proximity{[&tf_buffer, &sub_node, &presence_state,
  //                                  &threads_run] {
  //   // TODO: handle 2 things:
  //   // Check if base robot link exists
  //   std::map<std::string, geometry_msgs::msg::TransformStamped>
  //       robot_links_body_keypoints_tfs;
  //   std::map<std::string, geometry_msgs::msg::TransformStamped>
  //       robot_base_joints_tfs;
  //
  //   while (!threads_run.load()) {
  //     std::this_thread::sleep_for(1s);
  //   }
  //
  //   RCLCPP_INFO_STREAM(sub_node->get_logger(),
  //                      "Calculate proximity thread started");
  //
  //   rclcpp::Time last_time{sub_node->now()};
  //   rclcpp::Time last_time_print{sub_node->now()};
  //
  //   while (threads_run.load() && rclcpp::ok()) {
  //
  //     bool print_every_half_sec =
  //         (sub_node->now() - last_time_print).seconds() > 0.5;
  //
  //     if (tf_buffer->canTransform(robot_base_frame_name, world_frame,
  //                                 tf2::TimePointZero)) {
  //       // Robot base exists, now get the transforms of the body keypoints
  //       // wrt the base
  //       auto now = sub_node->now();
  //       // Clear the map to avoid getting false data
  //       robot_links_body_keypoints_tfs.clear();
  //       // Cycle through all the body keypoints frame: get the transform if
  //       // the body frame is relatively new in the tf2 system and obvoiusly
  //       // exists
  //       for (auto keypoint_frame_name : image_constants::coco_keypoints) {
  //         try {
  //           auto keypoint_frame = tf_buffer->lookupTransform(
  //               keypoint_frame_name, world_frame, tf2::TimePointZero);
  //
  //           if (now - keypoint_frame.header.stamp >= max_tf_age) {
  //             continue;
  //           }
  //         } catch (const tf2::TransformException &ex) {
  //           continue;
  //         }
  //
  //         for (auto robot_frame_name :
  //              panda_interface_names::panda_link_names) {
  //           try {
  //             auto tf = tf_buffer->lookupTransform(
  //                 robot_frame_name, keypoint_frame_name, tf2::TimePointZero);
  //             robot_links_body_keypoints_tfs[robot_frame_name + "_to_" +
  //                                            keypoint_frame_name] = tf;
  //           } catch (const tf2::TransformException &ex) {
  //             // Tf does not exists
  //           }
  //         }
  //       }
  //
  //       if (print_every_half_sec) {
  //
  //         RCLCPP_INFO_STREAM(sub_node->get_logger(),
  //                            robot_links_body_keypoints_tfs.size()
  //                                << " tfs looked");
  //       }
  //
  //       // Now will check for proximity with the entire robot
  //       bool human_in_area = false;
  //       for (std::pair<std::string, geometry_msgs::msg::TransformStamped>
  //                named_tf : robot_links_body_keypoints_tfs) {
  //         double dist = geom_utils::distance(named_tf.second);
  //         if (dist <= robot_radius_area) {
  //           human_in_area = true;
  //           break;
  //         }
  //       }
  //       // Increase the time the human have been seen inside the area or
  //       // decrease if he's not there
  //       {
  //         std::lock_guard<std::shared_mutex> mutex(presence_state.mut);
  //         if (human_in_area) {
  //           presence_state.time_present += (sub_node->now() - last_time);
  //         } else {
  //           presence_state.time_present -= (sub_node->now() - last_time);
  //         }
  //         // Normalize the time according to limit
  //         presence_state.normalize_time();
  //         if (print_every_half_sec) {
  //
  //           RCLCPP_INFO_STREAM(sub_node->get_logger(),
  //                              "Presence time: "
  //                                  << presence_state.time_present.seconds()
  //                                  << "[s]");
  //         }
  //         if (human_in_area &&
  //             presence_state.time_present.seconds() >=
  //                 presence_state.MAX_TIME &&
  //             !presence_state.human_present) {
  //           // If human is inside by max time at least then he's surely in
  //           // area
  //           RCLCPP_INFO_STREAM(sub_node->get_logger(),
  //                              "The human entered the robot area");
  //           presence_state.human_present = true;
  //         } else if (!human_in_area &&
  //                    presence_state.time_present.seconds() == 0.0 &&
  //                    presence_state.human_present) {
  //           RCLCPP_INFO_STREAM(sub_node->get_logger(),
  //                              "The human left the robot area");
  //           presence_state.human_present = false;
  //         }
  //         // We leave the mutex property here because we need to recall
  //         // other tfs
  //       }
  //
  //       // Now, if the human is in the area, we have to check if he's
  //       // touching a joint Don't check if every frame exists, we'll let
  //       // throw the exception if doesn't still exist and continue
  //       if (presence_state.human_present) {
  //
  //         std::vector<double> left_wrist_distances;
  //         std::vector<double> right_wrist_distances;
  //
  //         // Calculate distances for left wrist to all links
  //         for (auto robot_link_name :
  //         panda_interface_names::panda_link_names) {
  //           try {
  //             auto tf =
  //             tf_buffer->lookupTransform(image_constants::left_wrist,
  //                                                  robot_link_name,
  //                                                  tf2::TimePointZero);
  //             if (now - tf.header.stamp <= max_wrist_tfs_age) {
  //               left_wrist_distances.push_back(geom_utils::distance(tf));
  //             }
  //           } catch (const tf2::TransformException &ex) {
  //             // Tf does not exist or is too old, skip for this link
  //           }
  //         }
  //
  //         // Calculate distances for right wrist to all links
  //         for (auto robot_link_name :
  //         panda_interface_names::panda_link_names) {
  //           try {
  //             auto tf =
  //             tf_buffer->lookupTransform(image_constants::right_wrist,
  //                                                  robot_link_name,
  //                                                  tf2::TimePointZero);
  //             if (now - tf.header.stamp <= max_wrist_tfs_age) {
  //               right_wrist_distances.push_back(geom_utils::distance(tf));
  //             }
  //           } catch (const tf2::TransformException &ex) {
  //             // Tf does not exist or is too old, skip for this link
  //           }
  //         }
  //
  //         std::optional<double> mean_dist_left;
  //         std::optional<double> min_dist_left = 100;
  //         if (!left_wrist_distances.empty()) {
  //           double sum = 0.0;
  //           for (double d : left_wrist_distances) {
  //             if (d < min_dist_left) {
  //               min_dist_left = d;
  //             }
  //             sum += d;
  //           }
  //           mean_dist_left = sum / left_wrist_distances.size();
  //         }
  //
  //         std::optional<double> mean_dist_right;
  //         std::optional<double> min_dist_right = 100;
  //         if (!right_wrist_distances.empty()) {
  //           double sum = 0.0;
  //           for (double d : right_wrist_distances) {
  //             if (d < min_dist_right) {
  //               min_dist_right = d;
  //             }
  //             sum += d;
  //           }
  //           mean_dist_right = sum / right_wrist_distances.size();
  //         }
  //
  //         std::optional<std::string> active_wrist_frame{std::nullopt};
  //
  //         if (mean_dist_left.has_value() &&
  //             (!mean_dist_right.has_value() ||
  //              mean_dist_left.value() < mean_dist_right.value()) &&
  //             min_dist_left.value() <= min_distance_from_joint) {
  //           active_wrist_frame = image_constants::left_wrist;
  //         } else if (mean_dist_right.has_value() &&
  //                    min_dist_right.value() <= min_distance_from_joint) {
  //           active_wrist_frame = image_constants::right_wrist;
  //         }
  //
  //         // We update the contact joint var
  //         {
  //           std::shared_lock<std::shared_mutex> mutex(presence_state.mut);
  //           presence_state.contact_wrist = active_wrist_frame;
  //         }
  //       }
  //     }
  //     // - Proximity value as bool, depending on distance from working area
  //     // - Minimum distance between each joint of the robot and the human
  //     // wrist, if they are in scene
  //
  //     last_time = sub_node->now();
  //     if (print_every_half_sec) {
  //       last_time_print = sub_node->now();
  //     }
  //     std::this_thread::sleep_for(10ms);
  //   }
  // }};

  {
    RCLCPP_INFO(main_node->get_logger(), "Press enter to start demo");
    std::cin.ignore();
    threads_run.store(true);
    RCLCPP_INFO(main_node->get_logger(), "Threads running");
  }

  enable_human_presence_request->set__data(true);
  human_presence_enabler->async_send_request(enable_human_presence_request);

  auto start = rclcpp::Time(0, 0, main_node->get_clock()->get_clock_type());

  // Resetting compliance mode if set in controller
  // Firstly, update the desired pose and other commands to the read one
  send_current_pose_as_cmd(pose_state.pose);
  // Sleep to avoid that the robot passes in non compliance mode before updating
  // his desired state
  std::this_thread::sleep_for(500ms);

  compliance_request.cmd = false;
  auto compliance_future = compliance_mode_client->async_send_request(
      std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
          compliance_request));
  bool stopped = false;
  while (!stopped && rclcpp::ok()) {
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
        RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
        auto future = cart_traj_action_client->async_send_goal(
            home_goal, cart_traj_options);
        auto fut_return =
            rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
        switch (fut_return) {
        case rclcpp::FutureReturnCode::SUCCESS: {
          auto handle = future.get();
          if (handle) {
            cartesian_traj_handle = handle;
            RCLCPP_INFO(main_node->get_logger(),
                        "Cartesian trajectory accepted");
          } else {
            RCLCPP_INFO(main_node->get_logger(),
                        "Cartesian trajectory refused");
            break;
          }
          break;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
          RCLCPP_ERROR(main_node->get_logger(),
                       "Cartesian trajectory interrupted");
          break;
        }
        case rclcpp::FutureReturnCode::TIMEOUT: {
          RCLCPP_ERROR(main_node->get_logger(),
                       "Cartesian trajectory went timeout");
          break;
        } break;
        };

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
        auto future = loop_traj_action_client->async_send_goal(
            triangle_task_goal, loop_cart_traj_options);
        auto fut_return =
            rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
        switch (fut_return) {
        case rclcpp::FutureReturnCode::SUCCESS: {
          auto handle = future.get();
          if (handle) {
            loop_cartesian_traj_handle = handle;
            RCLCPP_INFO(main_node->get_logger(),
                        "Loop cartesian trajectory accepted");
            start = main_node->now();
          } else {
            RCLCPP_INFO(main_node->get_logger(),
                        "Loop cartesian trajectory refused");
            break;
          }
          break;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
          RCLCPP_ERROR(main_node->get_logger(),
                       "Loop cartesian trajectory interrupted");
          break;
        }
        case rclcpp::FutureReturnCode::TIMEOUT: {
          RCLCPP_ERROR(main_node->get_logger(),
                       "Loop cartesian trajectory went timeout");
          break;
        } break;
        };
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
      auto future = stop_traj_action_client->async_send_goal(stop_traj_goal,
                                                             stop_traj_options);
      auto fut_return =
          rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
      switch (fut_return) {
      case rclcpp::FutureReturnCode::SUCCESS: {
        auto handle = future.get();
        if (handle) {
          stop_traj_handle = handle;
          RCLCPP_INFO(main_node->get_logger(), "Stop trajectory accepted");
        } else {
          RCLCPP_INFO(main_node->get_logger(), "Stop trajectory refused");
          break;
        }
        break;
      }
      case rclcpp::FutureReturnCode::INTERRUPTED: {
        RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory interrupted");
        break;
      }
      case rclcpp::FutureReturnCode::TIMEOUT: {
        RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory went timeout");
        break;
      } break;
      };

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
      RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
      auto future = cart_traj_action_client->async_send_goal(home_goal,
                                                             cart_traj_options);
      auto fut_return =
          rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
      switch (fut_return) {
      case rclcpp::FutureReturnCode::SUCCESS: {
        auto handle = future.get();
        if (handle) {
          cartesian_traj_handle = handle;
          RCLCPP_INFO(main_node->get_logger(), "Cartesian trajectory accepted");
        } else {
          RCLCPP_INFO(main_node->get_logger(), "Cartesian trajectory refused");
          break;
        }
        break;
      }
      case rclcpp::FutureReturnCode::INTERRUPTED: {
        RCLCPP_ERROR(main_node->get_logger(),
                     "Cartesian trajectory interrupted");
        break;
      }
      case rclcpp::FutureReturnCode::TIMEOUT: {
        RCLCPP_ERROR(main_node->get_logger(),
                     "Cartesian trajectory went timeout");
        break;
      } break;
      };

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
  RCLCPP_INFO(main_node->get_logger(),
              "Sending velocity and acceleration to 0 exponentially");
  auto future = stop_traj_action_client->async_send_goal(stop_traj_goal,
                                                         stop_traj_options);
  auto fut_return =
      rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
  switch (fut_return) {
  case rclcpp::FutureReturnCode::SUCCESS: {
    auto handle = future.get();
    if (handle) {
      stop_traj_handle = handle;
      RCLCPP_INFO(main_node->get_logger(), "Stop trajectory accepted");
    } else {
      RCLCPP_INFO(main_node->get_logger(), "Stop trajectory refused");
      break;
    }
    break;
  }
  case rclcpp::FutureReturnCode::INTERRUPTED: {
    RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory interrupted");
    break;
  }
  case rclcpp::FutureReturnCode::TIMEOUT: {
    RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory went timeout");
    break;
  } break;
  };

  while (stop_traj_handle.has_value()) {
    RCLCPP_INFO(main_node->get_logger(),
                "Waiting robot to reach 0 velocity and acceleration");
    if (!rclcpp::ok()) {
      break;
    }
    rclcpp::sleep_for(2s);
  }
  executor.cancel();
  threads_run.store(false);
  info_thread.join();
  safe_keeper_thread.join();

  rclcpp::shutdown();
}
