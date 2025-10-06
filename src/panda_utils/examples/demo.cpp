#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_processing/constants.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_interfaces/action/stop_traj.hpp"
#include "panda_interfaces/msg/cartesian_command.hpp"
#include "panda_interfaces/msg/human_contact.hpp"
#include "panda_interfaces/msg/human_detected.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
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

class DemoNode : public rclcpp::Node {
public:
  DemoNode() : Node("demo_thread_manager_node") {

    spawn_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_demo",
        std::bind(&DemoNode::handleSpawnDemoThread, this, std::placeholders::_1,
                  std::placeholders::_2));

    RCLCPP_INFO(
        this->get_logger(),
        "DemoThreadManagedNode created. Service 'spawn_demo_thread' ready.");

    this->declare_parameter<std::vector<double>>(
        "home_pose", std::vector<double>{0.6, 0.0, 0.5});

    // Initilizing variables
    home_pose_.orientation.w = 0.0;
    home_pose_.orientation.x = 1.0;
    home_pose_.orientation.y = 0.0;
    home_pose_.orientation.z = 0.0;

    std::vector<double> home_pose;
    this->get_parameter<std::vector<double>>("home_pose", home_pose);

    home_pose_.position.x = home_pose[0];
    home_pose_.position.y = home_pose[1];
    home_pose_.position.z = home_pose[2];

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    confirmation_node_ =
        std::make_shared<rclcpp::Node>("actions_confirmation_node");

    std::unique_ptr<tf2_ros::TransformListener> tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

    auto pose_state_cb = [this](const PoseStamped msg) { pose_state_ = msg; };

    pose_state_sub_ = this->create_subscription<PoseStamped>(
        panda_interface_names::panda_pose_state_topic_name,
        panda_interface_names::CONTROLLER_PUBLISHER_QOS(), pose_state_cb);

    human_present_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        panda_interface_names::human_presence_topic,
        panda_interface_names::DEFAULT_TOPIC_QOS(),
        [this](const std_msgs::msg::Bool present) {
          presence_state_.human_present = present.data;
        });

    compliance_mode_client_ =
        confirmation_node_
            ->create_client<panda_interfaces::srv::SetComplianceMode>(
                panda_interface_names::set_compliance_mode_service_name);

    human_presence_enabler_ = this->create_client<std_srvs::srv::SetBool>(
        panda_interface_names::enable_human_presence_service_name);

    cartesian_cmd_pub_ =
        this->create_publisher<panda_interfaces::msg::CartesianCommand>(
            "/panda/cartesian_cmd", panda_interface_names::DEFAULT_TOPIC_QOS());

    state_publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>(
        panda_interface_names::demo_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS());

    cart_traj_action_client_ = rclcpp_action::create_client<CartTraj>(
        this, panda_interface_names::panda_cart_move_action_name);

    loop_traj_action_client_ = rclcpp_action::create_client<LoopCartTraj>(
        this, panda_interface_names::panda_cart_loop_action_name);

    stop_traj_action_client_ = rclcpp_action::create_client<StopTraj>(
        this, panda_interface_names::panda_exponential_stop_action_name);

    auto cart_result_callback =
        [this](
            const rclcpp_action::ClientGoalHandle<CartTraj>::WrappedResult &) {
          cartesian_traj_handle_ = std::nullopt;
        };
    cart_traj_options_.result_callback = cart_result_callback;

    auto loop_result_callback =
        [this](
            const rclcpp_action::ClientGoalHandle<LoopCartTraj>::WrappedResult
                &) { loop_cartesian_traj_handle_ = std::nullopt; };
    loop_cart_traj_options_.result_callback = loop_result_callback;

    auto stop_result_callback =
        [this](
            const rclcpp_action::ClientGoalHandle<StopTraj>::WrappedResult &) {
          stop_traj_handle_ = std::nullopt;
        };
    stop_traj_options_.result_callback = stop_result_callback;

    home_goal_.desired_pose = home_pose_;
    home_goal_.total_time = 3.0;

    Eigen::Quaterniond triangle_orient{
        home_pose_.orientation.w, home_pose_.orientation.x,
        home_pose_.orientation.y, home_pose_.orientation.z};
    triangle_task_goal_ = generate_triangle_task(
        home_pose_.position.x, home_pose_.position.y, home_pose_.position.z,
        0.1, 0.2, triangle_orient.normalized(), 9.0);

    stop_traj_goal_.total_time = 1.5;
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr spawn_service_;

  rclcpp::Client<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr human_presence_enabler_;

  rclcpp::Subscription<PoseStamped>::SharedPtr pose_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_present_sub_;

  rclcpp::Publisher<panda_interfaces::msg::CartesianCommand>::SharedPtr
      cartesian_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr state_publisher_;

  rclcpp::Node::SharedPtr confirmation_node_;

  Pose home_pose_;
  PoseStamped pose_state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  SceneState state_{SceneState::no_state};
  human_presence::HumanPresentState presence_state_;
  std::atomic<bool> threads_run_{false};

  std::thread safe_keeper_thread_;
  std::thread demo_thread_;

  // Action variables
  OptionalGoalHandle<CartTraj> cartesian_traj_handle_;
  OptionalGoalHandle<LoopCartTraj> loop_cartesian_traj_handle_;
  OptionalGoalHandle<StopTraj> stop_traj_handle_;

  // Goals
  panda_interfaces::action::CartTraj_Goal home_goal_;
  panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal_;
  panda_interfaces::action::StopTraj_Goal stop_traj_goal_;

  // Goal options
  GoalOptions<CartTraj> cart_traj_options_;
  GoalOptions<LoopCartTraj> loop_cart_traj_options_;
  GoalOptions<StopTraj> stop_traj_options_;

  rclcpp_action::Client<CartTraj>::SharedPtr cart_traj_action_client_;
  rclcpp_action::Client<LoopCartTraj>::SharedPtr loop_traj_action_client_;
  rclcpp_action::Client<StopTraj>::SharedPtr stop_traj_action_client_;

  void
  handleSpawnDemoThread(const std_srvs::srv::SetBool_Request::SharedPtr req,
                        std_srvs::srv::SetBool_Response::SharedPtr resp) {

    if (req->data && !threads_run_.load()) {

      state_ = SceneState::no_state;

      RCLCPP_INFO(this->get_logger(), "Waiting for servers...");

      RCLCPP_INFO(this->get_logger(), "Waiting for cartesian trajectory");
      cart_traj_action_client_->wait_for_action_server(10s);

      RCLCPP_INFO(this->get_logger(), "Waiting for loop cartesian trajectory");
      loop_traj_action_client_->wait_for_action_server(10s);

      RCLCPP_INFO(this->get_logger(),
                  "Waiting for exponential stop trajectory");
      stop_traj_action_client_->wait_for_action_server(10s);

      RCLCPP_INFO(this->get_logger(), "Waiting for compliance mode server");
      compliance_mode_client_->wait_for_service(10s);

      RCLCPP_INFO(this->get_logger(), "Waiting for human presence server");
      human_presence_enabler_->wait_for_service(10s);

      RCLCPP_INFO(this->get_logger(), "Servers UP");

      send_current_pose_as_cmd(pose_state_.pose);
      // Sleep to avoid that the robot passes in non compliance mode before
      // updating his desired state
      std::this_thread::sleep_for(500ms);

      if (!switch_compliance_mode(false)) {
        resp->success = false;
        return;
      }

      safe_keeper_thread_ = std::thread{[this]() {
        while (!threads_run_.load()) {
          std::this_thread::sleep_for(1s);
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Started thread 'safe_keeper'");

        while (threads_run_.load()) {
          {
            if (presence_state_.human_present &&
                state_ != SceneState::transition_human &&
                state_ != SceneState::compliance) {
              RCLCPP_INFO_STREAM(this->get_logger(),
                                 "Human present in scene, cancelling "
                                 "actions and transitioning");
              cancel_actions();
              state_ = SceneState::transition_human;
            } else if (!presence_state_.human_present &&
                       state_ == SceneState::compliance) {
              RCLCPP_INFO_STREAM(this->get_logger(),
                                 "Human left scene, transitioning");
              state_ = SceneState::transition_leave_human;
            }
          }

          std::this_thread::sleep_for(5ms);
        }
      }};

      std_srvs::srv::SetBool_Request::SharedPtr enable_human_presence_request =
          std::make_shared<std_srvs::srv::SetBool_Request>();
      enable_human_presence_request->set__data(true);
      human_presence_enabler_->async_send_request(
          enable_human_presence_request);

      update_state();

      demo_thread_ = std::thread{[this]() {
        while (rclcpp::ok() && threads_run_.load()) {

          while (!threads_run_.load()) {
            std::this_thread::sleep_for(1s);
          }

          switch (state_) {
          case SceneState::no_state: {

            update_state();

            // Ensure the robot is up and running
            // Going to pose
            RCLCPP_INFO(this->get_logger(), "Robot has no known state");

            if (!cartesian_traj_handle_.has_value()) {

              if (!rclcpp::ok()) {
                break;
              }

              RCLCPP_INFO(this->get_logger(), "Sending robot home pose");
              RCLCPP_INFO(this->get_logger(), "Waiting for goal confirmation");
              auto future = cart_traj_action_client_->async_send_goal(
                  home_goal_, cart_traj_options_);
              auto fut_return = rclcpp::spin_until_future_complete(
                  confirmation_node_, future, 3s);
              switch (fut_return) {
              case rclcpp::FutureReturnCode::SUCCESS: {
                auto handle = future.get();
                if (handle) {
                  cartesian_traj_handle_ = handle;
                  RCLCPP_INFO(this->get_logger(),
                              "Cartesian trajectory accepted");
                } else {
                  RCLCPP_INFO(this->get_logger(),
                              "Cartesian trajectory refused");
                  break;
                }
                break;
              }
              case rclcpp::FutureReturnCode::INTERRUPTED: {
                RCLCPP_ERROR(this->get_logger(),
                             "Cartesian trajectory interrupted");
                break;
              }
              case rclcpp::FutureReturnCode::TIMEOUT: {
                RCLCPP_ERROR(this->get_logger(),
                             "Cartesian trajectory went timeout");
                break;
              } break;
              };

              while (cartesian_traj_handle_.has_value()) {
                RCLCPP_INFO(this->get_logger(),
                            "Waiting robot to reach home pose");
                if (!rclcpp::ok()) {
                  break;
                }
                rclcpp::sleep_for(2s);
              }

              // Check if action has been interrupted from the human entering
              // the area
              if (state_ != SceneState::no_state || !rclcpp::ok()) {
                break;
              }

              RCLCPP_INFO(
                  this->get_logger(),
                  "Robot reached home pose, transitioning to task state");
              state_ = SceneState::task;
              break;
            }
            break;
          }
          case SceneState::task: {

            update_state();

            if (!loop_cartesian_traj_handle_.has_value()) {
              auto future = loop_traj_action_client_->async_send_goal(
                  triangle_task_goal_, loop_cart_traj_options_);
              auto fut_return = rclcpp::spin_until_future_complete(
                  confirmation_node_, future, 3s);
              switch (fut_return) {
              case rclcpp::FutureReturnCode::SUCCESS: {
                auto handle = future.get();
                if (handle) {
                  loop_cartesian_traj_handle_ = handle;
                  RCLCPP_INFO(this->get_logger(),
                              "Loop cartesian trajectory accepted");
                } else {
                  RCLCPP_INFO(this->get_logger(),
                              "Loop cartesian trajectory refused");
                  break;
                }
                break;
              }
              case rclcpp::FutureReturnCode::INTERRUPTED: {
                RCLCPP_ERROR(this->get_logger(),
                             "Loop cartesian trajectory interrupted");
                break;
              }
              case rclcpp::FutureReturnCode::TIMEOUT: {
                RCLCPP_ERROR(this->get_logger(),
                             "Loop cartesian trajectory went timeout");
                break;
              } break;
              };
            }

            break;
            // Read the human_state and handle the task accordingly; when
            // human enter
            // -> go to transition_human
          }
          case SceneState::transition_human: {

            update_state();

            //
            // Stop the task, save the state and enter compliance mode -> go
            // to compliance
            //
            RCLCPP_INFO(this->get_logger(), "In transition_human state");
            // Issue exponential decay of velocity and acceleration at current
            // read pose

            RCLCPP_INFO(this->get_logger(),
                        "Sending velocity and acceleration to 0 exponentially");
            auto future = stop_traj_action_client_->async_send_goal(
                stop_traj_goal_, stop_traj_options_);
            auto fut_return = rclcpp::spin_until_future_complete(
                confirmation_node_, future, 500ms);
            switch (fut_return) {
            case rclcpp::FutureReturnCode::SUCCESS: {
              auto handle = future.get();
              if (handle) {
                stop_traj_handle_ = handle;
                RCLCPP_INFO(this->get_logger(), "Stop trajectory accepted");
              } else {
                RCLCPP_INFO(this->get_logger(), "Stop trajectory refused");
                break;
              }
              break;
            }
            case rclcpp::FutureReturnCode::INTERRUPTED: {
              RCLCPP_ERROR(this->get_logger(), "Stop trajectory interrupted");
              break;
            }
            case rclcpp::FutureReturnCode::TIMEOUT: {
              RCLCPP_ERROR(this->get_logger(), "Stop trajectory went timeout");
              break;
            } break;
            };

            while (stop_traj_handle_.has_value()) {
              RCLCPP_INFO(this->get_logger(),
                          "Waiting robot to reach 0 velocity and acceleration");
              if (!rclcpp::ok()) {
                break;
              }
              rclcpp::sleep_for(2s);
            }

            // Check if action has been interrupted from the human entering
            // the area
            if (state_ != SceneState::transition_human || !rclcpp::ok()) {
              break;
            }
            bool flag;

            do {
              flag = switch_compliance_mode(true);
            } while (!flag);

            break;
          }
          case SceneState::compliance: {
            update_state();
            RCLCPP_INFO(this->get_logger(), "In compliance state");
            std::this_thread::sleep_for(2s);
            break;
            // Read the human_state and handle the compliance mode
            // accordingly; when human leaves -> go to transition_leave_human
          }
          case SceneState::transition_leave_human: {

            update_state();
            RCLCPP_INFO(this->get_logger(), "In transition_leave_human state");

            send_current_pose_as_cmd(pose_state_.pose);

            bool flag;
            do {
              flag = switch_compliance_mode(false);
            } while (!flag);

            RCLCPP_INFO(this->get_logger(), "Sending robot home pose");
            RCLCPP_INFO(this->get_logger(), "Waiting for goal confirmation");
            auto future = cart_traj_action_client_->async_send_goal(
                home_goal_, cart_traj_options_);
            auto fut_return = rclcpp::spin_until_future_complete(
                confirmation_node_, future, 3s);
            switch (fut_return) {
            case rclcpp::FutureReturnCode::SUCCESS: {
              auto handle = future.get();
              if (handle) {
                cartesian_traj_handle_ = handle;
                RCLCPP_INFO(this->get_logger(),
                            "Cartesian trajectory accepted");
              } else {
                RCLCPP_INFO(this->get_logger(), "Cartesian trajectory refused");
                break;
              }
              break;
            }
            case rclcpp::FutureReturnCode::INTERRUPTED: {
              RCLCPP_ERROR(this->get_logger(),
                           "Cartesian trajectory interrupted");
              break;
            }
            case rclcpp::FutureReturnCode::TIMEOUT: {
              RCLCPP_ERROR(this->get_logger(),
                           "Cartesian trajectory went timeout");
              break;
            } break;
            };

            while (cartesian_traj_handle_.has_value()) {
              RCLCPP_INFO(this->get_logger(),
                          "Waiting robot to reach home pose");
              if (!rclcpp::ok()) {
                break;
              }
              rclcpp::sleep_for(2s);
            }

            // Check if action has been interrupted from the human entering
            // the area
            if (state_ != SceneState::transition_leave_human || !rclcpp::ok()) {
              break;
            }

            rclcpp::sleep_for(2s);

            RCLCPP_INFO(this->get_logger(), "Transitioning to task state");
            state_ = SceneState::task;
            break;
            // Exit from compliance mode staying in current pose, return
            // to last state pose, resume the task -> go to task
          } break;
          }
        }
      }};

      threads_run_.store(true);
      resp->set__success(true);
    } else if (!req->data && threads_run_.load()) {
      cancel_actions();
      threads_run_.store(false);
      safe_keeper_thread_.join();
      demo_thread_.join();

      RCLCPP_INFO(this->get_logger(),
                  "Sending velocity and acceleration to 0 exponentially");
      auto future = stop_traj_action_client_->async_send_goal(
          stop_traj_goal_, stop_traj_options_);
      auto fut_return =
          rclcpp::spin_until_future_complete(confirmation_node_, future, 500ms);
      switch (fut_return) {
      case rclcpp::FutureReturnCode::SUCCESS: {
        auto handle = future.get();
        if (handle) {
          stop_traj_handle_ = handle;
          RCLCPP_INFO(this->get_logger(), "Stop trajectory accepted");
        } else {
          RCLCPP_INFO(this->get_logger(), "Stop trajectory refused");
          break;
        }
        break;
      }
      case rclcpp::FutureReturnCode::INTERRUPTED: {
        RCLCPP_ERROR(this->get_logger(), "Stop trajectory interrupted");
        break;
      }
      case rclcpp::FutureReturnCode::TIMEOUT: {
        RCLCPP_ERROR(this->get_logger(), "Stop trajectory went timeout");
        break;
      } break;
      };

      while (stop_traj_handle_.has_value()) {
        RCLCPP_INFO(this->get_logger(),
                    "Waiting robot to reach 0 velocity and acceleration");
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::sleep_for(2s);
      }
      resp->set__success(true);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not stop/start the demo thread");
      resp->set__success(false);
    }
  }

  bool switch_compliance_mode(bool status) {
    panda_interfaces::srv::SetComplianceMode_Request compliance_request;
    compliance_request.cmd = status;
    auto compliance_future = compliance_mode_client_->async_send_request(
        std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
            compliance_request));
    RCLCPP_INFO(this->get_logger(), "Waiting for compliance mode confirmation");
    if (rclcpp::spin_until_future_complete(confirmation_node_,
                                           compliance_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Robot in non compliance mode");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Robot not entered compliance mode");
      return false;
    }
    // bool stopped = false;
    // while (!stopped && rclcpp::ok()) {
    //   auto res = compliance_future.wait_for(1s);
    //   if (res == std::future_status::ready) {
    //     RCLCPP_INFO(this->get_logger(), "Robot in non compliance mode");
    //     stopped = true;
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Robot exiting compliance mode");
    //   }
    // }
  }
  void update_state() { publish_state(state_publisher_, state_); };
  void send_current_pose_as_cmd(const Pose desired_pose) {
    panda_interfaces::msg::CartesianCommand cartesian_cmd;
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

    cartesian_cmd_pub_->publish(cartesian_cmd);
  };
  void cancel_actions() {
    if (cartesian_traj_handle_.has_value()) {
      try {
        cart_traj_action_client_->async_cancel_goal(
            cartesian_traj_handle_.value());
        RCLCPP_INFO(this->get_logger(),
                    "Requested cancel of cartesian trajectory action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Goal handle has null value or terminated");
      }
    }

    if (loop_cartesian_traj_handle_.has_value()) {
      try {
        loop_traj_action_client_->async_cancel_goal(
            loop_cartesian_traj_handle_.value());
        RCLCPP_INFO(this->get_logger(),
                    "Requested cancel of loop cartesian trajectory action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Goal handle has null value or terminated");
      }
    }

    if (stop_traj_handle_.has_value()) {
      try {
        stop_traj_action_client_->async_cancel_goal(stop_traj_handle_.value());
        RCLCPP_INFO(this->get_logger(),
                    "Requested cancel of exponential stop action");
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Goal handle has null value or terminated");
      }
    }
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();
  return 0;
}

// using geometry_msgs::msg::Accel;
// using geometry_msgs::msg::Pose;
// using geometry_msgs::msg::PoseStamped;
// using geometry_msgs::msg::Twist;
// using panda_interfaces::action::CartTraj;
// using panda_interfaces::action::LoopCartTraj;
// using panda_interfaces::action::StopTraj;
// using sensor_msgs::msg::JointState;
// using namespace std::chrono_literals;
// template <typename ActionT>
// using GoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;
// template <typename ActionT>
// using OptionalGoalHandle = typename std::optional<
//     typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>;
//
// enum class SceneState {
//   // The robot has to be configured
//   no_state,
//   // The robot is doing whichever task has to do
//   task,
//   // The human enters the robot area and the robot has to enter compliance
//   // mode
//   transition_human,
//   // The robot is in compliance mode: it can be freely moved by the human
//   compliance,
//   // The human leaves the robot area, allowing the robot to resume task
//   transition_leave_human,
//
// };
//
// void publish_state(
//     const rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pub,
//     const SceneState &state) {
//
//   std_msgs::msg::ColorRGBA color;
//
//   switch (state) {
//
//   case SceneState::no_state: {
//     color.r = 0.0;
//     color.g = 0.0;
//     color.b = 0.0;
//     break;
//   }
//   case SceneState::task: {
//     color.r = 0.0;
//     color.g = 255.0;
//     color.b = 0.0;
//     break;
//   }
//   case SceneState::transition_human: {
//     color.r = 255.0;
//     color.g = 255.0;
//     color.b = 0.0;
//     break;
//   }
//   case SceneState::compliance: {
//     color.r = 255.0;
//     color.g = 0.0;
//     color.b = 0.0;
//     break;
//   }
//   case SceneState::transition_leave_human: {
//     color.r = 0.0;
//     color.g = 255.0;
//     color.b = 255.0;
//     break;
//   } break;
//   }
//
//   pub->publish(color);
// }
//
// void fill_pose_orientation(geometry_msgs::msg::Pose &pose,
//                            const Eigen::Quaterniond &orientation) {
//
//   pose.orientation.w = orientation.w();
//   pose.orientation.x = orientation.x();
//   pose.orientation.y = orientation.y();
//   pose.orientation.z = orientation.z();
// }
//
// void fill_pose_position(geometry_msgs::msg::Pose &pose, const double &x,
//                         const double &y, const double &z) {
//
//   pose.position.x = x;
//   pose.position.y = y;
//   pose.position.z = z;
// }
//
// panda_interfaces::action::LoopCartTraj_Goal
// generate_triangle_task(double x, double y, double z, double h, double l,
//                        Eigen::Quaterniond orientation, double total_time) {
//
//   using geometry_msgs::msg::Pose;
//   panda_interfaces::action::LoopCartTraj_Goal goal;
//   Pose initial;
//   Pose right_corner;
//   Pose left_corner;
//
//   fill_pose_orientation(initial, orientation);
//   fill_pose_orientation(right_corner, orientation);
//   fill_pose_orientation(left_corner, orientation);
//
//   fill_pose_position(initial, x, y, z);
//   fill_pose_position(right_corner, x, y + l / 2, z - h);
//   fill_pose_position(left_corner, x, y - l / 2, z - h);
//
//   goal.desired_poses = std::vector{initial, right_corner, left_corner};
//   goal.total_time = total_time;
//   return goal;
// }
//
// void go_to_pose(
//     const rclcpp::Node &node, rclcpp_action::Client<CartTraj> &client,
//     const rclcpp_action::Client<CartTraj>::SendGoalOptions
//     &cart_traj_options, const panda_interfaces::action::CartTraj_Goal
//     &pose_goal) {}
//
// std::array<double, 7> home_joint_config;
// Pose home_pose;
// Pose initial_task_pose;
// double triangle_height; // h
// double triangle_base;   // l
// double total_task_time;
//
// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//
//   auto main_node = std::make_shared<rclcpp::Node>("task_node");
//   bool use_sim_time;
//
//   switch (argc) {
//   case 5: {
//     use_sim_time = (atoi(argv[1]) == 0) ? false : true;
//     RCLCPP_INFO_STREAM(main_node->get_logger(), "x: " << argv[2]
//                                                       << ", y: " << argv[3]
//                                                       << ", z: " << argv[4]);
//     home_pose.position.x = atof(argv[2]);
//     home_pose.position.y = atof(argv[3]);
//     home_pose.position.z = atof(argv[4]);
//     break;
//   }
//   default: {
//     RCLCPP_ERROR(main_node->get_logger(),
//                  "Usage: <use_sim_time> <x> <y> <z> home pose position");
//     rclcpp::shutdown();
//     return 0;
//   }
//   }
//   auto use_sim_time_param = rclcpp::Parameter("use_sim_time", use_sim_time);
//
//   auto sub_node = std::make_shared<rclcpp::Node>("task_sub_node");
//   sub_node->set_parameter(use_sim_time_param);
//   main_node->set_parameter(use_sim_time_param);
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(sub_node);
//   executor.add_node(main_node);
//
//   // Choosing home pose
//
//   home_pose.orientation.w = 0.0;
//   home_pose.orientation.x = 1.0;
//   home_pose.orientation.y = 0.0;
//   home_pose.orientation.z = 0.0;
//
//   geometry_msgs::msg::Pose initial_pose;
//   geometry_msgs::msg::Pose desired_pose;
//   panda_interfaces::msg::CartesianCommand cartesian_cmd;
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer;
//   SceneState state{SceneState::no_state};
//   human_presence::HumanPresentState presence_state;
//   std::atomic<bool> threads_run{false};
//   JointState joint_states;
//   PoseStamped pose_state;
//   // The goal handles for the 2 possible actions of robot, these needs to be
//   // stopped in any error or shutdown state
//   OptionalGoalHandle<CartTraj> cartesian_traj_handle;
//   OptionalGoalHandle<LoopCartTraj> loop_cartesian_traj_handle;
//   OptionalGoalHandle<StopTraj> stop_traj_handle;
//
//   // Goal options
//   GoalOptions<CartTraj> cart_traj_options;
//   GoalOptions<LoopCartTraj> loop_cart_traj_options;
//   GoalOptions<StopTraj> stop_traj_options;
//
//   RCLCPP_INFO(main_node->get_logger(), "Defining Home goal");
//   // Home goal definition
//   panda_interfaces::action::CartTraj_Goal home_goal;
//   home_goal.desired_pose = home_pose;
//   home_goal.total_time = 5.0;
//
//   // Stop trajectory goal definition
//   panda_interfaces::action::StopTraj_Goal stop_traj_goal;
//   stop_traj_goal.total_time = 1.5;
//
//   RCLCPP_INFO(main_node->get_logger(), "Defining Task goal: triangle");
//   // Task goal definition: triangle
//   Eigen::Quaterniond triangle_orient{
//       home_pose.orientation.w, home_pose.orientation.x,
//       home_pose.orientation.y, home_pose.orientation.z};
//   panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal =
//       generate_triangle_task(home_pose.position.x, home_pose.position.y,
//                              home_pose.position.z, 0.1, 0.2,
//                              triangle_orient.normalized(), 15.0);
//
//   // Topic to read for scene infos
//
//   auto joint_states_cb = [&joint_states](const JointState msg) {
//     joint_states = msg;
//   };
//
//   rclcpp::Subscription<JointState>::SharedPtr joint_states_sub =
//       sub_node->create_subscription<JointState>(
//           panda_interface_names::joint_state_topic_name,
//           panda_interface_names::CONTROLLER_PUBLISHER_QOS(),
//           joint_states_cb);
//
//   auto pose_state_cb = [&pose_state](const PoseStamped msg) {
//     pose_state = msg;
//   };
//
//   rclcpp::Subscription<PoseStamped>::SharedPtr pose_state_sub =
//       sub_node->create_subscription<PoseStamped>(
//           panda_interface_names::panda_pose_state_topic_name,
//           panda_interface_names::CONTROLLER_PUBLISHER_QOS(), pose_state_cb);
//
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_present_sub =
//       sub_node->create_subscription<std_msgs::msg::Bool>(
//           panda_interface_names::human_presence_topic,
//           panda_interface_names::DEFAULT_TOPIC_QOS(),
//           [&presence_state](const std_msgs::msg::Bool present) {
//             presence_state.human_present = present.data;
//           });
//
//   tf_buffer = std::make_unique<tf2_ros::Buffer>(sub_node->get_clock());
//
//   std::unique_ptr<tf2_ros::TransformListener> tf_listener =
//       std::make_unique<tf2_ros::TransformListener>(*tf_buffer, sub_node);
//
//   // Compliance mode service client
//   rclcpp::Client<panda_interfaces::srv::SetComplianceMode>::SharedPtr
//       compliance_mode_client =
//           main_node->create_client<panda_interfaces::srv::SetComplianceMode>(
//               panda_interface_names::set_compliance_mode_service_name);
//   panda_interfaces::srv::SetComplianceMode_Request compliance_request;
//
//   // Human presence service client
//   rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr human_presence_enabler =
//       main_node->create_client<std_srvs::srv::SetBool>(
//           panda_interface_names::enable_human_presence_service_name);
//   std_srvs::srv::SetBool_Request::SharedPtr enable_human_presence_request =
//       std::make_shared<std_srvs::srv::SetBool_Request>();
//
//   // Publisher for the update of desired commands when switching to
//   compliance
//   // mode
//
//   rclcpp::Publisher<panda_interfaces::msg::CartesianCommand>::SharedPtr
//       cartesian_cmd_pub =
//           main_node->create_publisher<panda_interfaces::msg::CartesianCommand>(
//               "/panda/cartesian_cmd",
//               panda_interface_names::DEFAULT_TOPIC_QOS());
//
//   rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr state_publisher =
//       main_node->create_publisher<std_msgs::msg::ColorRGBA>(
//           panda_interface_names::demo_state_topic_name,
//           panda_interface_names::DEFAULT_TOPIC_QOS());
//
//   auto update_state = [&state_publisher, &state]() {
//     publish_state(state_publisher, state);
//   };
//
//   auto send_current_pose_as_cmd = [&cartesian_cmd_pub,
//                                    &cartesian_cmd](const Pose desired_pose) {
//     Twist twist;
//     Accel accel;
//
//     twist.linear.x = 0.0;
//     twist.linear.y = 0.0;
//     twist.linear.z = 0.0;
//     twist.angular.x = 0.0;
//     twist.angular.y = 0.0;
//     twist.angular.z = 0.0;
//
//     accel.linear.x = 0.0;
//     accel.linear.y = 0.0;
//     accel.linear.z = 0.0;
//     accel.angular.x = 0.0;
//     accel.angular.y = 0.0;
//     accel.angular.z = 0.0;
//
//     cartesian_cmd.twist = twist;
//     cartesian_cmd.accel = accel;
//     cartesian_cmd.pose = desired_pose;
//
//     cartesian_cmd_pub->publish(cartesian_cmd);
//     // twist_cmd_pub->publish(twist);
//     // accel_cmd_pub->publish(accel);
//     // pose_cmd_pub->publish(desired_pose);
//   };
//
//   // Action clients for trajectories (actions)
//   rclcpp_action::Client<CartTraj>::SharedPtr cart_traj_action_client =
//       rclcpp_action::create_client<CartTraj>(
//           main_node, panda_interface_names::panda_cart_move_action_name);
//
//   rclcpp_action::Client<LoopCartTraj>::SharedPtr loop_traj_action_client =
//       rclcpp_action::create_client<LoopCartTraj>(
//           main_node, panda_interface_names::panda_cart_loop_action_name);
//
//   rclcpp_action::Client<StopTraj>::SharedPtr stop_traj_action_client =
//       rclcpp_action::create_client<StopTraj>(
//           main_node,
//           panda_interface_names::panda_exponential_stop_action_name);
//
//   rclcpp::Node::SharedPtr confirmation_node =
//       std::make_shared<rclcpp::Node>("actions_confirmation_node");
//
//   // Management of action nodes for the 2 types of trajectories
//   auto cancel_actions = [&cartesian_traj_handle, &loop_cartesian_traj_handle,
//                          &stop_traj_handle, &stop_traj_action_client,
//                          &cart_traj_action_client, &loop_traj_action_client,
//                          &main_node]() {
//     if (cartesian_traj_handle.has_value()) {
//       try {
//         cart_traj_action_client->async_cancel_goal(
//             cartesian_traj_handle.value());
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Requested cancel of cartesian trajectory action");
//       } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
//         RCLCPP_WARN_STREAM(main_node->get_logger(),
//                            "Goal handle has null value or terminated");
//       }
//     }
//
//     if (loop_cartesian_traj_handle.has_value()) {
//       try {
//         loop_traj_action_client->async_cancel_goal(
//             loop_cartesian_traj_handle.value());
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Requested cancel of loop cartesian trajectory action");
//       } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
//         RCLCPP_WARN_STREAM(main_node->get_logger(),
//                            "Goal handle has null value or terminated");
//       }
//     }
//
//     if (stop_traj_handle.has_value()) {
//       try {
//         stop_traj_action_client->async_cancel_goal(stop_traj_handle.value());
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Requested cancel of exponential stop action");
//       } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
//         RCLCPP_WARN_STREAM(main_node->get_logger(),
//                            "Goal handle has null value or terminated");
//       }
//     }
//   };
//
//   // Definition of SendGoalOptions struct
//   {
//     auto cart_result_callback =
//         [&cartesian_traj_handle](
//             const rclcpp_action::ClientGoalHandle<CartTraj>::WrappedResult &)
//             {
//           cartesian_traj_handle = std::nullopt;
//         };
//     cart_traj_options.result_callback = cart_result_callback;
//
//     auto loop_result_callback =
//         [&loop_cartesian_traj_handle](
//             const
//             rclcpp_action::ClientGoalHandle<LoopCartTraj>::WrappedResult
//                 &) { loop_cartesian_traj_handle = std::nullopt; };
//     loop_cart_traj_options.result_callback = loop_result_callback;
//
//     auto stop_result_callback =
//         [&stop_traj_handle, &main_node](
//             const rclcpp_action::ClientGoalHandle<StopTraj>::WrappedResult &)
//             {
//           stop_traj_handle = std::nullopt;
//         };
//     stop_traj_options.result_callback = stop_result_callback;
//   }
//
//   RCLCPP_INFO(main_node->get_logger(), "Waiting for servers...");
//
//   RCLCPP_INFO(main_node->get_logger(), "Waiting for cartesian trajectory");
//   cart_traj_action_client->wait_for_action_server(10s);
//
//   RCLCPP_INFO(main_node->get_logger(), "Waiting for loop cartesian
//   trajectory"); loop_traj_action_client->wait_for_action_server(10s);
//
//   RCLCPP_INFO(main_node->get_logger(),
//               "Waiting for exponential stop trajectory");
//   stop_traj_action_client->wait_for_action_server(10s);
//
//   RCLCPP_INFO(main_node->get_logger(), "Waiting for compliance mode server");
//   compliance_mode_client->wait_for_service(10s);
//
//   RCLCPP_INFO(main_node->get_logger(), "Waiting for human presence server");
//   human_presence_enabler->wait_for_service(10s);
//
//   RCLCPP_INFO(main_node->get_logger(), "Servers UP");
//
//   // Go to initial pose
//   // Begin task: cancel when a human enters the area
//   // Enter in compliance mode: exit when human leaves the area, staying in
//   // current pose Go back to last task state Resume task
//
//   std::thread info_thread([&executor]() { executor.spin(); });
//
//   std::thread safe_keeper_thread{[cancel_actions, &main_node,
//   &presence_state,
//                                   &state, &threads_run]() {
//     while (!threads_run.load()) {
//       std::this_thread::sleep_for(1s);
//     }
//
//     RCLCPP_INFO_STREAM(main_node->get_logger(), "Started thread
//     'safe_keeper'");
//
//     while (threads_run.load()) {
//       {
//         if (presence_state.human_present &&
//             state != SceneState::transition_human &&
//             state != SceneState::compliance) {
//           RCLCPP_INFO_STREAM(
//               main_node->get_logger(),
//               "Human present in scene, cancelling actions and
//               transitioning");
//           cancel_actions();
//           state = SceneState::transition_human;
//         } else if (!presence_state.human_present &&
//                    state == SceneState::compliance) {
//           RCLCPP_INFO_STREAM(main_node->get_logger(),
//                              "Human left scene, transitioning");
//           state = SceneState::transition_leave_human;
//         }
//       }
//
//       std::this_thread::sleep_for(5ms);
//     }
//   }};
//
//   {
//     RCLCPP_INFO(main_node->get_logger(), "Press enter to start demo");
//     std::cin.ignore();
//     threads_run.store(true);
//     RCLCPP_INFO(main_node->get_logger(), "Threads running");
//   }
//
//   enable_human_presence_request->set__data(true);
//   human_presence_enabler->async_send_request(enable_human_presence_request);
//
//   auto start = rclcpp::Time(0, 0, main_node->get_clock()->get_clock_type());
//
//   // Resetting compliance mode if set in controller
//   // Firstly, update the desired pose and other commands to the read one
//   send_current_pose_as_cmd(pose_state.pose);
//   // Sleep to avoid that the robot passes in non compliance mode before
//   updating
//   // his desired state
//   std::this_thread::sleep_for(500ms);
//
//   compliance_request.cmd = false;
//   auto compliance_future = compliance_mode_client->async_send_request(
//       std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
//           compliance_request));
//   bool stopped = false;
//   while (!stopped && rclcpp::ok()) {
//     auto res = compliance_future.wait_for(10ms);
//     if (res == std::future_status::ready) {
//       RCLCPP_INFO(main_node->get_logger(), "Robot in non compliance mode");
//       stopped = true;
//     } else {
//       RCLCPP_INFO(main_node->get_logger(), "Robot exiting compliance mode");
//     }
//   }
//
//   update_state();
//
//   while (rclcpp::ok()) {
//     switch (state) {
//     case SceneState::no_state: {
//
//       update_state();
//
//       // Ensure the robot is up and running
//       // Going to pose
//       RCLCPP_INFO(main_node->get_logger(), "Robot has no known state");
//
//       if (!cartesian_traj_handle.has_value()) {
//
//         if (!rclcpp::ok()) {
//           break;
//         }
//
//         RCLCPP_INFO(main_node->get_logger(), "Sending robot home pose");
//         RCLCPP_INFO(main_node->get_logger(), "Waiting for goal
//         confirmation"); auto future =
//         cart_traj_action_client->async_send_goal(
//             home_goal, cart_traj_options);
//         auto fut_return =
//             rclcpp::spin_until_future_complete(confirmation_node, future,
//             3s);
//         switch (fut_return) {
//         case rclcpp::FutureReturnCode::SUCCESS: {
//           auto handle = future.get();
//           if (handle) {
//             cartesian_traj_handle = handle;
//             RCLCPP_INFO(main_node->get_logger(),
//                         "Cartesian trajectory accepted");
//           } else {
//             RCLCPP_INFO(main_node->get_logger(),
//                         "Cartesian trajectory refused");
//             break;
//           }
//           break;
//         }
//         case rclcpp::FutureReturnCode::INTERRUPTED: {
//           RCLCPP_ERROR(main_node->get_logger(),
//                        "Cartesian trajectory interrupted");
//           break;
//         }
//         case rclcpp::FutureReturnCode::TIMEOUT: {
//           RCLCPP_ERROR(main_node->get_logger(),
//                        "Cartesian trajectory went timeout");
//           break;
//         } break;
//         };
//
//         while (cartesian_traj_handle.has_value()) {
//           RCLCPP_INFO(main_node->get_logger(),
//                       "Waiting robot to reach home pose");
//           if (!rclcpp::ok()) {
//             break;
//           }
//           rclcpp::sleep_for(500ms);
//         }
//
//         // Check if action has been interrupted from the human entering the
//         area if (state != SceneState::no_state || !rclcpp::ok()) {
//           break;
//         }
//
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Robot reached home pose, transitioning to task state");
//         state = SceneState::task;
//         break;
//       }
//       break;
//     }
//     case SceneState::task: {
//
//       update_state();
//
//       if (!loop_cartesian_traj_handle.has_value()) {
//         auto future = loop_traj_action_client->async_send_goal(
//             triangle_task_goal, loop_cart_traj_options);
//         auto fut_return =
//             rclcpp::spin_until_future_complete(confirmation_node, future,
//             3s);
//         switch (fut_return) {
//         case rclcpp::FutureReturnCode::SUCCESS: {
//           auto handle = future.get();
//           if (handle) {
//             loop_cartesian_traj_handle = handle;
//             RCLCPP_INFO(main_node->get_logger(),
//                         "Loop cartesian trajectory accepted");
//             start = main_node->now();
//           } else {
//             RCLCPP_INFO(main_node->get_logger(),
//                         "Loop cartesian trajectory refused");
//             break;
//           }
//           break;
//         }
//         case rclcpp::FutureReturnCode::INTERRUPTED: {
//           RCLCPP_ERROR(main_node->get_logger(),
//                        "Loop cartesian trajectory interrupted");
//           break;
//         }
//         case rclcpp::FutureReturnCode::TIMEOUT: {
//           RCLCPP_ERROR(main_node->get_logger(),
//                        "Loop cartesian trajectory went timeout");
//           break;
//         } break;
//         };
//       }
//
//       break;
//       // Read the human_state and handle the task accordingly; when human
//       // enter
//       // -> go to transition_human
//     }
//     case SceneState::transition_human: {
//
//       update_state();
//
//       //
//       // Stop the task, save the state and enter compliance mode -> go to
//       // compliance
//       //
//       RCLCPP_INFO(main_node->get_logger(), "In transition_human state");
//       // Issue exponential decay of velocity and acceleration at current read
//       // pose
//
//       RCLCPP_INFO(main_node->get_logger(),
//                   "Sending velocity and acceleration to 0 exponentially");
//       auto future = stop_traj_action_client->async_send_goal(stop_traj_goal,
//                                                              stop_traj_options);
//       auto fut_return =
//           rclcpp::spin_until_future_complete(confirmation_node, future,
//           500ms);
//       switch (fut_return) {
//       case rclcpp::FutureReturnCode::SUCCESS: {
//         auto handle = future.get();
//         if (handle) {
//           stop_traj_handle = handle;
//           RCLCPP_INFO(main_node->get_logger(), "Stop trajectory accepted");
//         } else {
//           RCLCPP_INFO(main_node->get_logger(), "Stop trajectory refused");
//           break;
//         }
//         break;
//       }
//       case rclcpp::FutureReturnCode::INTERRUPTED: {
//         RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory interrupted");
//         break;
//       }
//       case rclcpp::FutureReturnCode::TIMEOUT: {
//         RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory went
//         timeout"); break;
//       } break;
//       };
//
//       while (stop_traj_handle.has_value()) {
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Waiting robot to reach 0 velocity and acceleration");
//         if (!rclcpp::ok()) {
//           break;
//         }
//         rclcpp::sleep_for(500ms);
//       }
//
//       RCLCPP_INFO(main_node->get_logger(), "Stop trajectory succeded");
//
//       // Check if action has been interrupted from the human entering the
//       area if (state != SceneState::transition_human || !rclcpp::ok()) {
//         break;
//       }
//
//       compliance_request.cmd = true;
//       auto compliance_future = compliance_mode_client->async_send_request(
//           std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
//               compliance_request));
//       bool stopped = false;
//       while (!stopped) {
//
//         auto res = compliance_future.wait_for(10ms);
//         while (res != std::future_status::ready && rclcpp::ok()) {
//           res = compliance_future.wait_for(10ms);
//           RCLCPP_INFO(main_node->get_logger(),
//                       "Robot entering compliance mode");
//         }
//         if (res == std::future_status::ready) {
//           RCLCPP_INFO(main_node->get_logger(),
//                       "Robot stopped and passed in compliance mode");
//           stopped = true;
//           state = SceneState::compliance;
//         }
//       }
//       break;
//     }
//     case SceneState::compliance: {
//       update_state();
//       RCLCPP_INFO(main_node->get_logger(), "In compliance state");
//       std::this_thread::sleep_for(2s);
//       break;
//       // Read the human_state and handle the compliance mode accordingly;
//       when
//       // human leaves -> go to transition_leave_human
//     }
//     case SceneState::transition_leave_human: {
//
//       update_state();
//       RCLCPP_INFO(main_node->get_logger(), "In transition_leave_human
//       state");
//
//       send_current_pose_as_cmd(pose_state.pose);
//
//       compliance_request.cmd = false;
//       auto compliance_future = compliance_mode_client->async_send_request(
//           std::make_shared<panda_interfaces::srv::SetComplianceMode_Request>(
//               compliance_request));
//       bool stopped = false;
//       while (!stopped) {
//
//         auto res = compliance_future.wait_for(10ms);
//         if (res == std::future_status::ready && rclcpp::ok()) {
//           RCLCPP_INFO(main_node->get_logger(),
//                       "Robot passed in non compliance mode");
//           stopped = true;
//           state = SceneState::task;
//         } else {
//           RCLCPP_INFO(main_node->get_logger(), "Robot exiting compliance
//           mode"); if (!rclcpp::ok()) {
//             break;
//           }
//         }
//       }
//
//       RCLCPP_INFO(main_node->get_logger(), "Sending robot home pose");
//       RCLCPP_INFO(main_node->get_logger(), "Waiting for goal confirmation");
//       auto future = cart_traj_action_client->async_send_goal(home_goal,
//                                                              cart_traj_options);
//       auto fut_return =
//           rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
//       switch (fut_return) {
//       case rclcpp::FutureReturnCode::SUCCESS: {
//         auto handle = future.get();
//         if (handle) {
//           cartesian_traj_handle = handle;
//           RCLCPP_INFO(main_node->get_logger(), "Cartesian trajectory
//           accepted");
//         } else {
//           RCLCPP_INFO(main_node->get_logger(), "Cartesian trajectory
//           refused"); break;
//         }
//         break;
//       }
//       case rclcpp::FutureReturnCode::INTERRUPTED: {
//         RCLCPP_ERROR(main_node->get_logger(),
//                      "Cartesian trajectory interrupted");
//         break;
//       }
//       case rclcpp::FutureReturnCode::TIMEOUT: {
//         RCLCPP_ERROR(main_node->get_logger(),
//                      "Cartesian trajectory went timeout");
//         break;
//       } break;
//       };
//
//       while (cartesian_traj_handle.has_value()) {
//         RCLCPP_INFO(main_node->get_logger(),
//                     "Waiting robot to reach home pose");
//         if (!rclcpp::ok()) {
//           break;
//         }
//         rclcpp::sleep_for(2s);
//       }
//
//       // Check if action has been interrupted from the human entering the
//       area if (state != SceneState::transition_leave_human || !rclcpp::ok())
//       {
//         break;
//       }
//
//       rclcpp::sleep_for(2s);
//
//       RCLCPP_INFO(main_node->get_logger(), "Transitioning to task state");
//       state = SceneState::task;
//       break;
//       // Exit from compliance mode staying in current pose, return to last
//       // state pose, resume the task -> go to task
//     } break;
//     }
//   }
//   RCLCPP_INFO(main_node->get_logger(), "Requested shutdown");
//
//   cancel_actions();
//   RCLCPP_INFO(main_node->get_logger(),
//               "Sending velocity and acceleration to 0 exponentially");
//   auto future = stop_traj_action_client->async_send_goal(stop_traj_goal,
//                                                          stop_traj_options);
//   auto fut_return =
//       rclcpp::spin_until_future_complete(confirmation_node, future, 3s);
//   switch (fut_return) {
//   case rclcpp::FutureReturnCode::SUCCESS: {
//     auto handle = future.get();
//     if (handle) {
//       stop_traj_handle = handle;
//       RCLCPP_INFO(main_node->get_logger(), "Stop trajectory accepted");
//     } else {
//       RCLCPP_INFO(main_node->get_logger(), "Stop trajectory refused");
//       break;
//     }
//     break;
//   }
//   case rclcpp::FutureReturnCode::INTERRUPTED: {
//     RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory interrupted");
//     break;
//   }
//   case rclcpp::FutureReturnCode::TIMEOUT: {
//     RCLCPP_ERROR(main_node->get_logger(), "Stop trajectory went timeout");
//     break;
//   } break;
//   };
//
//   while (stop_traj_handle.has_value()) {
//     RCLCPP_INFO(main_node->get_logger(),
//                 "Waiting robot to reach 0 velocity and acceleration");
//     if (!rclcpp::ok()) {
//       break;
//     }
//     rclcpp::sleep_for(2s);
//   }
//   executor.cancel();
//   threads_run.store(false);
//   info_thread.join();
//   safe_keeper_thread.join();
//
//   rclcpp::shutdown();
// }
