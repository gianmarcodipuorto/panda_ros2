#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <memory>
#include <optional>
#include <rcl/time.h>
#include <rclcpp/client.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;
template <typename ActionT> using ActionServer = rclcpp_action::Server<ActionT>;
template <typename ActionT> using ActionClient = rclcpp_action::Client<ActionT>;

using TrajMove = panda_interfaces::action::CartTraj;
using TrajGoal = panda_interfaces::action::CartTraj_Goal;
using TrajMoveGoalHandle = rclcpp_action::ServerGoalHandle<TrajMove>;
using ClientTrajMoveGoalHandle = rclcpp_action::ClientGoalHandle<TrajMove>;

using TrajLoop = panda_interfaces::action::LoopCartTraj;
using TrajLoopGoal = panda_interfaces::action::LoopCartTraj_Goal;
using TrajLoopGoalHandle = rclcpp_action::ServerGoalHandle<TrajLoop>;

using namespace std::chrono_literals;

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort;

class ControllerManager : public rclcpp_lifecycle::LifecycleNode {

public:
  ControllerManager()
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::controller_manager_node_name) {

    // Cartesian trajectory move action server definition
    //
    auto handle_goal_traj_move = [this](const rclcpp_action::GoalUUID uuid,
                                        std::shared_ptr<const TrajGoal>) {
      RCLCPP_INFO(this->get_logger(),
                  "Received goal request for Cartesian trajectory movement");

      (void)uuid;

      if (trajectory_in_progress.load()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "There's another trajectory in progress");
        return rclcpp_action::GoalResponse::REJECT;
      }

      // Call action server and pass callback to re publish feedbacks to the
      // client

      // configure_clik();
      //
      // if (get_state(panda_interface_names::clik_node_name) !=
      //     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      //   RCLCPP_ERROR(this->get_logger(), "Clik node not configured");
      //   return rclcpp_action::GoalResponse::REJECT;
      // }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel_traj_move =
        [this](const std::shared_ptr<TrajMoveGoalHandle> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

          // Stop internal action server
          cart_traj_client->async_cancel_goal(
              cart_traj_internal_handler.value());

          // // Stop clik node
          // change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
          //              panda_interface_names::clik_node_name);

          // Update trajectory_in_progress flag
          trajectory_in_progress.store(false);

          RCLCPP_INFO(this->get_logger(),
                      "Keeping robot in last reached configuration");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted_traj_move =
        [this](const std::shared_ptr<TrajMoveGoalHandle> goal_handle) {
          using namespace std::placeholders;

          // change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
          //              panda_interface_names::clik_node_name);
          // Start trajectory loop control thread
          trajectory_in_progress.store(true);
          action_thread = std::thread{
              std::bind(&ControllerManager::control_traj_move, this, _1),
              goal_handle};
        };

    cart_traj_server = rclcpp_action::create_server<TrajMove>(
        this, panda_interface_names::panda_cart_move_action_name,
        handle_goal_traj_move, handle_cancel_traj_move,
        handle_accepted_traj_move);

    ////////////////////////////////////////////////////////////////////////

    // Action client calling the actual action server nodes
    //
    cart_traj_client =
        rclcpp_action::create_client<panda_interfaces::action::CartTraj>(
            this, panda_interface_names::cart_traj_node_name +
                      std::string{"/"} +
                      panda_interface_names::panda_cart_move_action_name);

    loop_traj_client =
        rclcpp_action::create_client<panda_interfaces::action::LoopCartTraj>(
            this, panda_interface_names::panda_cart_loop_action_name);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Configuring...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Activating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");

    return CallbackReturn::SUCCESS;
  }

private:
  // Action servers
  ActionServer<panda_interfaces::action::CartTraj>::SharedPtr cart_traj_server;
  ActionServer<panda_interfaces::action::LoopCartTraj>::SharedPtr
      loop_cart_traj_server;

  ActionClient<panda_interfaces::action::CartTraj>::SharedPtr cart_traj_client;
  std::optional<ClientTrajMoveGoalHandle::SharedPtr>
      cart_traj_internal_handler{};
  ActionClient<panda_interfaces::action::LoopCartTraj>::SharedPtr
      loop_traj_client;
  std::optional<rclcpp_action::GoalUUID> loop_traj_goal_uuid{};

  // Thread related variables
  std::atomic<bool> trajectory_in_progress{false};
  std::thread action_thread{};

  void control_traj_move(const std::shared_ptr<TrajMoveGoalHandle> goal_handle);
  void control_traj_loop(const std::shared_ptr<TrajLoopGoalHandle> goal_handle);

  bool change_state(std::uint8_t transition, const std::string &node_name,
                    std::chrono::seconds time_out = 3s) {
    auto lifecycle_client_node =
        std::make_shared<rclcpp::Node>("lifecycle_client_node");
    auto client =
        lifecycle_client_node->create_client<lifecycle_msgs::srv::ChangeState>(
            node_name + "/change_state");
    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.

    if (rclcpp::spin_until_future_complete(lifecycle_client_node,
                                           future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Service"
                                                  << client->get_service_name()
                                                  << " not responding");
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(get_logger(),
                  "Transition %d successfully triggered for service %s",
                  static_cast<int>(transition), client->get_service_name());
      return true;
    } else {
      RCLCPP_WARN(
          get_logger(), "Failed to trigger transition %u for service %s",
          static_cast<unsigned int>(transition), client->get_service_name());
      return false;
    }
  }

  unsigned int get_state(const std::string &node_name,
                         std::chrono::seconds time_out = 3s) {
    auto lifecycle_client_node =
        std::make_shared<rclcpp::Node>("lifecycle_client_node");
    auto client_get =
        lifecycle_client_node->create_client<lifecycle_msgs::srv::GetState>(
            node_name + "/get_state");
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client_get->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = client_get->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    if (rclcpp::spin_until_future_complete(lifecycle_client_node,
                                           future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Unknown service " << client_get->get_service_name()
                                             << " state");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(get_logger(), "Service %s has current state %s.",
                  client_get->get_service_name(),
                  future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state for service %s",
                   client_get->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  void configure_clik() {
    RCLCPP_INFO(this->get_logger(), "Configuring CLIK node");
    if (get_state(panda_interface_names::clik_node_name) !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                   panda_interface_names::clik_node_name);
    } else {
      change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                   panda_interface_names::clik_node_name);
      change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                   panda_interface_names::clik_node_name);
    }
  }
};

void ControllerManager::control_traj_move(
    const std::shared_ptr<TrajMoveGoalHandle> goal_handle) {

  auto node = std::make_shared<rclcpp::Node>("cart_traj_bridge_node");

  auto goal_msg = goal_handle->get_goal();

  // Wait for the underlying server
  if (!cart_traj_client->wait_for_action_server(std::chrono::seconds(5))) {
    goal_handle->abort(std::make_shared<TrajMove::Result>());
    return;
  }

  // Send goal
  auto send_goal_options = rclcpp_action::Client<TrajMove>::SendGoalOptions();

  // Republish feedback to caller via the server
  send_goal_options.feedback_callback =
      [goal_handle](ClientTrajMoveGoalHandle::SharedPtr,
                    const std::shared_ptr<const TrajMove::Feedback> feedback) {
        goal_handle->publish_feedback(
            std::const_pointer_cast<TrajMove::Feedback>(feedback));
      };

  send_goal_options.result_callback = [this, goal_handle](const auto &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal_handle->succeed(result.result);
    } else {
      goal_handle->abort(result.result);
    }
  };

  auto future_goal_handle =
      cart_traj_client->async_send_goal(*goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    goal_handle->abort(std::make_shared<TrajMove::Result>());
    return;
  }

  rclcpp_action::ClientGoalHandle<TrajMove>::SharedPtr client_goal_handle =
      future_goal_handle.get();
  if (!client_goal_handle) {
    goal_handle->abort(std::make_shared<TrajMove::Result>());
    return;
  }

  cart_traj_internal_handler = client_goal_handle;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
