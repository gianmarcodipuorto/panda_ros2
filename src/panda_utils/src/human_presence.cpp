#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_processing/constants.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/src/Core/PartialReduxEvaluator.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>
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
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
class HumanPresence : public rclcpp::Node {
public:
  HumanPresence() : Node("human_presence") {
    // Parameter declarations
    this->declare_parameter("robot_base_frame_name", "panda_link0");

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create the service to activate/deactivate the timer
    activate_timer_service_ = this->create_service<std_srvs::srv::SetBool>(
        panda_interface_names::enable_human_presence_service_name,
        std::bind(&HumanPresence::activate_timer_service_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create the timer (initially not active, will be activated by service)
    // The timer period is 10ms as in the original thread sleep_for
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&HumanPresence::timer_callback, this));
    timer_->cancel(); // Start cancelled
    //
    human_present_pub = this->create_publisher<std_msgs::msg::Bool>(
        panda_interface_names::human_presence_topic,
        panda_interface_names::DEFAULT_TOPIC_QOS());

    RCLCPP_INFO(this->get_logger(), "HumanPresence node initialized. Timer "
                                    "can be activated via "
                                    "/human_presence_timer_activate service.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_timer_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_present_pub;

  std_msgs::msg::Bool human_presence;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string robot_base_frame_name =
      panda_interface_names::panda_link_names.front();

  human_presence::HumanPresentState
      presence_state_; // Member variable for the state

  void timer_callback() {
    std::map<std::string, geometry_msgs::msg::TransformStamped>
        robot_links_body_keypoints_tfs;

    rclcpp::Time last_time{this->get_clock()->now()};
    rclcpp::Time last_time_print{this->get_clock()->now()}; // Only used for
    // a print throttle in original thread, removed.

    bool print_every_half_sec =
        (this->get_clock()->now() - last_time_print).seconds() > 0.5; // Only
    //     used for a print throttle in original thread, removed.

    if (tf_buffer_->canTransform(robot_base_frame_name, "world",
                                 tf2::TimePointZero)) {
      // Robot base exists, now get the transforms of the body
      // keypoints wrt the base
      auto now = this->get_clock()->now();
      // Clear the map to avoid getting false data
      robot_links_body_keypoints_tfs.clear();
      // Cycle through all the body keypoints frame: get the
      // transform
      // if the body frame is relatively new in the tf2 system and
      // obvoiusly exists
      for (auto keypoint_frame_name : image_constants::coco_keypoints) {
        try {
          auto keypoint_frame = tf_buffer_->lookupTransform(
              keypoint_frame_name, "world", tf2::TimePointZero);

          if (now - keypoint_frame.header.stamp >= max_tf_age) {
            continue;
          }
        } catch (const tf2::TransformException &ex) {
          RCLCPP_ERROR_STREAM_THROTTLE(
              this->get_logger(), *this->get_clock(), 10000.0,
              "Keypoint transform not found: " << ex.what());
          continue;
        }

        for (auto robot_frame_name : panda_interface_names::panda_link_names) {
          try {
            auto tf = tf_buffer_->lookupTransform(
                robot_frame_name, keypoint_frame_name, tf2::TimePointZero);
            robot_links_body_keypoints_tfs[robot_frame_name + "_to_" +
                                           keypoint_frame_name] = tf;
          } catch (const tf2::TransformException &ex) {
            // Tf does not exists
          }
        }
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
      // Increase the time the human have been seen inside the area
      // or decrease if he's not there
      if (human_in_area) {
        presence_state_.time_present += (this->get_clock()->now() - last_time);
      } else {
        presence_state_.time_present -= (this->get_clock()->now() - last_time);
      }
      // Normalize the time according to limit
      presence_state_.normalize_time();
      if (human_in_area &&
          presence_state_.time_present.seconds() >= presence_state_.MAX_TIME &&
          !presence_state_.human_present) {
        // If human is inside by max time at least then he's
        // surely in area
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "The human entered the robot area");
        presence_state_.human_present = true;
      } else if (!human_in_area &&
                 presence_state_.time_present.seconds() == 0.0 &&
                 presence_state_.human_present) {
        RCLCPP_INFO_STREAM(this->get_logger(), "The human left the robot area");
        presence_state_.human_present = false;
      }

      if (presence_state_.human_present) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "HUMAN PRESENT IN THE AREA");
      }

      human_presence.data = presence_state_.human_present;
      human_present_pub->publish(human_presence);
      last_time = this->now();

      rclcpp::sleep_for(100ms);

    }
  }

  void activate_timer_service_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
      // Try to activate the timer
      if (!timer_->is_canceled()) {
        response->success = false;
        response->message = "Timer is already active.";
        RCLCPP_WARN(this->get_logger(),
                    "Attempted to activate an already active timer.");
      } else {
        timer_->reset(); // Start the timer
        response->success = true;
        response->message = "Human presence timer activated.";
        RCLCPP_INFO(this->get_logger(), "Human presence timer activated.");
      }
    } else {
      // Try to deactivate the timer
      if (timer_->is_canceled()) {
        response->success = false;
        response->message = "Timer is already deactivated.";
        RCLCPP_WARN(this->get_logger(),
                    "Attempted to deactivate an already deactivated timer.");
      } else {
        timer_->cancel(); // Stop the timer
        response->success = true;
        response->message = "Human presence timer deactivated.";
        RCLCPP_INFO(this->get_logger(), "Human presence timer deactivated.");
      }
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanPresence>());
  rclcpp::shutdown();
  return 0;
}
