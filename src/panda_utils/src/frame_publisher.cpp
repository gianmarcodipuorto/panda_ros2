#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp" // For Eigen to geometry_msgs conversions
#include "tf2_ros/transform_broadcaster.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class FramePublisher : public rclcpp::Node {
public:
  FramePublisher()
      : rclcpp::Node("frame_publisher_node"), publishing_enabled_(false) {
    // Declare and get parameters
    this->declare_parameter<double>("publish_rate", 100.0);
    this->declare_parameter<std::string>("robot_base_frame_id", "fr3_link0");

    double publish_rate = this->get_parameter("publish_rate").as_double();
    robot_base_frame_id_ =
        this->get_parameter("robot_base_frame_id").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Service to enable/disable publishing
    enable_publishing_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/set_publishing_pose_frames",
        std::bind(&FramePublisher::enablePublishingServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Subscription to receive an array of poses to publish
    // The message is expected to contain an array of poses and their
    // corresponding frame_ids.
    poses_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseArray>(
            panda_interface_names::panda_frame_poses_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS(),
            std::bind(&FramePublisher::posesCallback, this,
                      std::placeholders::_1));

    // Create a timer for publishing frames at the specified rate
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&FramePublisher::publishFrames, this));
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "FramePublisher node initialized. Publishing rate: %.2f Hz, "
                "Base Frame: %s",
                publish_rate, robot_base_frame_id_.c_str());
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_publishing_service_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      poses_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> publishing_enabled_;
  std::string robot_base_frame_id_;

  // Data structures to hold received poses and their names, protected by a
  // mutex.
  std::vector<geometry_msgs::msg::Pose> poses_to_publish_;
  std::vector<std::string> frame_ids_to_publish_;
  std::mutex poses_mutex_;

  // Callback for the SetBool service to enable or disable TF publishing.
  void enablePublishingServiceCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    publishing_enabled_.store(request->data);
    response->success = true;
    response->message = publishing_enabled_.load()
                            ? "Frame publishing enabled"
                            : "Frame publishing disabled";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    if (publishing_enabled_.load()) {
      timer_->reset();
    } else {
      timer_->cancel();
    }
  }

  // Callback for the subscription to an array of poses.
  void posesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_to_publish_ = msg->poses;
    frame_ids_to_publish_ = panda_interface_names::panda_link_names;
    RCLCPP_DEBUG(this->get_logger(), "Received %zu poses to publish.",
                 poses_to_publish_.size());
  }

  // Timer callback to publish the frames.
  void publishFrames() {
    if (!publishing_enabled_.load()) {
      return;
    }

    std::lock_guard<std::mutex> lock(poses_mutex_);
    if (poses_to_publish_.empty()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "No poses currently available to publish.");
      return;
    }

    rclcpp::Time now = this->now();
    std::string prev_frame = robot_base_frame_id_;
    tf2::Transform prev_abs_tf = tf2::Transform::getIdentity();

    for (size_t i = 0; i < poses_to_publish_.size(); ++i) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = now;
      transform_stamped.header.frame_id = prev_frame;
      transform_stamped.child_frame_id =
          frame_ids_to_publish_[i + 1]; // First one is fr3_link0

      tf2::Transform current_abs_tf;
      tf2::fromMsg(poses_to_publish_[i], current_abs_tf);

      tf2::Transform relative_tf = prev_abs_tf.inverse() * current_abs_tf;
      // Convert geometry_msgs::msg::Pose to geometry_msgs::msg::Transform
      transform_stamped.transform = tf2::toMsg(relative_tf);
      tf_broadcaster_->sendTransform(transform_stamped);

      prev_abs_tf = current_abs_tf;
      prev_frame = frame_ids_to_publish_[i + 1];
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
