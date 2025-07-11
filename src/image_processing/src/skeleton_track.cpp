#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_processing/constants.hpp"
#include "image_processing/skeleton_infer.hpp"
#include "image_processing/utils.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <boost/fusion/sequence/intrinsic_fwd.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/video/tracking.hpp>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <utility>

template <typename message> using Subscription = rclcpp::Subscription<message>;
using sensor_msgs::msg::Image;

using namespace std::chrono_literals;

template <typename type, int STATES, int OUTPUTS> struct kalman_filt_params {
  // Noise process matrix
  Eigen::Matrix<type, STATES, STATES> Q;
  // Noise output matrix
  Eigen::Matrix<type, OUTPUTS, OUTPUTS> R;
  // State transition matrix
  Eigen::Matrix<type, STATES, STATES> F;
  // Output matrix
  Eigen::Matrix<type, OUTPUTS, STATES> H;
};

template <typename type, int STATES> struct state {
  Eigen::Vector<type, STATES> internal_state;
  // Prediction matrix
  Eigen::Matrix<type, STATES, STATES> P;
  bool is_initialized;

  void init(Eigen::Vector<type, STATES> init_state,
            Eigen::Matrix<type, STATES, STATES> init_P) {
    is_initialized = true;
    internal_state = init_state;
    P = init_P;
  };

  void deinit() {
    is_initialized = false;
    internal_state.setZero();
  }
};

enum class TargetState {
  // The Skeleton tracker does not track any person inside the scene, all the
  // states are deinitialized
  NO_PERSON,
  // The skeleton tracker is tracking a person consistently inside the scene,
  // the states are up and running
  PERSON_TRACKED,
  // The skeleton tracker does not have a good confidence in the person
  // previously tracked, due to lost keypoints or obstacle in scene obstructing
  // the person. The tracking continues with some criteria, till the person is
  // completely lost
  PERSON_LOST,
};

struct StateTransitionParams {
  // Minimum number of frames to switch in tracking mode
  int no_person_to_tracked_frames = 2;
  // Minimum number of valid keypoints to start tracking
  int min_tracked_valid_keypoints = 7;
  // Minimum number of frames to lose the tracking
  int tracked_to_lost_frames = 3;
  // Minimum number of valid keypoints to keep someone tracked
  int min_keypoints_valid_lost = 5;
  // Maximum time the tracker can remain in lost mode
  double lost_time = 2.0;
  // Minimum confidence of valid keypoints to track a person
  double confidence_threshold = 0.4;
  // Minimum confidence to not consider a keypoint as an hallucination
  double hallucination_threshold = 0.2;
};

struct DecisionData {
  int valid_frames = 0;
  int non_valid_frames = 0;
  double mean_confidence = 0.0;
  int valid_keypoints = 0;
  rclcpp::Time lost_event_time{};
};

class SkeletonTracker : public rclcpp::Node {
public:
  SkeletonTracker(std::string model_path, const char *net_log_id,
                  std::vector<int64_t> net_tensor_shape, double min_depth,
                  double max_depth)
      : Node("skeleton_tracker"), net(model_path, net_log_id, net_tensor_shape),
        min_depth(min_depth), max_depth(max_depth) {

    this->declare_parameter<double>("process_noise", 1.0);
    this->declare_parameter<double>("measurement_noise", 0.4);
    this->declare_parameter<bool>("no_depth", false);

    // Initializing kalman filter variables
    kalman_params.Q.setIdentity();
    kalman_params.Q *= this->get_parameter("process_noise").as_double();

    kalman_params.R.setIdentity();
    kalman_params.R *= this->get_parameter("measurement_noise").as_double();

    kalman_params.F.setIdentity();

    kalman_params.H.setZero();
    kalman_params.H(0, 0) = 1.0;
    kalman_params.H(1, 1) = 1.0;
    kalman_params.H(2, 2) = 1.0;

    num_landmarks = 17;

    filter_state.resize(num_landmarks);
    for (auto &state : filter_state) {
      state.P = kalman_params.Q;
      state.internal_state.setZero();
      state.is_initialized = false;
    }

    landmark_3d.resize(num_landmarks);
    for (geometry_msgs::msg::Point &point : landmark_3d) {
      point.x = 0.0;
      point.y = 0.0;
      point.z = 0.0;
    }

    transition_params.lost_time = 2.0;
    transition_params.no_person_to_tracked_frames = 2;
    transition_params.tracked_to_lost_frames = 3;
    transition_params.min_keypoints_valid_lost = 5;
    transition_params.min_tracked_valid_keypoints = 7;
    transition_params.confidence_threshold = 0.3;

    last_image_stamp = this->now();
    bool no_depth = this->get_parameter("no_depth").as_bool();

    auto rgb_img_cb = [this](const Image::SharedPtr img) {
      std::lock_guard<std::mutex> img_mutex(images_mutex);
      current_rgb_image = img;
      delta_time = this->now() - last_image_stamp;
      // RCLCPP_INFO_STREAM(this->get_logger(),
      //                    "Delta time: " << delta_time.seconds());
      // RCLCPP_INFO_STREAM(
      //     this->get_logger(),
      //     "Current image stamp: " << current_rgb_image->header.stamp.sec);
      last_image_stamp = this->now();
    };

    auto depth_img_cb = [this](const Image::SharedPtr img) {
      std::lock_guard<std::mutex> img_mutex(images_mutex);
      current_depth_image = img;
    };

    auto camera_info_cb = [this](const sensor_msgs::msg::CameraInfo msg) {
      current_camera_info = msg;
      image_geom.fromCameraInfo(current_camera_info);
    };

    rgb_image_sub = this->create_subscription<Image>(
        image_constants::rgb_image_topic, image_constants::DEFAULT_TOPIC_QOS,
        rgb_img_cb);

    std::string depth_image_topic;
    if (no_depth) {
      depth_image_topic = image_constants::rgb_image_topic;
    } else {
      depth_image_topic = image_constants::depth_image_topic;
    }

    depth_image_sub = this->create_subscription<Image>(
        depth_image_topic, image_constants::DEFAULT_TOPIC_QOS, depth_img_cb);

    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        image_constants::camera_info_topic, image_constants::DEFAULT_TOPIC_QOS,
        camera_info_cb);

    skeleton_image_pub =
        std::make_shared<realtime_tools::RealtimePublisher<Image>>(
            this->create_publisher<Image>(image_constants::skeleton_image_topic,
                                          image_constants::DEFAULT_TOPIC_QOS));

    tf_skel_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_cb = [this, no_depth]() {
      // If these two images are not nullptr
      if (current_rgb_image && current_depth_image) {
        auto start = std::chrono::high_resolution_clock::now();
        rclcpp::Time now = this->now();
        // Save current messages to not lose future messages
        Image rgb_img, depth_img;
        {
          std::lock_guard<std::mutex> img_mutex(images_mutex);
          rgb_img = *current_rgb_image;
          depth_img = *current_depth_image;
        }

        // 1) Get landmark on current image
        //
        cv_bridge::CvImagePtr rgb_img_mat =
            cv_bridge::toCvCopy(rgb_img, "rgb8");
        net.load_input(rgb_img_mat->image);
        net.run(false);
        landmarks = net.get_landmarks();
        std::vector<double> depths;
        if (no_depth) {
          // If no depth, create a vector of 1.0s. Don't call calculate_depths.
          depths.assign(num_landmarks, 1.0);
        } else {
          // Only call this if you have a real depth image
          depths = calculate_depths(cv_bridge::toCvCopy(depth_img)->image);
        }
        // std::vector<double> depths =
        //     calculate_depths(cv_bridge::toCvCopy(depth_img)->image);
        // if (no_depth) {
        //   for (auto &depth : depths) {
        //     depth = 1.0;
        //   }
        // }
        std::vector<std::optional<geometry_msgs::msg::Point>> points =
            project_pixels_to_3d(depths);

        // 2) Take decision based on current state
        //
        calc_decision_data(points);
        switch (tracking_state) {
        case TargetState::NO_PERSON: {

          if (tracking_decision_data.valid_frames >=
                  transition_params.no_person_to_tracked_frames &&
              tracking_decision_data.valid_keypoints >=
                  transition_params.min_tracked_valid_keypoints) {

            // switch to tracking person and initialize kalman filters
            //
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Switching to tracked person state");
            no_person_to_tracked(points);
            tracking_state = TargetState::PERSON_TRACKED;
            break;
          }
          break;
        }
        case TargetState::PERSON_TRACKED: {
          if (tracking_decision_data.mean_confidence <
                  transition_params.confidence_threshold ||
              tracking_decision_data.valid_keypoints <
                  transition_params.min_keypoints_valid_lost) {
            // switch to lost person and initialize the lost event time for lost
            // state
            this->kalman_predict(points, false, true);
            tracking_decision_data.lost_event_time = now;
            tracking_state = TargetState::PERSON_LOST;
            break;
          }
          // Init new kalman filter if keypoint is valid and run kalman update
          this->kalman_predict(points);
          break;
        }
        case TargetState::PERSON_LOST: {
          if ((now - tracking_decision_data.lost_event_time).seconds() >
              transition_params.lost_time) {
            // Switch to no tracked person state and deinitialize all the kalman
            // filters
            for (auto filt : filter_state) {
              filt.deinit();
            }
            tracking_state = TargetState::NO_PERSON;
            break;
          } else if (tracking_decision_data.mean_confidence >=
                     transition_params.confidence_threshold) {
            // Switch to tracked person state, probably there have been an
            // obstruction for some frames
            tracking_state = TargetState::PERSON_TRACKED;
            this->kalman_predict(points);
            break;
          }
          // Keep predict with kalman filters
          this->kalman_predict(points, false, true);
          break;
        }
        }
        switch (tracking_state) {

        case TargetState::NO_PERSON: {
          RCLCPP_INFO_STREAM(this->get_logger(), "No person tracked");
          break;
        }
        case TargetState::PERSON_TRACKED: {
          RCLCPP_INFO_STREAM(this->get_logger(), "Person tracked");
          break;
        }
        case TargetState::PERSON_LOST: {
          RCLCPP_INFO_STREAM(this->get_logger(), "Person lost");
          break;
        } break;
        }

        // 3) Publish infos to external network
        //
        if (tracking_state == TargetState::PERSON_LOST ||
            tracking_state == TargetState::PERSON_TRACKED) {
          this->publish_keypoints_tf();
        }
        skel_image_output =
            *(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8",
                                 this->create_skel_img(rgb_img))
                  .toImageMsg());
        skeleton_image_pub->tryPublish(skel_image_output.value());
        debug_print();

        // Set current images to nullptr
        current_rgb_image = nullptr;
        current_depth_image = nullptr;
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        // RCLCPP_INFO_STREAM(
        //     this->get_logger(),
        //     "Timer callback time passed: " << duration_ns.count() * 1e-9);
      }
    };

    timer = rclcpp::create_timer(this, this->get_clock(), 1ms, timer_cb);

    // DEBUG

    dbg_innovations.resize(num_landmarks, Eigen::Vector3d::Zero());
    dbg_uncertainties.resize(num_landmarks,
                             Eigen::Matrix<double, 6, 1>::Zero());
    dbg_innovation_pubs.resize(num_landmarks);
    dbg_uncertainty_pubs.resize(num_landmarks);
    dbg_state_position_pubs.resize(num_landmarks);
    dbg_state_velocity_pubs.resize(num_landmarks);

    dbg_mean_conf_pub =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "debug/mean_confidence", 10);

    dbg_all_confs_pub =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/all_confidences", 10);

    for (int i = 0; i < num_landmarks; ++i) {
      std::string topic_name_position =
          "debug/landmark_" + image_constants::coco_keypoints[i] + "/position";
      std::string topic_name_velocity =
          "debug/landmark_" + image_constants::coco_keypoints[i] + "/velocity";
      dbg_state_position_pubs[i] =
          this->create_publisher<geometry_msgs::msg::PointStamped>(
              topic_name_position, 10);
      dbg_state_velocity_pubs[i] =
          this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
              topic_name_velocity, 10);
    }

    for (int i = 0; i < num_landmarks; ++i) {
      std::string innovation_topic_name = "debug/landmark_" +
                                          image_constants::coco_keypoints[i] +
                                          "/innovation";
      dbg_innovation_pubs[i] =
          this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
              innovation_topic_name, 10);

      std::string uncertainty_topic_name = "debug/landmark_" +
                                           image_constants::coco_keypoints[i] +
                                           "/uncertainty";
      dbg_uncertainty_pubs[i] =
          this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
              uncertainty_topic_name, 10);
    }
  }

private:
  // Subscribers
  Subscription<Image>::SharedPtr rgb_image_sub;
  Subscription<Image>::SharedPtr depth_image_sub;
  Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

  // Publishers
  realtime_tools::RealtimePublisher<Image>::SharedPtr skeleton_image_pub{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_skel_publisher;
  rclcpp::TimerBase::SharedPtr timer;

  // DEBUG PUBLISHERS
  rclcpp::Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      dbg_mean_conf_pub;
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      dbg_all_confs_pub;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr>
      dbg_state_position_pubs;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
      dbg_state_velocity_pubs;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
      dbg_innovation_pubs;
  std::vector<
      rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr>
      dbg_uncertainty_pubs;

  // Image processing
  SkeletonInfer net;
  std::mutex images_mutex;
  Image::SharedPtr current_rgb_image;
  Image::SharedPtr current_depth_image;
  sensor_msgs::msg::CameraInfo current_camera_info;
  const std::string camera_frame = "/camera_link";
  double min_depth, max_depth;
  int num_landmarks;
  image_geometry::PinholeCameraModel image_geom;

  // Keypoints state variables
  std::optional<Image> skel_image_output;
  std::map<int, skeleton_utils::landmark> landmarks;
  std::vector<geometry_msgs::msg::Point> landmark_3d;

  TargetState tracking_state = TargetState::NO_PERSON;
  StateTransitionParams transition_params;
  DecisionData tracking_decision_data;
  kalman_filt_params<double, 6, 3> kalman_params;
  std::vector<state<double, 6>> filter_state;
  rclcpp::Time last_image_stamp;
  rclcpp::Duration delta_time{0, 0};

  // Debug
  std::vector<Eigen::Vector3d> dbg_innovations;
  std::vector<Eigen::Matrix<double, 6, 1>> dbg_uncertainties;

  cv::Mat create_skel_img(Image background_scene = Image());
  std::vector<std::optional<geometry_msgs::msg::Point>>
  project_pixels_to_3d(std::vector<double> depths);
  void publish_keypoints_tf();
  std::vector<double> calculate_depths(cv::Mat depth_image);
  void kalman_predict(
      const std::vector<std::optional<geometry_msgs::msg::Point>> &points,
      bool init_new = true, bool only_predict = false);
  void debug_print();

  void calc_decision_data(
      std::vector<std::optional<geometry_msgs::msg::Point>> points) {
    // Reset current valid keypoints number
    tracking_decision_data.valid_keypoints = 0;
    double conf_sum = 0.0;

    for (int i = 0; i < num_landmarks; i++) {
      auto point = points[i];
      auto landmark = landmarks[i];
      if (landmark.conf > transition_params.hallucination_threshold) {
        conf_sum += landmark.conf;
      }
      if (point.has_value() &&
          landmark.conf > transition_params.confidence_threshold) {
        tracking_decision_data.valid_keypoints += 1;
      }
    }

    tracking_decision_data.mean_confidence = conf_sum / num_landmarks;

    if (tracking_decision_data.mean_confidence >=
        transition_params.confidence_threshold) {
      tracking_decision_data.valid_frames += 1;
      tracking_decision_data.non_valid_frames = 0;
    } else {
      tracking_decision_data.valid_frames = 0;
      tracking_decision_data.non_valid_frames += 1;
    }

    // RCLCPP_INFO_STREAM(
    //     this->get_logger(),
    //     "Decision datas: "
    //         << "mean_conf = " << tracking_decision_data.mean_confidence
    //         << ", valid keypoints = " <<
    //         tracking_decision_data.valid_keypoints
    //         << ", valid frames = " << tracking_decision_data.valid_frames
    //         << ", non valid frames = "
    //         << tracking_decision_data.non_valid_frames);
  }
  // State transition methods
  void no_person_to_tracked(
      const std::vector<std::optional<geometry_msgs::msg::Point>> &points) {
    // There's good confidence in the last frames, init the kalman filters of
    // valid keypoints right now without running the update
    for (int i = 0; i < num_landmarks; i++) {
      auto point = points[i];
      auto landmark = landmarks[i];
      if (landmark.conf >= transition_params.confidence_threshold &&
          point.has_value()) {
        Eigen::Vector<double, 6> state;
        state.setZero();
        state.x() = point->x;
        state.y() = point->y;
        state.z() = point->z;
        filter_state[i].init(state, kalman_params.Q);
      }
    }
  }
};

void SkeletonTracker::debug_print() {
  if (landmarks.empty()) {
    return; // Nothing to publish if no landmarks were detected
  }
  auto now = this->now();
  auto confs_msg = panda_interfaces::msg::DoubleArrayStamped();
  confs_msg.header.stamp = now;

  for (const auto &pair : landmarks) {
    confs_msg.data.push_back(pair.second.conf);
  }

  dbg_all_confs_pub->publish(confs_msg);

  auto mean_conf_msg = panda_interfaces::msg::DoubleStamped();
  mean_conf_msg.header.stamp = now;
  mean_conf_msg.data = tracking_decision_data.mean_confidence;
  dbg_mean_conf_pub->publish(mean_conf_msg);

  for (int i = 0; i < num_landmarks; ++i) {
    auto point_msg = geometry_msgs::msg::PointStamped();
    auto velocity_msg = geometry_msgs::msg::Vector3Stamped();
    point_msg.header.stamp = now;
    velocity_msg.header.stamp = now;

    point_msg.header.frame_id = camera_frame;
    velocity_msg.header.frame_id = camera_frame;

    point_msg.point.x = filter_state[i].internal_state(0);
    point_msg.point.y = filter_state[i].internal_state(1);
    point_msg.point.z = filter_state[i].internal_state(2);

    velocity_msg.vector.x = filter_state[i].internal_state(3);
    velocity_msg.vector.y = filter_state[i].internal_state(4);
    velocity_msg.vector.z = filter_state[i].internal_state(5);

    dbg_state_position_pubs[i]->publish(point_msg);
    dbg_state_velocity_pubs[i]->publish(velocity_msg);
  }

  for (int i = 0; i < num_landmarks; ++i) {
    // Publish Innovation (Measurement Error)
    auto innovation_msg = geometry_msgs::msg::Vector3Stamped();
    innovation_msg.header.stamp = now;
    innovation_msg.vector.x = dbg_innovations[i](0);
    innovation_msg.vector.y = dbg_innovations[i](1);
    innovation_msg.vector.z = dbg_innovations[i](2);
    dbg_innovation_pubs[i]->publish(innovation_msg);

    auto uncertainty_msg = panda_interfaces::msg::DoubleArrayStamped();
    uncertainty_msg.header.stamp = now;
    uncertainty_msg.data = {dbg_uncertainties[i](0), dbg_uncertainties[i](1),
                            dbg_uncertainties[i](2), dbg_uncertainties[i](3),
                            dbg_uncertainties[i](4), dbg_uncertainties[i](5)};
    dbg_uncertainty_pubs[i]->publish(uncertainty_msg);
  }
}

// Update the state of the kalman filters if the points are valid (depth is
// valid) and initialize remaining kalman filter states when new keypoints have
// good confidence
void SkeletonTracker::kalman_predict(
    const std::vector<std::optional<geometry_msgs::msg::Point>> &points,
    bool init_new, bool only_predict) {
  // Update F matrix of prediction based on time passed
  double dt = delta_time.seconds();
  kalman_params.F(0, 3) = dt;
  kalman_params.F(1, 4) = dt;
  kalman_params.F(2, 5) = dt;
  for (int i = 0; i < num_landmarks; i++) {
    auto point = points[i];
    auto &current_state = filter_state[i];

    if (current_state.is_initialized) {
      // Prediction
      Eigen::Vector<double, 6> pred_state =
          kalman_params.F * current_state.internal_state;
      Eigen::Matrix<double, 6, 6> pred_P =
          kalman_params.Q +
          kalman_params.F * current_state.P * kalman_params.F.transpose();

      Eigen::Matrix<double, 3, 3> S =
          kalman_params.H * pred_P * kalman_params.H.transpose() +
          kalman_params.R;

      Eigen::Matrix<double, 6, 3> K_gain =
          pred_P * kalman_params.H.transpose() * S.inverse();

      // RCLCPP_INFO_STREAM(this->get_logger(), "Kalman gain matrix: " << K_gain);
      // RCLCPP_INFO_STREAM(this->get_logger(), "P_K_1: " << pred_P);
      // RCLCPP_INFO_STREAM(this->get_logger(),
      //                    "P_k_1 * H': " << pred_P *
      //                                          kalman_params.H.transpose());
      // RCLCPP_INFO_STREAM(this->get_logger(), "S: " << S);
      // RCLCPP_INFO_STREAM(this->get_logger(), "S^-1: " << S.inverse());

      // if (image_constants::coco_keypoints[i] == "left_wrist") {
      //   RCLCPP_INFO_STREAM(this->get_logger(),
      //                      "Prediction matrix of left wrist: " << pred_P);
      //   RCLCPP_INFO_STREAM(this->get_logger(),
      //                      "Kalman gain matrix of left wrist: " << K_gain);
      // }
      // if (image_constants::coco_keypoints[i] == "right_wrist") {
      //   RCLCPP_INFO_STREAM(this->get_logger(),
      //                      "Prediction matrix of right wrist: " << pred_P);
      //   RCLCPP_INFO_STREAM(this->get_logger(),
      //                      "Kalman gain matrix of right wrist: " << K_gain);
      // }

      // Update
      // If the point inferred is valid i include the measurement, otherwise
      // just predict based on model and current state
      if (point.has_value() && !only_predict) {
        Eigen::Vector3d z;
        z(0) = point->x;
        z(1) = point->y;
        z(2) = point->z;
        dbg_innovations[i] = z - kalman_params.H * pred_state;
        current_state.internal_state = pred_state + K_gain * dbg_innovations[i];
        current_state.P = pred_P - K_gain * S * K_gain.transpose();
      } else {
        dbg_innovations[i].setZero();
        current_state.internal_state = pred_state;
        current_state.P = pred_P;
      }
    } else {
      // Initialize new keypoint only if requested with init_new
      if (point.has_value() &&
          landmarks[i].conf >= transition_params.confidence_threshold &&
          init_new) {
        Eigen::Vector<double, 6> state;
        state.setZero();
        state(0) = point->x;
        state(1) = point->y;
        state(2) = point->z;
        current_state.init(state, kalman_params.Q);
      }
    }

    dbg_uncertainties[i] = current_state.P.diagonal();
    landmark_3d[i].x = current_state.internal_state.x();
    landmark_3d[i].y = current_state.internal_state.y();
    landmark_3d[i].z = current_state.internal_state.z();
  }
}

void SkeletonTracker::publish_keypoints_tf() {
  geometry_msgs::msg::TransformStamped tf;
  auto parent_frame = camera_frame;
  tf.header.stamp = this->now();
  tf.header.frame_id = parent_frame;

  for (int i = 0; i < num_landmarks; i++) {

    tf.child_frame_id = image_constants::coco_keypoints[i];

    tf.transform.translation.x = filter_state[i].internal_state.x();
    tf.transform.translation.y = filter_state[i].internal_state.y();
    tf.transform.translation.z = filter_state[i].internal_state.z();

    tf_skel_publisher->sendTransform(tf);
  }
}

std::vector<double> SkeletonTracker::calculate_depths(cv::Mat depth_image) {
  std::vector<double> depths;
  double height, width;
  height = depth_image.rows;
  width = depth_image.cols;

  for (size_t i = 0; i < landmarks.size(); i++) {
    depths.push_back(
        depth_image.at<float>(landmarks[i].y * height, landmarks[i].x * width));
  }

  return depths;
}

std::vector<std::optional<geometry_msgs::msg::Point>>
SkeletonTracker::project_pixels_to_3d(std::vector<double> depths) {

  std::vector<std::optional<geometry_msgs::msg::Point>> points;
  geometry_msgs::msg::Point point_in_optical_frame;
  geometry_msgs::msg::Point point_in_camera_frame;
  int height = image_geom.cameraInfo().height;
  int width = image_geom.cameraInfo().width;
  double depth;
  for (size_t i = 0; i < landmarks.size(); i++) {
    cv::Point2d point;
    point.x = landmarks[i].x * width;
    point.y = landmarks[i].y * height;
    cv::Point3d point_3d = image_geom.projectPixelTo3dRay(point);
    if (depths[i] >= min_depth && depths[i] <= max_depth &&
        !std::isnan(depths[i])) {
      depth = depths[i];
      point_in_optical_frame.x = point_3d.x * depth;
      point_in_optical_frame.y = point_3d.y * depth;
      point_in_optical_frame.z = point_3d.z * depth;

      // Rotation from optical frame to camera frame
      point_in_camera_frame.x = point_in_optical_frame.z;
      point_in_camera_frame.y = -point_in_optical_frame.x;
      point_in_camera_frame.z = -point_in_optical_frame.y;

      if (std::isnan(point_in_camera_frame.x) ||
          std::isnan(point_in_camera_frame.y) ||
          std::isnan(point_in_camera_frame.z)) {
        RCLCPP_ERROR(this->get_logger(),
                     "project_pixels_to_3d produced a nan!");
        points.push_back(
            std::optional<geometry_msgs::msg::Point>{}); // Push an invalid
                                                         // optional
      } else {
        points.push_back(std::optional(point_in_camera_frame));
      }
    } else {
      points.push_back(std::optional<geometry_msgs::msg::Point>{});
    }
    // std::cout << "3D point of " << image_constants::coco_keypoints[i] <<
    // ":
    // ("
    //           << ros_point.x << ", " << ros_point.y << ", " <<
    //           ros_point.z
    //           << ")" << std::endl;
  }
  return points;
}
cv::Mat SkeletonTracker::create_skel_img(Image background_scene) {

  cv::Mat img = cv_bridge::toCvCopy(background_scene)->image;
  skeleton_utils::draw_skeleton(img, landmarks, image_constants::skeleton, 0.3);

  return img;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  double min_depth = 0.8;
  double max_depth = 4.0;
  std::string model_path = ament_index_cpp::get_package_prefix("onnxruntime") +
                           "/models/movenet_singlepose_lightning.onnx";
  std::vector<int64_t> input_dims = {1, 192, 192, 3};
  rclcpp::spin(std::make_shared<SkeletonTracker>(
      model_path, "skeleton_tracker", input_dims, min_depth, max_depth));
  rclcpp::shutdown();
  return 0;
}
