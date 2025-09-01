#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_processing/constants.hpp"
#include "image_processing/utils.hpp"
#include "panda_interfaces/msg/body_lengths.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "yolo_msgs/msg/key_point3_d.hpp"
#include "yolo_msgs/msg/key_point3_d_array.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <atomic>
#include <boost/fusion/sequence/intrinsic_fwd.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <image_geometry/stereo_camera_model.hpp>
#include <limits>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/synchronizer.hpp>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
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
#include <rmw/types.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2/convert.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

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
  // Deque to keep last confidence values
  std::deque<double> confidence_history;
  // Moving average confidence of the last frames
  double ma_conf = 0.0;
  // Num of frames the keypoint has been invalid (out of depth)
  int invalid_keypoint_frames = 0;
  // Frames in which consider the state
  std::string frame_id = "world";

  void init(Eigen::Vector<type, STATES> init_state,
            Eigen::Matrix<type, STATES, STATES> init_P,
            const std::string frame_id) {
    is_initialized = true;
    internal_state = init_state;
    P = init_P;
    this->frame_id = frame_id;
  };

  void deinit() {
    ma_conf = 0.0;
    is_initialized = false;
    internal_state.setZero();
  }

  void calc_ma(int ma_window_size) {

    if (confidence_history.size() > ma_window_size) {
      confidence_history.pop_back();
    }

    double sum = 0.0;
    if (!confidence_history.empty()) {
      for (double val : confidence_history) {
        sum += val;
      }
      ma_conf = sum / confidence_history.size();
    } else {
      ma_conf = 0.0;
    }
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
  // Minimum number of valid keypoints to start tracking
  int min_tracked_valid_keypoints = 6;
  // Minimum number of valid keypoints to keep someone tracked
  int min_keypoints_valid_lost = 6;
  // Maximum time the tracker can remain in lost mode
  double lost_time = 0.5;
  // Minimum confidence of single valid keypoint to track a person
  double single_keypoint_ma_confidence_threshold = 0.35;
  // MA mean confidence to keep a keypoint valid
  double ma_confidence_threshold = 0.40;
  // Minimum confidence to not consider a keypoint as an hallucination
  double hallucination_threshold = 0.25;
  // Minimum number of frames with point != null to consider a keypoint
  // recognized
  int min_invalid_frames = 30;
};

struct DecisionData {
  int valid_frames = 0;
  int non_valid_frames = 0;
  double ma_mean_confidence = 0.0;
  int valid_keypoints = 0;
  rclcpp::Time lost_event_time{};
};

const auto sensor_qos =
    rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();

class SkeletonTrackerYolo : public rclcpp::Node {
public:
  SkeletonTrackerYolo(double min_depth, double max_depth)
      : Node("skeleton_tracker"), min_depth(min_depth), max_depth(max_depth) {

    RCLCPP_INFO_STREAM(this->get_logger(), "Getting parameters");
    this->declare_parameter<double>("process_noise", 0.5);
    this->declare_parameter<double>("measurement_noise", 1.0);
    this->declare_parameter<bool>("no_depth", false);
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("filter", false);
    this->declare_parameter<bool>("predict", true);
    this->declare_parameter<int>("MA_window_size", 40);
    std::map<std::string, double> thresholds_parameters;
    thresholds_parameters["ma_confidence_threshold"] = 0.40;
    thresholds_parameters["hallucination_threshold"] = 0.25;
    thresholds_parameters["single_keypoint_ma_confidence_threshold"] = 0.35;
    this->declare_parameters<double>(std::string{}, thresholds_parameters);

    transition_params.min_keypoints_valid_lost = 6;
    transition_params.min_tracked_valid_keypoints = 6;
    transition_params.ma_confidence_threshold =
        this->get_parameter("ma_confidence_threshold").as_double();
    transition_params.hallucination_threshold =
        this->get_parameter("hallucination_threshold").as_double();
    transition_params.single_keypoint_ma_confidence_threshold =
        this->get_parameter("single_keypoint_ma_confidence_threshold")
            .as_double();

    MA_window_size = this->get_parameter("MA_window_size").as_int();
    use_filtering = this->get_parameter("filter").as_bool();
    use_prediction = this->get_parameter("predict").as_bool();

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Initializing kalman filter variables");
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

    bool no_depth = this->get_parameter("no_depth").as_bool();

    RCLCPP_INFO_STREAM(this->get_logger(), "Declaring callbacks");
    auto det_array_cb =
        [this](const yolo_msgs::msg::DetectionArray::SharedPtr array) {
          detections = array;
        };

    auto rgb_img_cb = [this](const Image::SharedPtr img) {
      // std::lock_guard<std::mutex> img_mutex(rgb_mutex);
      current_rgb_image = img;
    };

    auto rgb_camera_info_cb = [this](const sensor_msgs::msg::CameraInfo msg) {
      current_rgb_camera_info = msg;
      rgb_image_geom.fromCameraInfo(current_rgb_camera_info);
    };

    RCLCPP_INFO_STREAM(this->get_logger(), "Creating Subscribers");
    detect_array_sub =
        this->create_subscription<yolo_msgs::msg::DetectionArray>(
            image_constants::detection_array_topic, sensor_qos, det_array_cb);

    rgb_image_sub = this->create_subscription<Image>(
        image_constants::rgb_image_topic, sensor_qos, rgb_img_cb);

    filter_rgb_image_sub.subscribe(this, image_constants::rgb_image_topic,
                                   sensor_qos.get_rmw_qos_profile());

    filter_detect_array_sub.subscribe(this,
                                      image_constants::detection_array_topic,
                                      sensor_qos.get_rmw_qos_profile());

    sync = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            Image, yolo_msgs::msg::DetectionArray>>>(
        message_filters::sync_policies::ApproximateTime<
            Image, yolo_msgs::msg::DetectionArray>(2),
        filter_rgb_image_sub, filter_detect_array_sub);

    using namespace std::placeholders;
    sync->setAgePenalty(0.10);
    sync->registerCallback(
        std::bind(&SkeletonTrackerYolo::synchronized_callback, this, _1, _2));

    std::string depth_image_topic;
    std::string depth_camera_info_topic;
    if (no_depth) {
      depth_camera_info_topic = image_constants::rgb_camera_info_topic;
      depth_image_topic = image_constants::rgb_image_topic;
    } else {
      depth_camera_info_topic = image_constants::depth_camera_info_topic;
      depth_image_topic = image_constants::depth_image_topic;
    }

    rgb_camera_info_sub =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            image_constants::rgb_camera_info_topic, sensor_qos,
            rgb_camera_info_cb);

    skeleton_image_pub =
        std::make_shared<realtime_tools::RealtimePublisher<Image>>(
            this->create_publisher<Image>(image_constants::skeleton_image_topic,
                                          sensor_qos));

    RCLCPP_INFO_STREAM(this->get_logger(), "Creating broadcaster");
    tf_skel_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    skeleton_marker_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            image_constants::skeleton_marker_array_topic, sensor_qos);

    body_lengths_pub =
        this->create_publisher<panda_interfaces::msg::BodyLengths>(
            image_constants::body_lengths_topic, sensor_qos);

    wrists_lenght_pub =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "/wrists_length", sensor_qos);

    publishing_thread = std::thread{[this]() {
      while (!publish_run) {
        std::this_thread::sleep_for(5ms);
      }

      auto debug = this->get_parameter("debug").as_bool();
      while (rclcpp::ok() && publish_run) {
        if (pub) {
          if (use_filtering) {
            this->publish_skeleton_markers();
            this->publish_keypoints_tf();
            this->publish_body_lengths();
          } else {
            this->publish_skeleton_markers(this->shared_keypoints,
                                           !use_filtering);
            this->publish_keypoints_tf(this->shared_keypoints, !use_filtering);
            this->publish_body_lengths(this->shared_keypoints, !use_filtering);
          }
          if (debug) {
            debug_print(this->shared_keypoints);
          }
          pub.store(false);
        }
      }
    }};

    // Start thread
    pub.store(false);
    publish_run.store(true);

    // DEBUG

    auto debug = this->get_parameter("debug").as_bool();
    dbg_uncertainties.resize(num_landmarks,
                             Eigen::Matrix<double, 6, 1>::Zero());
    dbg_innovations.resize(num_landmarks, Eigen::Vector3d::Zero());
    dbg_mahal_dist.data.resize(num_landmarks);
    if (debug) {

      RCLCPP_INFO_STREAM(this->get_logger(), "Creating debug publishers");
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

      dbg_all_depths_pub =
          this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
              "debug/all_depths", 10);

      dbg_all_mahal_dist_pub =
          this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
              "debug/all_mahal_dist", 10);

      for (int i = 0; i < num_landmarks; ++i) {
        std::string topic_name_position = "debug/landmark_" +
                                          image_constants::coco_keypoints[i] +
                                          "/position";
        std::string topic_name_velocity = "debug/landmark_" +
                                          image_constants::coco_keypoints[i] +
                                          "/velocity";
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

        std::string uncertainty_topic_name =
            "debug/landmark_" + image_constants::coco_keypoints[i] +
            "/uncertainty";
        dbg_uncertainty_pubs[i] =
            this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
                uncertainty_topic_name, 10);
      }
    }

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Created successfully the "
                           << this->get_name() << " node with minimum depth: "
                           << min_depth << " and maximum depth: " << max_depth);
  }
  ~SkeletonTrackerYolo() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Joining publish thread");
    publish_run.store(false);
    publishing_thread.join();
  }

private:
  // Subscribers
  // Detection array Sub
  Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detect_array_sub;
  Subscription<Image>::SharedPtr rgb_image_sub;

  message_filters::Subscriber<Image> filter_rgb_image_sub;
  message_filters::Subscriber<yolo_msgs::msg::DetectionArray>
      filter_detect_array_sub;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          Image, yolo_msgs::msg::DetectionArray>>>
      sync;

  Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_sub;
  Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  // Publishers
  realtime_tools::RealtimePublisher<Image>::SharedPtr skeleton_image_pub{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_skel_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_marker_pub;
  rclcpp::Publisher<panda_interfaces::msg::BodyLengths>::SharedPtr
      body_lengths_pub;
  rclcpp::Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      wrists_lenght_pub;

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
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      dbg_all_depths_pub;
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      dbg_all_mahal_dist_pub;

  // Image processing
  std::mutex rgb_mutex;
  Image::SharedPtr current_rgb_image;
  sensor_msgs::msg::CameraInfo current_rgb_camera_info;
  std::string camera_frame = {""};
  std::string keypoints_frame = {""};
  double min_depth, max_depth;
  int num_landmarks;
  image_geometry::PinholeCameraModel rgb_image_geom;
  geometry_msgs::msg::TransformStamped keypoints_camera_tf{};
  geometry_msgs::msg::Point camera_point_wrt_keypoints;

  // Detection array
  yolo_msgs::msg::DetectionArray::SharedPtr detections{nullptr};
  rclcpp::Time last_detection_stamp;

  // Keypoints state variables
  std::optional<Image> skel_image_output;
  std::vector<geometry_msgs::msg::Point> landmark_3d;

  TargetState tracking_state = TargetState::NO_PERSON;
  StateTransitionParams transition_params;
  int MA_window_size;
  DecisionData tracking_decision_data;
  kalman_filt_params<double, 6, 3> kalman_params;
  std::vector<state<double, 6>> filter_state;
  bool use_filtering;
  bool use_prediction;
  rclcpp::Duration delta_time{0, 0};
  std::thread publishing_thread;
  std::atomic<bool> publish_run;
  std::atomic<bool> pub;
  std::map<int, skeleton_utils::landmark_3d> shared_keypoints;

  // Debug
  std::vector<Eigen::Vector3d> dbg_innovations;
  std::vector<Eigen::Matrix<double, 6, 1>> dbg_uncertainties;
  panda_interfaces::msg::DoubleArrayStamped dbg_mahal_dist;

  cv::Mat
  create_skel_img(Image background_scene,
                  const std::map<int, skeleton_utils::landmark_3d> &keypoints);
  void publish_body_lengths(
      const std::map<int, skeleton_utils::landmark_3d> &keypoints =
          std::map<int, skeleton_utils::landmark_3d>(),
      bool print_keypoints = false);
  void publish_keypoints_tf(
      const std::map<int, skeleton_utils::landmark_3d> &keypoints =
          std::map<int, skeleton_utils::landmark_3d>(),
      bool print_keypoints = false);
  void publish_skeleton_markers(
      const std::map<int, skeleton_utils::landmark_3d> &keypoints =
          std::map<int, skeleton_utils::landmark_3d>(),
      bool print_keypoints = false);
  void
  kalman_predict(const std::map<int, skeleton_utils::landmark_3d> &keypoints,
                 bool init_new = true, bool only_predict = false);
  void synchronized_callback(
      const Image::ConstSharedPtr &image,
      const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections);
  void debug_print(const std::map<int, skeleton_utils::landmark_3d> &keypoints);

  std::map<int, skeleton_utils::landmark_3d> get_keypoints(
      const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections) {
    if (detections->detections.empty()) {
      // Returns an empty map if no detections
      return std::map<int, skeleton_utils::landmark_3d>();
    }

    // WARN: Assuming that the first and ONLY
    // detection is human
    std::map<int, skeleton_utils::landmark_3d> keypoints_map;
    std::vector<yolo_msgs::msg::KeyPoint3D> keypoints =
        detections->detections[0].keypoints3d.data;
    // At every detection the vector is filled again
    // with the keypoints detected
    for (auto keypoint : keypoints) {
      // YOLO counting keypoints from 1 to 17
      auto id = keypoint.id;

      // auto depth = keypoint.point.z;
      auto depth = skeleton_utils::distance(this->camera_point_wrt_keypoints,
                                            keypoint.point);
      // Check for depth
      if (depth >= min_depth && depth <= max_depth &&
          (keypoint.point != geometry_msgs::msg::Point{})) {
        keypoints_map[id - 1].point = keypoint.point;
        keypoints_map[id - 1].conf = keypoint.score;
      }
    }
    return keypoints_map;
  }

  void calc_state_track_data(
      const std::map<int, skeleton_utils::landmark_3d> &keypoints) {

    for (int i = 0; i < num_landmarks; i++) {
      auto &current_state = this->filter_state[i];

      if (keypoints.find(i) != keypoints.end()) {
        auto keypoint = keypoints.at(i);
        current_state.confidence_history.push_front(keypoint.conf);
        current_state.calc_ma(MA_window_size);

        if (current_state.ma_conf >
            this->transition_params.single_keypoint_ma_confidence_threshold) {
          current_state.invalid_keypoint_frames = 0;
        }
      } else {
        current_state.invalid_keypoint_frames += 1;
      }
    }
  }

  void
  calc_decision_data(std::map<int, skeleton_utils::landmark_3d> keypoints) {

    // Reset current valid keypoints number
    this->tracking_decision_data.valid_keypoints = 0;
    double ma_conf_sum = 0.0;
    int keypoints_above_hallucination = 0;

    for (int i = 0; i < num_landmarks; i++) {
      const auto &current_state = this->filter_state[i];
      if (keypoints.find(i) != keypoints.end()) {

        if (current_state.ma_conf >
            this->transition_params.single_keypoint_ma_confidence_threshold) {
          tracking_decision_data.valid_keypoints++;
        }

        // Calculate a robust mean confidence based on
        // all non-hallucinated points
        if (current_state.ma_conf > transition_params.hallucination_threshold) {
          ma_conf_sum += current_state.ma_conf;
          keypoints_above_hallucination++;
        }
      }
    }

    if (keypoints_above_hallucination > 0) {
      this->tracking_decision_data.ma_mean_confidence =
          ma_conf_sum / keypoints_above_hallucination;
    } else {
      this->tracking_decision_data.ma_mean_confidence = 0.0;
    }
  }

  // State transition methods
  void no_person_to_tracked(
      const std::map<int, skeleton_utils::landmark_3d> &keypoints) {
    // There's good confidence in the last frames,
    // init the kalman filters of valid keypoints
    // right now without running the update
    for (int i = 0; i < num_landmarks; i++) {
      if (keypoints.find(i) != keypoints.end()) {

        if (this->filter_state[i].ma_conf >=
            this->transition_params.single_keypoint_ma_confidence_threshold) {
          Eigen::Vector<double, 6> state;
          state.setZero();
          state.x() = keypoints.at(i).point.x;
          state.y() = keypoints.at(i).point.y;
          state.z() = keypoints.at(i).point.z;
          this->filter_state[i].init(state, kalman_params.Q,
                                     this->keypoints_frame);
        }
      }
    }
  }
};

void SkeletonTrackerYolo::synchronized_callback(
    const Image::ConstSharedPtr &image,
    const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Entered in sync callback");

  // auto start_callback = std::chrono::high_resolution_clock::now();

  if (!detections->detections.empty()) {
    auto detections_frame_id =
        std::string{detections->detections[0].keypoints3d.frame_id};
    if (keypoints_frame != detections_frame_id &&
        !detections_frame_id.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "Keypoint frame different from the prevoius one, updating "
                  "the keypoint frame and the camera frame, then "
                  "deiniting filters");
      keypoints_frame = detections_frame_id;
      camera_frame = std::string{detections->header.frame_id};
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Camera frame: " << camera_frame
                                          << ", Keypoints frame: "
                                          << keypoints_frame);
      for (auto &filt : this->filter_state) {
        filt.deinit();
      }
      this->keypoints_camera_tf = tf_buffer->lookupTransform(
          keypoints_frame, camera_frame, tf2::TimePointZero, 10s);
      geometry_msgs::msg::Point camera_point;
      this->camera_point_wrt_keypoints.x =
          this->keypoints_camera_tf.transform.translation.x;
      this->camera_point_wrt_keypoints.y =
          this->keypoints_camera_tf.transform.translation.y;
      this->camera_point_wrt_keypoints.z =
          keypoints_camera_tf.transform.translation.z;
      this->tracking_state = TargetState::NO_PERSON;
    }
  }

  auto start = this->now();
  if (last_detection_stamp.seconds() == 0) { // First frame
    last_detection_stamp = image->header.stamp;
    return;
  }
  delta_time = rclcpp::Time(image->header.stamp) - last_detection_stamp;
  last_detection_stamp = image->header.stamp;

  // // Save current messages to not lose future messages

  // 1) Get landmark on current image
  //
  auto keypoints_map = get_keypoints(detections);

  // 2) Take decision based on current state
  //
  // Update internal filter state and tracking data
  calc_state_track_data(keypoints_map);

  // Update higher level tracking data
  calc_decision_data(keypoints_map);

  switch (this->tracking_state) {
  case TargetState::NO_PERSON: {

    if (this->tracking_decision_data.ma_mean_confidence >=
            this->transition_params.ma_confidence_threshold &&
        this->tracking_decision_data.valid_keypoints >=
            this->transition_params.min_tracked_valid_keypoints) {

      // switch to tracking person and initialize kalman filters
      //
      RCLCPP_INFO_STREAM(this->get_logger(), "Switching to tracking person");
      no_person_to_tracked(keypoints_map);
      this->tracking_state = TargetState::PERSON_TRACKED;
      break;
    }
    break;
  }
  case TargetState::PERSON_TRACKED: {
    if (this->tracking_decision_data.ma_mean_confidence <
        this->transition_params.ma_confidence_threshold
        // ||
        // tracking_decision_data.valid_keypoints <
        //     transition_params.min_keypoints_valid_lost
    ) {
      // switch to lost person and initialize the lost event time for
      // lost state
      RCLCPP_INFO_STREAM(this->get_logger(), "Switching to lost person");
      if (use_prediction) {
        this->kalman_predict(keypoints_map, false, true);
      }
      this->tracking_decision_data.lost_event_time = last_detection_stamp;
      this->tracking_state = TargetState::PERSON_LOST;
      break;
    }
    // Init new kalman filter if keypoint is valid and run kalman
    // update
    this->kalman_predict(keypoints_map);
    break;
  }
  case TargetState::PERSON_LOST: {
    if ((this->last_detection_stamp -
         this->tracking_decision_data.lost_event_time)
            .seconds() > this->transition_params.lost_time) {
      // Switch to no tracked person state and deinitialize all the
      // kalman filters
      RCLCPP_INFO_STREAM(this->get_logger(), "Switching to NO person");
      for (auto &filt : this->filter_state) {
        filt.deinit();
      }
      this->tracking_state = TargetState::NO_PERSON;
      break;
    } else if (this->tracking_decision_data.ma_mean_confidence >=
               this->transition_params.ma_confidence_threshold) {
      // Switch to tracked person state, probably there have been an
      // obstruction for some frames
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Switching to tracked person from lost");
      this->tracking_state = TargetState::PERSON_TRACKED;
      this->kalman_predict(keypoints_map);
      break;
    }
    // Keep predict with kalman filters
    if (use_prediction) {
      this->kalman_predict(keypoints_map, false, true);
    }
    break;
  }
  }

  // 3) Publish infos to external network
  //
  this->shared_keypoints = keypoints_map;
  this->pub.store(true);

  skel_image_output =
      *(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8",
                           this->create_skel_img(*image, keypoints_map))
            .toImageMsg());
  skeleton_image_pub->tryPublish(skel_image_output.value());
  // RCLCPP_INFO_STREAM(
  //     this->get_logger(),
  //     "Time passed inside callback: "
  //         << std::chrono::duration_cast<std::chrono::nanoseconds>(
  //                std::chrono::high_resolution_clock::now() - start_callback)
  //                .count() * 1e-9);
}

void SkeletonTrackerYolo::debug_print(
    const std::map<int, skeleton_utils::landmark_3d> &keypoints) {
  if (keypoints.empty()) {
    return; // Nothing to publish if no landmarks were detected
  }
  auto now = this->now();
  auto confs_msg = panda_interfaces::msg::DoubleArrayStamped();
  confs_msg.header.stamp = now;

  for (const auto &pair : keypoints) {
    confs_msg.data.push_back(pair.second.conf);
  }

  dbg_all_confs_pub->publish(confs_msg);

  auto mean_conf_msg = panda_interfaces::msg::DoubleStamped();
  mean_conf_msg.header.stamp = now;
  mean_conf_msg.data = tracking_decision_data.ma_mean_confidence;
  dbg_mean_conf_pub->publish(mean_conf_msg);

  for (int i = 0; i < num_landmarks; ++i) {
    auto point_msg = geometry_msgs::msg::PointStamped();
    auto velocity_msg = geometry_msgs::msg::Vector3Stamped();
    point_msg.header.stamp = now;
    velocity_msg.header.stamp = now;

    point_msg.header.frame_id = this->keypoints_frame;
    velocity_msg.header.frame_id = this->keypoints_frame;

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

  dbg_all_mahal_dist_pub->publish(dbg_mahal_dist);
}

// Update the state of the kalman filters if the points are valid (depth is
// valid) and initialize remaining kalman filter states when new keypoints have
// good confidence
void SkeletonTrackerYolo::kalman_predict(
    const std::map<int, skeleton_utils::landmark_3d> &keypoints, bool init_new,
    bool only_predict) {
  // Update F matrix of prediction based on time passed
  double dt = delta_time.seconds();
  kalman_params.F(0, 3) = dt;
  kalman_params.F(1, 4) = dt;
  kalman_params.F(2, 5) = dt;

  const double mahalanobis_threshold = 6.5;

  for (int i = 0; i < num_landmarks; i++) {
    auto &current_state = filter_state[i];
    dbg_mahal_dist.data[i] = 0.0;

    if (current_state.is_initialized) {
      if (current_state.invalid_keypoint_frames >
          transition_params.min_invalid_frames) {
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //                    "Deinit state of "
        //                        << image_constants::coco_keypoints[i]);
        current_state.deinit();
        continue;
      }

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

      // Update
      // If the point inferred is valid i include the measurement, otherwise
      // just predict based on model and current state
      if (keypoints.find(i) != keypoints.end() && !only_predict) {
        auto point = keypoints.at(i).point;
        Eigen::Vector3d z;
        z(0) = point.x;
        z(1) = point.y;
        z(2) = point.z;
        dbg_innovations[i] = z - kalman_params.H * pred_state;

        double mahalanobis_squared =
            dbg_innovations[i].transpose() * S.inverse() * dbg_innovations[i];
        dbg_mahal_dist.data[i] = mahalanobis_squared;
        if (mahalanobis_squared < mahalanobis_threshold) {
          current_state.internal_state =
              pred_state + K_gain * dbg_innovations[i];
          current_state.P = pred_P - K_gain * S * K_gain.transpose();
        } else {
          current_state.invalid_keypoint_frames++;
          dbg_innovations[i].setZero();
          current_state.internal_state = pred_state;
          current_state.P = pred_P;
        }
      } else {
        dbg_innovations[i].setZero();
        current_state.internal_state = pred_state;
        current_state.P = pred_P;
      }
    } else {
      // Initialize new keypoint only if requested with init_new
      if (keypoints.find(i) != keypoints.end() &&
          current_state.ma_conf >=
              transition_params.single_keypoint_ma_confidence_threshold &&
          init_new) {
        auto point = keypoints.at(i).point;
        Eigen::Vector<double, 6> state;
        state.setZero();
        state(0) = point.x;
        state(1) = point.y;
        state(2) = point.z;
        current_state.init(state, kalman_params.Q, this->keypoints_frame);
      }
    }

    dbg_uncertainties[i] = current_state.P.diagonal();

    // Update the single 3d landmark at each call to kalman
    landmark_3d[i].x = current_state.internal_state.x();
    landmark_3d[i].y = current_state.internal_state.y();
    landmark_3d[i].z = current_state.internal_state.z();
  }
}

void SkeletonTrackerYolo::publish_skeleton_markers(
    const std::map<int, skeleton_utils::landmark_3d> &keypoints,
    bool print_keypoints) {
  std::vector<state<double, 6>> filter_state;
  std::map<int, skeleton_utils::landmark_3d> keypoints_inside_func;
  if (!print_keypoints) {
    filter_state = this->filter_state;
  } else {
    keypoints_inside_func = keypoints;
  }
  auto now = this->get_clock()->now();
  visualization_msgs::msg::MarkerArray marker_array;

  // --- 1. Create the SPHERE_LIST for the joints ---
  visualization_msgs::msg::Marker joint_marker;
  joint_marker.header.frame_id = this->keypoints_frame;
  joint_marker.header.stamp = now;
  joint_marker.ns = "joints";
  joint_marker.id = 0;
  joint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  joint_marker.action = visualization_msgs::msg::Marker::ADD;
  joint_marker.pose.orientation.w = 1.0;

  // Define sphere properties
  joint_marker.scale.x = 0.03; // Sphere diameter in meters
  joint_marker.scale.y = 0.03;
  joint_marker.scale.z = 0.03;

  // Loop through all landmarks
  if (!print_keypoints) {
    for (int i = 0; i < num_landmarks; ++i) {
      if (filter_state[i].is_initialized) {
        // Add the 3D point from your Kalman filter's state
        geometry_msgs::msg::Point p = landmark_3d[i];
        joint_marker.points.push_back(p);

        // Set the color based on confidence or uncertainty
        std_msgs::msg::ColorRGBA color;
        // Example: Green for high confidence, Red for low.
        // You can get more fancy with gradients here.
        float confidence = filter_state[i].ma_conf;
        color.r = 1.0f - confidence;
        color.g = confidence;
        color.b = 0.0f;
        color.a = 1.0f;
        joint_marker.colors.push_back(color);
      }
    }
  } else {
    for (auto keypoint : keypoints_inside_func) {
      geometry_msgs::msg::Point p = keypoint.second.point;
      joint_marker.points.push_back(p);

      // Set the color based on confidence or uncertainty
      std_msgs::msg::ColorRGBA color;
      // Example: Green for high confidence, Red for low.
      // You can get more fancy with gradients here.
      float confidence = keypoint.second.conf;
      color.r = 1.0f - confidence;
      color.g = confidence;
      color.b = 0.0f;
      color.a = 1.0f;
      joint_marker.colors.push_back(color);
    }
  }

  // Add the joint marker to the array
  marker_array.markers.push_back(joint_marker);

  // --- 2. Create the LINE_LIST for the bones ---
  visualization_msgs::msg::Marker bone_marker;
  bone_marker.header.frame_id = this->keypoints_frame;
  bone_marker.header.stamp = now;
  bone_marker.ns = "bones";
  bone_marker.id = 1;
  bone_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  bone_marker.action = visualization_msgs::msg::Marker::ADD;
  bone_marker.pose.orientation.w = 1.0;

  // Define line properties
  bone_marker.scale.x = 0.015; // Line width in meters

  // A single color for all bones
  bone_marker.color.r = 1.0f;
  bone_marker.color.g = 1.0f;
  bone_marker.color.b = 1.0f;
  bone_marker.color.a = 1.0f;

  // Loop through our defined bone connections
  for (const auto &bone : image_constants::skeleton) {
    int start_idx = bone.first;
    int end_idx = bone.second;

    // Only draw the bone if both of its joints are being tracked
    if (!print_keypoints) {

      if (filter_state[start_idx].is_initialized &&
          filter_state[end_idx].is_initialized) {
        geometry_msgs::msg::Point p_start = landmark_3d[start_idx];
        geometry_msgs::msg::Point p_end = landmark_3d[end_idx];

        bone_marker.points.push_back(p_start);
        bone_marker.points.push_back(p_end);
      }
    } else {
      if (keypoints_inside_func.find(start_idx) !=
              keypoints_inside_func.end() &&
          keypoints_inside_func.find(end_idx) != keypoints_inside_func.end()) {
        geometry_msgs::msg::Point p_start =
            keypoints_inside_func.at(start_idx).point;
        geometry_msgs::msg::Point p_end =
            keypoints_inside_func.at(end_idx).point;

        bone_marker.points.push_back(p_start);
        bone_marker.points.push_back(p_end);
      }
    }
  }

  // Add the bone marker to the array
  marker_array.markers.push_back(bone_marker);

  // --- 3. Publish the entire array ---
  skeleton_marker_pub->publish(marker_array);
}

void SkeletonTrackerYolo::publish_keypoints_tf(
    const std::map<int, skeleton_utils::landmark_3d> &keypoints,
    bool print_keypoints) {
  geometry_msgs::msg::TransformStamped tf;
  auto parent_frame = this->keypoints_frame;
  tf.header.stamp = this->now();
  tf.header.frame_id = parent_frame;

  if (!print_keypoints) {
    for (int i = 0; i < num_landmarks; i++) {

      if (!filter_state[i].is_initialized) {
        continue;
      }
      tf.child_frame_id = image_constants::coco_keypoints[i];

      tf.transform.translation.x = filter_state[i].internal_state.x();
      tf.transform.translation.y = filter_state[i].internal_state.y();
      tf.transform.translation.z = filter_state[i].internal_state.z();

      tf_skel_publisher->sendTransform(tf);
    }
    // Print center of skeleton
    // 5, 6, 11, and 12 are the indexes (0 based) of shoulders and hips in coco
    // keypoints-17
    if (filter_state[5].is_initialized && filter_state[6].is_initialized &&
        filter_state[11].is_initialized && filter_state[12].is_initialized) {
      geometry_msgs::msg::Point hips_center;
      geometry_msgs::msg::Point shoulders_center;
      shoulders_center.x = (landmark_3d[5].x + landmark_3d[6].x) / 2.0;
      shoulders_center.y = (landmark_3d[5].y + landmark_3d[6].y) / 2.0;
      shoulders_center.z = (landmark_3d[5].z + landmark_3d[6].z) / 2.0;

      hips_center.x = (landmark_3d[11].x + landmark_3d[12].x) / 2.0;
      hips_center.y = (landmark_3d[11].y + landmark_3d[12].y) / 2.0;
      hips_center.z = (landmark_3d[11].z + landmark_3d[12].z) / 2.0;

      tf.child_frame_id = image_constants::skeleton_center;
      tf.transform.translation.x =
          0.9 * hips_center.x + 0.1 * shoulders_center.x;
      tf.transform.translation.y =
          0.9 * hips_center.y + 0.1 * shoulders_center.y;
      tf.transform.translation.z =
          0.9 * hips_center.z + 0.1 * shoulders_center.z;
      tf_skel_publisher->sendTransform(tf);
    }
  } else {
    for (auto keypoint : keypoints) {
      if (keypoint.first < num_landmarks) {
        tf.child_frame_id = image_constants::coco_keypoints[keypoint.first];

        tf.transform.translation.x = keypoint.second.point.x;
        tf.transform.translation.y = keypoint.second.point.y;
        tf.transform.translation.z = keypoint.second.point.z;

        tf_skel_publisher->sendTransform(tf);
      }
    }
    if (keypoints.find(5) != keypoints.end() &&
        keypoints.find(6) != keypoints.end() &&
        keypoints.find(11) != keypoints.end() &&
        keypoints.find(12) != keypoints.end()) {

      geometry_msgs::msg::Point hips_center;
      geometry_msgs::msg::Point shoulders_center;

      shoulders_center.x =
          (keypoints.at(5).point.x + keypoints.at(6).point.x) / 2.0;
      shoulders_center.y =
          (keypoints.at(5).point.y + keypoints.at(6).point.y) / 2.0;
      shoulders_center.z =
          (keypoints.at(5).point.z + keypoints.at(6).point.z) / 2.0;

      hips_center.x =
          (keypoints.at(11).point.x + keypoints.at(12).point.x) / 2.0;
      hips_center.y =
          (keypoints.at(11).point.y + keypoints.at(12).point.y) / 2.0;
      hips_center.z =
          (keypoints.at(11).point.z + keypoints.at(12).point.z) / 2.0;

      tf.child_frame_id = image_constants::skeleton_center;
      tf.transform.translation.x =
          0.9 * hips_center.x + 0.1 * shoulders_center.x;
      tf.transform.translation.y =
          0.9 * hips_center.y + 0.1 * shoulders_center.y;
      tf.transform.translation.z =
          0.9 * hips_center.z + 0.1 * shoulders_center.z;
      tf_skel_publisher->sendTransform(tf);
    }
  }
}

void SkeletonTrackerYolo::publish_body_lengths(
    const std::map<int, skeleton_utils::landmark_3d> &keypoints,
    bool print_keypoints) {

  std::vector<state<double, 6>> filter_state;
  std::map<int, skeleton_utils::landmark_3d> keypoints_inside_func;
  if (!print_keypoints) {
    filter_state = this->filter_state;
  } else {
    keypoints_inside_func = keypoints;
  }
  panda_interfaces::msg::BodyLengths body_lengths;

  for (size_t i = 0; i < image_constants::skeleton.size(); i++) {
    auto bone = std::pair<std::string, std::pair<int, int>>{
        image_constants::skeleton_parts[i], image_constants::skeleton[i]};

    auto start_idx = bone.second.first;
    auto end_idx = bone.second.second;
    // Only draw the bone if both of its joints are being tracked
    if (!print_keypoints) {

      if (filter_state[start_idx].is_initialized &&
          filter_state[end_idx].is_initialized) {
        geometry_msgs::msg::Point p_start = landmark_3d[start_idx];
        geometry_msgs::msg::Point p_end = landmark_3d[end_idx];

        std::string name = bone.first;
        double length = skeleton_utils::distance(p_start, p_end);
        body_lengths.name.push_back(name);
        body_lengths.value.push_back(length);
      }
    } else {
      if (keypoints_inside_func.find(start_idx) !=
              keypoints_inside_func.end() &&
          keypoints_inside_func.find(end_idx) != keypoints_inside_func.end()) {
        geometry_msgs::msg::Point p_start =
            keypoints_inside_func.at(start_idx).point;
        geometry_msgs::msg::Point p_end =
            keypoints_inside_func.at(end_idx).point;

        std::string name = bone.first;
        double length = skeleton_utils::distance(p_start, p_end);
        body_lengths.name.push_back(name);
        body_lengths.value.push_back(length);
      }
    }
  }
  if (filter_state[9].is_initialized && filter_state[10].is_initialized) {

    geometry_msgs::msg::Point p_start = landmark_3d[9];
    geometry_msgs::msg::Point p_end = landmark_3d[10];

    panda_interfaces::msg::DoubleStamped dist;
    dist.header.stamp = this->now();
    dist.data = skeleton_utils::distance(p_start, p_end);
    wrists_lenght_pub->publish(dist);
  }

  body_lengths_pub->publish(body_lengths);
}

cv::Mat SkeletonTrackerYolo::create_skel_img(
    Image background_scene,
    const std::map<int, skeleton_utils::landmark_3d> &) {

  cv::Mat img = cv_bridge::toCvCopy(background_scene)->image;
  std::map<int, skeleton_utils::landmark> tmp_landmarks;
  for (int i = 0; i < num_landmarks; i++) {
    if (filter_state[i].is_initialized) {
      geometry_msgs::msg::PointStamped mark_3d;
      mark_3d.header.frame_id = keypoints_frame;
      mark_3d.point = landmark_3d[i];

      if (filter_state[i].frame_id != camera_frame) {
        // Then the filter state is in keypoints frame, different from the
        // camera, we apply the transform
        mark_3d = tf_buffer->transform(mark_3d, camera_frame, 10s);
      }
      cv::Point3d tmp_point;
      tmp_point.x = mark_3d.point.x;
      tmp_point.y = mark_3d.point.y;
      tmp_point.z = mark_3d.point.z;
      auto mark = rgb_image_geom.project3dToPixel(tmp_point);
      tmp_landmarks[i].x = mark.x;
      tmp_landmarks[i].y = mark.y;
      tmp_landmarks[i].conf = filter_state[i].ma_conf;
    } else {
      tmp_landmarks[i].conf = -1.0;
    }
  }
  skeleton_utils::draw_skeleton(img, tmp_landmarks, image_constants::skeleton,
                                0.0, true);

  return img;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  double min_depth = 0.8;
  double max_depth = 4.0;
  rclcpp::spin(std::make_shared<SkeletonTrackerYolo>(min_depth, max_depth));
  rclcpp::shutdown();
  return 0;
}
