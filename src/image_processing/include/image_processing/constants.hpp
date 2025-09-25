#pragma once
#include <string>
#include <vector>

namespace image_constants {
const std::vector<std::pair<int, int>> skeleton = {
    {0, 1},   // Nose -> Left Eye
    {0, 2},   // Nose -> Right Eye
    {1, 3},   // Left Eye -> Left Ear
    {2, 4},   // Right Eye -> Right Ear
    {0, 5},   // Nose -> Left Shoulder
    {0, 6},   // Nose -> Right Shoulder
    {5, 7},   // Left Shoulder -> Left Elbow
    {7, 9},   // Left Elbow -> Left Wrist
    {6, 8},   // Right Shoulder -> Right Elbow
    {8, 10},  // Right Elbow -> Right Wrist
    {5, 6},   // Left Shoulder -> Right Shoulder
    {5, 11},  // Left Shoulder -> Left Hip
    {6, 12},  // Right Shoulder -> Right Hip
    {11, 12}, // Left Hip -> Right Hip
    {11, 13}, // Left Hip -> Left Knee
    {13, 15}, // Left Knee -> Left Ankle
    {12, 14}, // Right Hip -> Right Knee
    {14, 16}  // Right Knee -> Right Ankle
};
const std::vector<std::string> skeleton_parts = {
    std::string{"Nose_left_eye"},
    std::string{"Nose_right_eye"},
    std::string{"Left_eye_left_ear"},
    std::string{"Right_eye_right_ear"},
    std::string{"Nose_left_shoulder"},
    std::string{"Nose_right_shoulder"},
    std::string{"Left_shoulder_left_elbow"},
    std::string{"Left_elbow_left_wrist"},
    std::string{"Right_shoulder_right_elbow"},
    std::string{"Right_elbow_right_wrist"},
    std::string{"Left_shoulder_right_shoulder"},
    std::string{"Left_shoulder_left_hip"},
    std::string{"Right_shoulder_right_hip"},
    std::string{"Left_hip_right_hip"},
    std::string{"Left_hip_left_knee"},
    std::string{"Left_knee_left_ankle"},
    std::string{"Right_hip_right_knee"},
    std::string{"Right_knee_right_ankle"},
};
const std::vector<std::string> coco_keypoints = {
    "nose",        "left_eye",      "right_eye",      "left_ear",
    "right_ear",   "left_shoulder", "right_shoulder", "left_elbow",
    "right_elbow", "left_wrist",    "right_wrist",    "left_hip",
    "right_hip",   "left_knee",     "right_knee",     "left_ankle",
    "right_ankle"};
const int LEFT_WRIST_IDX = 9;
const int RIGHT_WRIST_IDX = 10;
const std::string left_wrist{"left_wrist"};
const std::string right_wrist{"right_wrist"};
const std::string skeleton_center{"belly"};

// Topics
const std::string rgb_image_topic{"/image_raw"};
const std::string rgb_camera_info_topic{"/depth/camera_info"};

const std::string depth_image_topic{"/depth/image_raw"};
const std::string depth_camera_info_topic{"/depth/camera_info"};

// const std::string depth_image_topic{"/depth_registered/image_rect"};
// const std::string depth_camera_info_topic{"/depth_registered/camera_info"};

const std::string detection_array_topic{"/yolo/detections_3d"};

const std::string skeleton_image_topic{"/skeleton"};
const std::string skeleton_marker_array_topic{"/skeleton_array"};
const std::string body_lengths_topic{"/body_lengths"};
const int DEFAULT_TOPIC_QOS = 10;

} // namespace image_constants
