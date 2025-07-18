#pragma once
#include "geometry_msgs/msg/point.hpp"
#include "image_processing/constants.hpp"
#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace skeleton_utils {

struct landmark {
  float x;
  float y;
  float conf;
};

struct landmark_3d {
  geometry_msgs::msg::Point point;
  double conf;
};

inline double distance(geometry_msgs::msg::Point p1,
                       geometry_msgs::msg::Point p2) {
  return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                   pow(p1.z - p2.z, 2));
}

inline cv::Scalar get_gradient_color(float confidence) {
  confidence = std::max(0.0f, std::min(1.0f, confidence));

  uchar value = static_cast<uchar>(255.0 * confidence);

  return cv::Scalar(0, value, 0);
}

inline void draw_skeleton(cv::Mat &image, std::map<int, landmark> landmarks,
                          const std::vector<std::pair<int, int>> &skeleton,
                          const float confidence_thresh,
                          bool is_pixel_normalized = true) {
  const int num_keypoints = 17;

  // Drawing segment first
  auto end = landmarks.end();

  for (std::pair<int, int> segment : skeleton) {
    if (landmarks.find(segment.first) != end &&
        landmarks.find(segment.second) != end) {
      landmark first = landmarks[segment.first];
      landmark second = landmarks[segment.second];
      if (first.conf > confidence_thresh && second.conf > confidence_thresh) {

        cv::Point first, second;
        if (is_pixel_normalized) {
          first.x = landmarks[segment.first].x;
          first.y = landmarks[segment.first].y;
          second.x = landmarks[segment.second].x;
          second.y = landmarks[segment.second].y;
        } else {

          first.x = landmarks[segment.first].x * image.cols;
          first.y = landmarks[segment.first].y * image.rows;
          second.x = landmarks[segment.second].x * image.cols;
          second.y = landmarks[segment.second].y * image.rows;
        }

        cv::line(image, first, second, cv::Scalar(0, 0, 255), 3);
      }
    }
  }
  // Drawing circle/dot after
  float conf_sum = 0;
  int keypoint_accepted = 0;
  for (int i = 0; i < num_keypoints; ++i) {
    landmark mark = landmarks[i];
    conf_sum += mark.conf;
    if (mark.conf > confidence_thresh) {
      keypoint_accepted++;
      // std::cout << "Landmark " << image_constants::coco_keypoints[i]
      //           << " with x:" << mark.x << ", y:" << mark.y
      //           << ", conf:" << mark.conf << std::endl;
      float y_norm = mark.y;
      float x_norm = mark.x;

      int x = static_cast<int>(x_norm * image.cols);
      int y = static_cast<int>(y_norm * image.rows);

      cv::Point point = cv::Point(x, y);
      cv::circle(image, point, 3, get_gradient_color(mark.conf), -1);
    }
  }
}

inline uint16_t get_median_depth(const cv::Mat &depth_image, int u, int v,
                                 int patch_size = 3) {
  // --- 1. Boundary check for the center pixel ---
  if (v < 0 || v >= depth_image.rows || u < 0 || u >= depth_image.cols) {
    return 0; // The target pixel is outside the image
  }

  // --- 2. Define the ROI and clamp it to the image dimensions ---
  const int patch_offset = patch_size / 2;
  cv::Rect roi(u - patch_offset, v - patch_offset, patch_size, patch_size);
  roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);

  // --- 3. Extract the patch and collect valid (non-zero) depth pixels ---
  cv::Mat patch = depth_image(roi);
  std::vector<uint16_t> valid_pixels;
  valid_pixels.reserve(patch_size * patch_size); // Pre-allocate memory

  for (int r = 0; r < patch.rows; ++r) {
    for (int c = 0; c < patch.cols; ++c) {
      uint16_t depth_raw_mm = patch.at<uint16_t>(r, c);
      // We only consider pixels that have a real depth reading
      if (depth_raw_mm > 0) {
        valid_pixels.push_back(depth_raw_mm);
      }
    }
  }

  // --- 4. Determine if the patch is reliable and find the median ---
  // If less than, say, 25% of the pixels in the patch are valid,
  // we consider the reading unreliable. This is a tunable heuristic.
  const size_t min_valid_pixels = (patch_size * patch_size) / 4;
  if (valid_pixels.size() < min_valid_pixels) {
    return 0; // Not enough good data in the neighborhood
  }

  // Find the median element by sorting
  std::sort(valid_pixels.begin(), valid_pixels.end());

  // The median is the element in the middle of the sorted vector
  return valid_pixels[valid_pixels.size() / 2];
}

} // namespace skeleton_utils
