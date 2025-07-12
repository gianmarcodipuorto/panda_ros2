#pragma once
#include "image_processing/constants.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace skeleton_utils {

struct landmark {
  float x;
  float y;
  float conf;
};

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

} // namespace skeleton_utils
