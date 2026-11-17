#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace wla::uvc {
struct GroundPoint {
  cv::Vec3d body;
  bool candidate;
};
struct GroundMap {
  bool available = false;
  std::string reason = "waiting_for_inputs";
  cv::Vec3d normal{0, 0, 1};
  double offset = 0, angleDeg = 0, offsetDelta = 0, ratio = 0, rms = 0,
         priorResidual = 0;
  cv::Matx33d bodyToMap = cv::Matx33d::eye();
  int width = 80, height = 80;
  double resolution = .05, x0 = -1, y0 = -2;
  std::vector<int8_t> occupancy;
  std::vector<uint8_t> classes;
  std::vector<cv::Vec3f> ground, obstacles, uncertain;
  // classes: 0 unknown, 1 observed near-plane, 2 raised, 3 below-plane, 4
  // uncertain height
};
GroundMap MakeGroundMap(const std::vector<GroundPoint> &points,
                        const cv::Vec3d &priorNormal, double priorOffset);
} // namespace wla::uvc
