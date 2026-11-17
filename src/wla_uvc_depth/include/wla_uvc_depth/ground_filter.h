#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace wla::uvc {
struct GroundPoint {
  cv::Vec3d body;
  bool candidate;
};
struct GroundFilter {
  bool available = false;
  std::string reason = "waiting_for_inputs";
  cv::Vec3d normal{0, 0, 1};
  double offset = 0, angleDeg = 0, offsetDelta = 0, ratio = 0, rms = 0,
         priorResidual = 0;
  cv::Matx33d bodyToPlane = cv::Matx33d::eye();
  std::vector<cv::Vec3f> ground, obstacles, uncertain;
  // Points remain in the input body coordinates; plane basis is for display
  // only.
};
GroundFilter MakeGroundFilter(const std::vector<GroundPoint> &points,
                              const cv::Vec3d &priorNormal, double priorOffset);
} // namespace wla::uvc
