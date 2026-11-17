#ifndef WLA_UVC_DEPTH_CORE_H
#define WLA_UVC_DEPTH_CORE_H
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <array>
#include <string>
#include <utility>

namespace wla::uvc {
struct Camera { cv::Mat k, d; cv::Size size; };
struct Calibration {
  std::array<Camera,3> cameras;
  cv::Mat rightFromLeft, rgbFromLeft;
};
struct Options {
  std::string algorithm="sgbm", rgbMode="crop";
  int disparities=128, block=5;
  cv::Size rectifiedSize{640,360};
  double nearM=.15, farM=5.;
};
struct Result { cv::Mat depth, disparity, left, right; };
[[nodiscard]] Calibration LoadCalibration(const std::string& path);
[[nodiscard]] cv::Mat CropMatrix(cv::Size output);
[[nodiscard]] cv::Mat RawColor(const cv::Mat& image,cv::Size output);
[[nodiscard]] cv::Mat ProjectDepth(const cv::Mat& points,const cv::Mat& k,
  const cv::Mat& transform,cv::Size output,double near_m=.15,double far_m=5.);
class Core {
 public:
  explicit Core(Calibration calibration,Options options={});
  Core(const Core&)=delete;
  Core& operator=(const Core&)=delete;
  Core(Core&&)=delete;
  Core& operator=(Core&&)=delete;
  ~Core()=default;
  [[nodiscard]] cv::Mat color(const cv::Mat& rgb) const;
  [[nodiscard]] std::pair<cv::Mat,cv::Mat> rectify(const cv::Mat& left,const cv::Mat& right) const;
  [[nodiscard]] cv::Mat pointsFromDisparity(const cv::Mat& disparity) const;
  [[nodiscard]] Result depth(const cv::Mat& left,const cv::Mat& right);
  [[nodiscard]] const cv::Mat& colorK() const noexcept { return m_colorK; }
  [[nodiscard]] cv::Size outputSize() const noexcept { return m_output; }
  [[nodiscard]] cv::Size rectifiedSize() const noexcept { return m_options.rectifiedSize; }
  [[nodiscard]] cv::Size stereoSize() const noexcept { return m_calibration.cameras[0].size; }
  [[nodiscard]] const cv::Mat& leftR() const noexcept { return m_r1; }
  [[nodiscard]] const cv::Mat& rightR() const noexcept { return m_r2; }
  [[nodiscard]] const cv::Mat& leftP() const noexcept { return m_p1; }
  [[nodiscard]] const cv::Mat& rightP() const noexcept { return m_p2; }
  [[nodiscard]] const cv::Mat& reprojectionMatrix() const noexcept { return m_q; }
  [[nodiscard]] std::array<cv::Mat,4> rectifyMaps() const { return {m_lx,m_ly,m_rx,m_ry}; }
 private:
  Calibration m_calibration;
  Options m_options;
  cv::Size m_output;
  cv::Mat m_r1,m_r2,m_p1,m_p2,m_q,m_colorK;
  cv::Mat m_lx,m_ly,m_rx,m_ry,m_cx,m_cy;
  cv::Ptr<cv::StereoMatcher> m_matcher;
};
}
#endif
