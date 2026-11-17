#ifndef WLA_UVC_DEPTH_STEREO_ENGINE_H
#define WLA_UVC_DEPTH_STEREO_ENGINE_H
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>
#include <array>
namespace wla::uvc {
struct StereoModel {
  std::string key,name,left,right,disparity,confidence,normalization;
  cv::Size network,content;int padTop=0,padBottom=0;float confidenceThreshold=0;
};
[[nodiscard]] StereoModel ModelSettings(const std::string& model,cv::Size network={});
[[nodiscard]] std::vector<float> StereoInput(const cv::Mat& image,const StereoModel& model);
// Compatibility for the original C-Fast profile.
[[nodiscard]] std::vector<float> StereoInput(const cv::Mat& image,cv::Size network_size={640,384});
// Outputs are disparity in rectified 640x360 pixels, never metres. No filling of invalid confidence holes.
[[nodiscard]] cv::Mat StereoOutput(const cv::Mat& disparity,const cv::Mat& confidence,const StereoModel& model);
struct StereoInference {cv::Mat disparity;double gpuMs=0.;cv::Mat points,depth,leftRect;double prepareMs=0.,postMs=0.;};
class StereoEngine {
 public:
  explicit StereoEngine(const std::string& path,const std::string& model="cfast",const std::string& plugin="");
  ~StereoEngine();
  StereoEngine(const StereoEngine&)=delete;StereoEngine& operator=(const StereoEngine&)=delete;
  StereoEngine(StereoEngine&&)=delete;StereoEngine& operator=(StereoEngine&&)=delete;
  [[nodiscard]] cv::Size networkSize() const noexcept;
  [[nodiscard]] const StereoModel& settings() const noexcept;
  [[nodiscard]] StereoInference infer(const cv::Mat& left,const cv::Mat& right);
  void configurePacked(const std::array<cv::Mat,4>& maps,const cv::Mat& q,const cv::Mat& leftR);
  [[nodiscard]] StereoInference inferPacked(const cv::Mat& usbPacked, bool hostPoints=true, bool hostPreview=true, bool hostDisparity=true);
  // Diagnostic copy of fused model input, for comparison with the CPU reference.
  [[nodiscard]] std::array<std::vector<float>,2> packedInputs() const;
 private:
  struct Impl;std::unique_ptr<Impl> m_impl;
};
}
#endif
