#ifndef WLA_UVC_DEPTH_STEREO_ENGINE_H
#define WLA_UVC_DEPTH_STEREO_ENGINE_H
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>
namespace wla::uvc {
// Rectified 640x360 BGR/mono -> optional 320x180 resize -> replicate pad to multiple32 -> ImageNet CHW.
[[nodiscard]] std::vector<float> StereoInput(const cv::Mat& image,cv::Size network_size={640,384});
struct StereoInference {cv::Mat disparity;double gpuMs=0.;};
class StereoEngine {
 public:
  explicit StereoEngine(const std::string& path);
  ~StereoEngine();
  StereoEngine(const StereoEngine&)=delete;StereoEngine& operator=(const StereoEngine&)=delete;
  StereoEngine(StereoEngine&&)=delete;StereoEngine& operator=(StereoEngine&&)=delete;
  [[nodiscard]] cv::Size networkSize() const noexcept;
  [[nodiscard]] StereoInference infer(const cv::Mat& left,const cv::Mat& right);
 private:
  struct Impl;std::unique_ptr<Impl> m_impl;
};
}
#endif
