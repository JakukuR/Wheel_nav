#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace wla::uvc {
struct JpegFrame {std::vector<uint8_t> bytes;int64_t monotonicNs=0;bool kernelTimestamp=false;};
// Nonblocking V4L2 MJPEG acquisition only. No decoding or ROS copies here.
class DirectCapture {
 public:
  explicit DirectCapture(const std::string& device);
  ~DirectCapture();
  DirectCapture(const DirectCapture&)=delete;DirectCapture& operator=(const DirectCapture&)=delete;
  std::shared_ptr<JpegFrame> latest(int timeoutMs=100);
 private:struct Impl;std::unique_ptr<Impl> m_impl;
};
}
