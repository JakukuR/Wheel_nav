#ifndef WLA_TOF_PROTOCOL_H
#define WLA_TOF_PROTOCOL_H
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <string>
#include <vector>

namespace wla::tof {
struct Frame {
  uint8_t version{}, sensor{}, flags{}, stream{};
  uint32_t boot{}, sequence{}, checks{}, errors{}, drops{};
  uint64_t detectedUs{}, readUs{}, notReadyUs{}, txUs{};
  std::array<int16_t,64> distance{};
  std::array<uint16_t,64> sigma{};
  std::array<uint8_t,64> status{}, targets{};
};
struct Point { float x{}, y{}, z{}, distance{}; uint16_t sigma{}, zone{}; };
[[nodiscard]] uint32_t Crc32(std::span<const uint8_t> data) noexcept;
[[nodiscard]] std::vector<Point> PreviewPoints(const Frame& frame, double fov_deg);
class Decoder {
 public:
  Decoder() = default;
  ~Decoder() = default;
  Decoder(const Decoder&) = delete;
  Decoder& operator=(const Decoder&) = delete;
  Decoder(Decoder&&) = delete;
  Decoder& operator=(Decoder&&) = delete;
  [[nodiscard]] std::vector<Frame> feed(std::span<const uint8_t> data);
  [[nodiscard]] uint64_t crcErrors() const noexcept { return m_crcErrors; }
  [[nodiscard]] uint64_t headerErrors() const noexcept { return m_headerErrors; }
  [[nodiscard]] const std::string& health(size_t id) const { return m_health.at(id); }
  [[nodiscard]] const std::deque<std::string>& diagnostics() const noexcept { return m_diagnostics; }
 private:
  void noise(uint8_t byte);
  std::vector<uint8_t> m_buffer;
  std::string m_line;
  std::array<std::string,4> m_health;
  std::deque<std::string> m_diagnostics;
  uint64_t m_crcErrors{}, m_headerErrors{};
};
}  // namespace wla::tof
#endif
