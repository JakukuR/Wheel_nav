#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

namespace wla_vio {

// Rendering only: IDs and coordinates come from cuVSLAM State::observations.
struct FeatureObservation { uint64_t id; float u, v; uint32_t camera_index; };
struct MonoView {
  const uint8_t * data;
  size_t size;
  uint32_t width, height, step;
};
struct FeatureOverlay {
  uint32_t width{}, height{};
  std::vector<uint8_t> rgb;
  size_t observations{}, stereo_matches{}, temporal_tracks{};
};

class FeatureRenderer {
  using Pixel = std::array<int, 2>;
  using Key = std::pair<uint32_t, uint64_t>;
  using Color = std::array<uint8_t, 3>;
  std::map<Key, Pixel> previous_;
  int64_t previous_stamp_{};
  uint32_t previous_width_{}, previous_height_{};

  static void pixel(FeatureOverlay & out, int x, int y, Color color) {
    if (x < 0 || y < 0 || x >= static_cast<int>(out.width) || y >= static_cast<int>(out.height)) return;
    const auto at = (static_cast<size_t>(y) * out.width + x) * 3;
    std::copy(color.begin(), color.end(), out.rgb.begin() + at);
  }
  static void line(FeatureOverlay & out, Pixel a, Pixel b, Color color) {
    const int dx = std::abs(b[0] - a[0]), sx = a[0] < b[0] ? 1 : -1;
    const int dy = -std::abs(b[1] - a[1]), sy = a[1] < b[1] ? 1 : -1;
    int error = dx + dy;
    for (;;) {
      pixel(out, a[0], a[1], color);
      if (a == b) break;
      const int twice = error * 2;
      if (twice >= dy) { error += dy; a[0] += sx; }
      if (twice <= dx) { error += dx; a[1] += sy; }
    }
  }

public:
  FeatureOverlay render(const std::array<MonoView, 2> & images,
    const std::vector<FeatureObservation> & observations, int64_t stamp) {
    for (const auto & img : images) {
      if (!img.data || !img.width || !img.height || img.width > 4096 || img.height > 4096 ||
        img.step < img.width || img.size < static_cast<size_t>(img.step) * img.height) {
        throw std::invalid_argument("invalid feature preview image");
      }
    }
    FeatureOverlay out;
    out.width = images[0].width + images[1].width;
    out.height = std::max(images[0].height, images[1].height);
    out.rgb.resize(static_cast<size_t>(out.width) * out.height * 3);
    for (size_t camera = 0; camera < 2; ++camera) {
      const auto & img = images[camera];
      const uint32_t offset = camera == 0 ? 0 : images[0].width;
      for (uint32_t y = 0; y < img.height; ++y) {
        for (uint32_t x = 0; x < img.width; ++x) {
          const auto gray = img.data[static_cast<size_t>(y) * img.step + x];
          pixel(out, offset + x, y, {gray, gray, gray});
        }
      }
    }
    if (stamp <= previous_stamp_ || stamp - previous_stamp_ > 1000000000LL ||
      out.width != previous_width_ || out.height != previous_height_) previous_.clear();
    std::map<Key, Pixel> current;
    for (const auto & obs : observations) {
      if (obs.camera_index > 1 || !std::isfinite(obs.u) || !std::isfinite(obs.v)) continue;
      const auto & img = images[obs.camera_index];
      if (obs.u < 0 || obs.v < 0 || obs.u >= img.width || obs.v >= img.height) continue;
      current[{obs.camera_index, obs.id}] = {static_cast<int>(obs.u) +
        (obs.camera_index == 0 ? 0 : static_cast<int>(images[0].width)), static_cast<int>(obs.v)};
    }
    // All points are drawn; cap long stereo lines to keep dense scenes readable.
    for (const auto & item : current) {
      if (item.first.first != 0) continue;
      const auto right = current.find({1, item.first.second});
      if (right != current.end()) {
        if (out.stereo_matches < 100) line(out, item.second, right->second, {255, 190, 45});
        ++out.stereo_matches;
      }
    }
    for (const auto & item : current) {
      const auto old = previous_.find(item.first);
      if (old != previous_.end()) {
        line(out, old->second, item.second, {55, 215, 255});
        ++out.temporal_tracks;
      }
    }
    for (const auto & item : current) {
      for (int d = -2; d <= 2; ++d) {
        pixel(out, item.second[0] + d, item.second[1], {75, 255, 130});
        pixel(out, item.second[0], item.second[1] + d, {75, 255, 130});
      }
    }
    out.observations = current.size();
    previous_ = std::move(current);
    previous_stamp_ = stamp;
    previous_width_ = out.width;
    previous_height_ = out.height;
    return out;
  }
};
}  // namespace wla_vio
