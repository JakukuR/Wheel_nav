#include "feature_overlay.hpp"
#include <cassert>
#include <fstream>
#include <limits>

int main(int argc, char ** argv) {
  using namespace wla_vio;
  std::vector<uint8_t> left(36 * 24, 18), right(36 * 24, 42);
  std::array<MonoView, 2> images{{{left.data(), left.size(), 32, 24, 36},
    {right.data(), right.size(), 32, 24, 36}}};
  FeatureRenderer renderer;
  const uint64_t large_id = (uint64_t{1} << 60) + 3;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<FeatureObservation> points{{large_id, 10, 8, 0}, {large_id, 8, 8, 1},
    {large_id + 1, 22, 16, 0}, {large_id + 2, 20, 16, 1},
    {9, nan, 0, 0}, {8, -1, 0, 0}, {7, 32, 0, 1}, {6, 5, 5, 2}};
  auto out = renderer.render(images, points, 1000000000LL);
  assert(out.width == 64 && out.height == 24 && out.observations == 4);
  assert(out.stereo_matches == 1 && out.temporal_tracks == 0);
  assert(out.rgb[0] == 18 && out.rgb[32 * 3] == 42);  // padded source rows
  assert(out.rgb[(8 * 64 + 30) * 3] == 255);  // same-ID stereo link
  assert(out.rgb[(16 * 64 + 32) * 3] == 42);  // different IDs must not connect
  points[0].u += 1;
  out = renderer.render(images, points, 1200000000LL);
  assert(out.temporal_tracks == 4);
  out = renderer.render(images, points, 2400000000LL);
  assert(out.temporal_tracks == 0);  // no bridging missing/old frames
  out = renderer.render(images, points, 2300000000LL);
  assert(out.temporal_tracks == 0);  // reset after timestamp reversal
  out = renderer.render(images, {}, 2500000000LL);
  assert(out.observations == 0 && out.stereo_matches == 0 && out.temporal_tracks == 0);
  assert(out.rgb[(8 * 64 + 30) * 3] == 18);  // no stale overlay
  images[0].size = 1;
  bool rejected = false;
  try { renderer.render(images, points, 2600000000LL); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);

  // Optional synthetic fixture for HTTP/UI integration, never a real-camera result.
  if (argc > 1) {
    constexpr uint32_t w = 640, h = 480;
    std::array<std::vector<uint8_t>, 2> data;
    for (auto & pixels : data) pixels.resize(w * h);
    for (uint32_t c = 0; c < 2; ++c) {
      for (uint32_t y = 0; y < h; ++y) for (uint32_t x = 0; x < w; ++x)
        data[c][y * w + x] = 35 + ((x / 40 + y / 40 + c) % 2) * 45 + y / 8;
      images[c] = {data[c].data(), data[c].size(), w, h, w};
    }
    points.clear();
    for (uint64_t i = 0; i < 24; ++i) {
      const float x = 65 + (i % 6) * 95, y = 70 + (i / 6) * 100;
      points.push_back({i, x, y, 0}); points.push_back({i, x - 18, y, 1});
    }
    out = renderer.render(images, points, 3000000000LL);
    for (auto & p : points) { p.u += 12; p.v += 6; }
    out = renderer.render(images, points, 3200000000LL);
    std::ofstream file(argv[1], std::ios::binary);
    file << "P6\n" << out.width << ' ' << out.height << "\n255\n";
    file.write(reinterpret_cast<const char *>(out.rgb.data()), out.rgb.size());
  }
}
