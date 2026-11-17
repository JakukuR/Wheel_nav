#include "wla_uvc_depth/ground_map.h"
#include <algorithm>
#include <cmath>
#include <random>
namespace wla::uvc {
GroundMap MakeGroundMap(const std::vector<GroundPoint> &points,
                        const cv::Vec3d &priorNormal, double priorOffset) {
  GroundMap out;
  out.occupancy.assign(out.width * out.height, -1);
  out.classes.assign(out.width * out.height, 0);
  auto fail = [&](const char *s) {
    out.reason = s;
    return out;
  };
  const double norm = cv::norm(priorNormal);
  if (!std::isfinite(norm) || std::abs(norm - 1) > .01 ||
      !std::isfinite(priorOffset) || priorOffset <= 0 || priorOffset > 1)
    return fail("invalid_support_plane");
  const auto prior = priorNormal / norm;
  std::vector<cv::Vec3d> fit;
  for (const auto &p : points) {
    if (!std::isfinite(cv::norm(p.body)) || cv::norm(p.body) > 100)
      return fail("invalid_point");
    if (p.candidate && p.body[0] > .15 && p.body[0] < 2.5)
      fit.push_back(p.body);
  }
  if (fit.size() < 400)
    return fail("insufficient_ground_candidates");
  std::mt19937 rng(42);
  std::uniform_int_distribution<size_t> pick(0, fit.size() - 1);
  std::vector<size_t> best, current;
  // Apply the same support envelope before scoring candidates: a dominant
  // slanted surface must not hide a smaller, support-consistent floor.
  for (int iter = 0; iter < 160; ++iter) {
    const auto a = fit[pick(rng)], b = fit[pick(rng)], c = fit[pick(rng)];
    auto n = (b - a).cross(c - a);
    const double s = cv::norm(n);
    if (s < 1e-7)
      continue;
    n /= s;
    if (n.dot(prior) < 0)
      n = -n;
    if (n.dot(prior) < std::cos(5 * CV_PI / 180))
      continue;
    const double h = -n.dot(a);
    if (std::abs(h - priorOffset) > .04)
      continue;
    current.clear();
    for (size_t i = 0; i < fit.size(); ++i)
      if (std::abs(n.dot(fit[i]) + h) < .015)
        current.push_back(i);
    if (current.size() > best.size())
      best = current;
  }
  if (best.size() < 400)
    return fail("no_supported_plane");
  cv::Vec3d mean{};
  for (auto i : best)
    mean += fit[i];
  mean /= double(best.size());
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (auto i : best) {
    const auto p = fit[i] - mean;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        covariance(r, c) += p[r] * p[c];
  }
  cv::Mat values, vectors;
  cv::eigen(cv::Mat(covariance), values, vectors);
  cv::Vec3d n(vectors.at<double>(2, 0), vectors.at<double>(2, 1),
              vectors.at<double>(2, 2));
  if (n.dot(prior) < 0)
    n = -n;
  out.normal = n;
  out.offset = -n.dot(mean);
  out.angleDeg = std::acos(std::clamp(n.dot(prior), -1., 1.)) * 180 / CV_PI;
  out.offsetDelta = out.offset - priorOffset;
  out.ratio = double(best.size()) / fit.size();
  cv::Vec3d lo = fit[best[0]], hi = lo;
  double sq = 0;
  std::vector<double> residuals;
  for (auto i : best) {
    const auto &p = fit[i];
    sq += std::pow(n.dot(p) + out.offset, 2);
    residuals.push_back(prior.dot(p) + priorOffset);
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
  out.rms = std::sqrt(sq / best.size());
  std::nth_element(residuals.begin(), residuals.begin() + residuals.size() / 2,
                   residuals.end());
  out.priorResidual = residuals[residuals.size() / 2];
  if (out.angleDeg > 5 || std::abs(out.offsetDelta) > .04)
    return fail("observed_plane_disagrees_with_support");
  if (out.ratio < .30 || out.rms > .012 || hi[0] - lo[0] < .35 ||
      hi[1] - lo[1] < .25 || lo[0] > .70)
    return fail("plane_coverage_or_residual_failed");
  const auto x = (cv::Vec3d(1, 0, 0) - n * n[0]);
  if (cv::norm(x) < .9)
    return fail("invalid_heading_projection");
  const auto ex = x / cv::norm(x), ey = n.cross(ex);
  out.bodyToMap =
      cv::Matx33d(ex[0], ex[1], ex[2], ey[0], ey[1], ey[2], n[0], n[1], n[2]);
  std::vector<int> groundHits(out.classes.size(), 0);
  for (const auto &input : points) {
    auto p = out.bodyToMap * input.body;
    p[2] += out.offset;
    const int xcell = int(std::floor((p[0] - out.x0) / out.resolution)),
              ycell = int(std::floor((p[1] - out.y0) / out.resolution));
    if (xcell < 0 || ycell < 0 || xcell >= out.width || ycell >= out.height)
      continue;
    const size_t i = ycell * out.width + xcell;
    const auto q = cv::Vec3f(p);
    if (std::abs(p[2]) <= .015) {
      ++groundHits[i];
      out.ground.push_back(q);
    } else if (p[2] >= .03) {
      out.classes[i] = 2;
      out.obstacles.push_back(q);
    } // Keep overhead surfaces: no assumed vehicle envelope yet.
    else if (p[2] <= -.03) {
      if (out.classes[i] != 2)
        out.classes[i] = 3;
      out.uncertain.push_back(q);
    } else {
      if (out.classes[i] < 2)
        out.classes[i] = 4;
      out.uncertain.push_back(q);
    }
  }
  for (size_t i = 0; i < out.classes.size(); ++i) {
    if (out.classes[i] >= 2)
      out.occupancy[i] = 100;
    else if (groundHits[i] >= 3) {
      out.classes[i] = 1;
      out.occupancy[i] = 0;
    }
  }
  out.available = true;
  out.reason = "single_frame_ground_obstacle_preview";
  return out;
}
} // namespace wla::uvc
