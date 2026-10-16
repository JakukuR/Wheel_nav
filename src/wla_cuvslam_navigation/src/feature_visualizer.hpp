#pragma once

#include "feature_overlay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace wla_vio {

// One pending snapshot, separate rendering thread, no SDK access from this thread.
class FeatureVisualizer {
  using Image = sensor_msgs::msg::Image;
  struct Snapshot {
    std::array<Image::ConstSharedPtr, 2> images;
    std::vector<FeatureObservation> observations;
  };
  rclcpp::Publisher<Image>::SharedPtr publisher_;
  rclcpp::Logger logger_;
  std::chrono::steady_clock::time_point last_{};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::unique_ptr<Snapshot> pending_;
  bool quit_{false};
  std::thread worker_;

  void run() {
    FeatureRenderer renderer;
    for (;;) {
      std::unique_ptr<Snapshot> snapshot;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return quit_ || pending_; });
        if (quit_) return;
        snapshot = std::move(pending_);
      }
      try {
        const auto & pair = snapshot->images;
        std::array<MonoView, 2> views;
        for (size_t i = 0; i < 2; ++i) {
          views[i] = {pair[i]->data.data(), pair[i]->data.size(), pair[i]->width,
            pair[i]->height, pair[i]->step};
        }
        const auto & stamp = pair[0]->header.stamp;
        auto overlay = renderer.render(views, snapshot->observations,
          static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec);
        Image msg;
        msg.header = pair[0]->header;
        msg.encoding = "rgb8";
        msg.width = overlay.width;
        msg.height = overlay.height;
        msg.step = msg.width * 3;
        msg.data = std::move(overlay.rgb);
        publisher_->publish(std::move(msg));
      } catch (const std::exception & ex) {
        // A preview failure must never change estimator health or reset tracking.
        RCLCPP_WARN(logger_, "Feature preview skipped: %s", ex.what());
      }
    }
  }

public:
  explicit FeatureVisualizer(rclcpp::Node & node)
  : publisher_(node.create_publisher<Image>("/r680_nav/vio/features/image",
      rclcpp::SensorDataQoS().keep_last(1))), logger_(node.get_logger()),
    worker_([this] { run(); }) {}
  ~FeatureVisualizer() {
    { std::lock_guard<std::mutex> lock(mutex_); quit_ = true; }
    cv_.notify_one();
    worker_.join();
  }
  template<class Observations>
  void submit(const std::array<Image::ConstSharedPtr, 2> & pair,
    const Observations & observations, int64_t observation_stamp) noexcept {
    try {
      const auto now = std::chrono::steady_clock::now();
      if (publisher_->get_subscription_count() == 0 ||
        now - last_ < std::chrono::milliseconds(200)) return;
      last_ = now;
      auto snapshot = std::make_unique<Snapshot>();
      snapshot->images = pair;  // retain the exact pair passed to Track()
      const auto & stamp = pair[0]->header.stamp;
      const auto frame_stamp = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
      if (observation_stamp == frame_stamp) {
        snapshot->observations.reserve(observations.size());
        for (const auto & obs : observations) {
          snapshot->observations.push_back({obs.id, obs.u, obs.v, obs.camera_index});
        }
      }
      // Never wait for rendering; replacing the pending snapshot drops old preview work.
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (!lock.owns_lock()) return;
      pending_ = std::move(snapshot);
      cv_.notify_one();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(logger_, "Feature snapshot skipped: %s", ex.what());
    }
  }
};
}  // namespace wla_vio
