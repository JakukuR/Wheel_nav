#include "feature_visualizer.hpp"
#include <cassert>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("feature_preview_software_test");
  using Image = sensor_msgs::msg::Image;
  Image::ConstSharedPtr received;
  auto sub = node->create_subscription<Image>("/r680_nav/vio/features/image",
    rclcpp::SensorDataQoS().keep_last(1), [&](Image::ConstSharedPtr msg) { received = msg; });
  {
    wla_vio::FeatureVisualizer preview(*node);
    std::array<Image::ConstSharedPtr, 2> images;
    for (size_t c = 0; c < 2; ++c) {
      auto msg = std::make_shared<Image>();
      msg->header.stamp.sec = 10;
      msg->header.frame_id = "software_fixture";
      msg->width = 32; msg->height = 24; msg->step = 32; msg->encoding = "mono8";
      msg->data.resize(32 * 24, 40 + c * 10);
      images[c] = msg;
    }
    const std::vector<wla_vio::FeatureObservation> points{{1, 10, 8, 0}, {1, 8, 8, 1}};
    auto wait_image = [&](int64_t observation_stamp) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (!received && std::chrono::steady_clock::now() < end) {
        preview.submit(images, points, observation_stamp);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      assert(received);
    };
    wait_image(10000000000LL);
    assert(received->header.stamp.sec == 10 && received->header.frame_id == "software_fixture");
    assert(received->encoding == "rgb8" && received->width == 64);
    assert(received->data[(8 * 64 + 10) * 3] == 75);  // green observed feature
    received.reset();
    // Let any queued delivery finish before checking a different frame.
    rclcpp::spin_some(node);
    received.reset();
    wait_image(9000000000LL);
    assert(received->data[(8 * 64 + 10) * 3] == 40);  // old observations not overlaid
  }
  node.reset();
  rclcpp::shutdown();
}
