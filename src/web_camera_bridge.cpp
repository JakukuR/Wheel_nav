#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class WebCameraBridge : public rclcpp::Node {
public:
  WebCameraBridge() : Node("r680_web_camera") {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/r680/d455/color/image_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/r680_nav/d455/color/compressed");
    output_rate_ = std::max(0.5, declare_parameter<double>("output_rate", 3.0));
    jpeg_quality_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int64_t>("jpeg_quality", 65), 30, 95));
    max_width_ = static_cast<int>(std::max<int64_t>(
      160, declare_parameter<int64_t>("max_width", 640)));
    output_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(1));
    manager_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&WebCameraBridge::manage_subscription, this));
  }

private:
  void manage_subscription() {
    if (output_->get_subscription_count() > 0 && !input_) {
      input_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&WebCameraBridge::on_image, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Web camera stream active at %.1f Hz", output_rate_);
    } else if (output_->get_subscription_count() == 0 && input_) {
      input_.reset();
      RCLCPP_INFO(get_logger(), "Web camera stream idle");
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / output_rate_);
    if (last_encode_.time_since_epoch().count() != 0 && now - last_encode_ < period) {
      return;
    }
    last_encode_ = now;
    try {
      cv::Mat source;
      cv::Mat converted;
      if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
        source = cv::Mat(msg->height, msg->width, CV_8UC3,
          const_cast<unsigned char *>(msg->data.data()), msg->step);
        if (msg->encoding == "rgb8") {
          cv::cvtColor(source, converted, cv::COLOR_RGB2BGR);
        }
      } else if (msg->encoding == "rgba8" || msg->encoding == "bgra8") {
        source = cv::Mat(msg->height, msg->width, CV_8UC4,
          const_cast<unsigned char *>(msg->data.data()), msg->step);
        cv::cvtColor(source, converted,
          msg->encoding == "rgba8" ? cv::COLOR_RGBA2BGR : cv::COLOR_BGRA2BGR);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Unsupported Web camera encoding: %s", msg->encoding.c_str());
        return;
      }
      cv::Mat frame = converted.empty() ? source : converted;
      cv::Mat resized;
      if (frame.cols > max_width_) {
        const double scale = static_cast<double>(max_width_) / frame.cols;
        cv::resize(frame, resized, cv::Size(), scale, scale, cv::INTER_AREA);
        frame = resized;
      }
      std::vector<unsigned char> encoded;
      if (!cv::imencode(".jpg", frame, encoded,
          {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
        return;
      }
      sensor_msgs::msg::CompressedImage out;
      out.header = msg->header;
      out.format = "jpeg";
      out.data = std::move(encoded);
      output_->publish(out);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Web camera encode failed: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double output_rate_;
  int jpeg_quality_;
  int max_width_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr output_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr manager_;
  std::chrono::steady_clock::time_point last_encode_{};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebCameraBridge>());
  rclcpp::shutdown();
  return 0;
}
