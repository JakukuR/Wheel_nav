// Kalo debug camera stream: D455 RGB -> JPEG CompressedImage for the browser.
// Subscribes the camera only while someone subscribes the output, keeps at
// most max_rate frames per second (by camera stamp) and encodes on the Orin
// NVJPG block through GStreamer (appsrc ! nvjpegenc ! appsink); the software
// jpegenc element is the fallback. OpenCV is used for colour conversion only:
// linking its imgcodecs (libjpeg-turbo) into this process makes nvjpegenc's
// libjpeg calls resolve to the wrong library ("JPEG parameter struct mismatch").
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

class GstJpeg
{
public:
  GstJpeg(const std::string & element, int width, int height, int quality)
  : width_(width), height_(height)
  {
    const std::string description =
      "appsrc name=src is-live=true format=time caps=video/x-raw,format=I420,width=" +
      std::to_string(width) + ",height=" + std::to_string(height) + ",framerate=0/1 "
      "! " + element + " quality=" + std::to_string(quality) +
      " ! appsink name=sink sync=false max-buffers=1";
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error) {
      error_ = error->message;
      g_error_free(error);
      return;
    }
    src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      error_ = "pipeline would not start";
    }
  }

  ~GstJpeg()
  {
    if (pipeline_) {gst_element_set_state(pipeline_, GST_STATE_NULL);}
    if (src_) {gst_object_unref(src_);}
    if (sink_) {gst_object_unref(sink_);}
    if (pipeline_) {gst_object_unref(pipeline_);}
  }

  bool ok() const {return error_.empty() && src_ && sink_;}
  const std::string & error() const {return error_;}
  bool fits(int width, int height) const {return width == width_ && height == height_;}

  bool encode(const cv::Mat & i420, std::vector<uint8_t> & jpeg)
  {
    const size_t size = i420.total() * i420.elemSize();
    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(buffer, 0, i420.data, size);
    if (gst_app_src_push_buffer(GST_APP_SRC(src_), buffer) != GST_FLOW_OK) {return false;}
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 200 * GST_MSECOND);
    if (!sample) {return false;}
    GstMapInfo map;
    GstBuffer * out = gst_sample_get_buffer(sample);
    const bool mapped = gst_buffer_map(out, &map, GST_MAP_READ);
    if (mapped) {
      jpeg.assign(map.data, map.data + map.size);
      gst_buffer_unmap(out, &map);
    }
    gst_sample_unref(sample);
    return mapped && !jpeg.empty();
  }

private:
  int width_, height_;
  GstElement * pipeline_{nullptr};
  GstElement * src_{nullptr};
  GstElement * sink_{nullptr};
  std::string error_;
};

}  // namespace

class KaloJpeg : public rclcpp::Node
{
public:
  KaloJpeg()
  : Node("kalo_jpeg")
  {
    input_topic_ = declare_parameter("input_topic", std::string("/r680/d455/color/image_raw"));
    const auto output_topic = declare_parameter("output_topic", std::string("/r680/debug/color/compressed"));
    const double max_rate = declare_parameter("max_rate", 10.0);
    quality_ = static_cast<int>(declare_parameter("quality", 80L));
    backend_ = declare_parameter("backend", std::string("auto"));  // auto | nvjpeg | cpu
    min_period_ns_ = static_cast<int64_t>(1e9 / std::max(max_rate, 0.1)) - 5'000'000;  // 5 ms stamp jitter margin
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic, rclcpp::SensorDataQoS().keep_last(1));
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {follow_demand();});
  }

private:
  // Subscribe the camera only while the output has subscribers.
  void follow_demand()
  {
    const bool wanted = publisher_->get_subscription_count() > 0;
    if (wanted && !subscription_) {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(input_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(*msg);});
      RCLCPP_INFO(get_logger(), "viewer connected: subscribing %s", input_topic_.c_str());
    } else if (!wanted && subscription_) {
      subscription_.reset();
      last_stamp_ns_ = 0;
      RCLCPP_INFO(get_logger(), "no viewers: camera unsubscribed");
    }
  }

  void on_image(const sensor_msgs::msg::Image & msg)
  {
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (last_stamp_ns_ && stamp - last_stamp_ns_ < min_period_ns_) {return;}
    int code_ycrcb;
    if (msg.encoding == "rgb8") {code_ycrcb = cv::COLOR_RGB2YCrCb;}
    else if (msg.encoding == "bgr8") {code_ycrcb = cv::COLOR_BGR2YCrCb;}
    else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported encoding %s", msg.encoding.c_str());
      return;
    }
    if (msg.width % 2 || msg.height % 2 || msg.step < msg.width * 3 || msg.data.size() < msg.step * msg.height) {return;}
    const cv::Mat image(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step);
    sensor_msgs::msg::CompressedImage out;
    out.header = msg.header;
    out.format = "jpeg";
    if (!encode(image, code_ycrcb, out.data)) {return;}
    last_stamp_ns_ = stamp;
    publisher_->publish(out);
  }

  bool encode(const cv::Mat & image, int code_ycrcb, std::vector<uint8_t> & jpeg)
  {
    if (!encoder_ || !encoder_->fits(image.cols, image.rows)) {
      const bool hardware = backend_ != "cpu" && !hw_failed_;
      encoder_ = std::make_unique<GstJpeg>(hardware ? "nvjpegenc" : "jpegenc", image.cols, image.rows, quality_);
      if (!encoder_->ok() && hardware) {
        RCLCPP_WARN(get_logger(), "nvjpegenc unavailable (%s); using software jpegenc", encoder_->error().c_str());
        hw_failed_ = true;
        encoder_ = std::make_unique<GstJpeg>("jpegenc", image.cols, image.rows, quality_);
      }
      if (!encoder_->ok()) {
        RCLCPP_ERROR(get_logger(), "no JPEG encoder: %s", encoder_->error().c_str());
        encoder_.reset();
        return false;
      }
      RCLCPP_INFO(get_logger(), "encoding %dx%d with %s, quality %d", image.cols, image.rows,
        backend_ != "cpu" && !hw_failed_ ? "NVJPG (nvjpegenc)" : "CPU (jpegenc)", quality_);
    }
    to_i420(image, code_ycrcb);
    return encoder_->encode(i420_, jpeg);
  }

  // Full-range (JFIF) Y'CbCr packed as I420. cvtColor's *2YUV_I420 codes produce
  // limited-range video levels, which a JPEG decoder shows as washed out.
  void to_i420(const cv::Mat & image, int code_ycrcb)
  {
    const int w = image.cols, h = image.rows;
    cv::cvtColor(image, ycrcb_, code_ycrcb);
    cv::split(ycrcb_, planes_);
    i420_.create(h * 3 / 2, w, CV_8UC1);
    planes_[0].copyTo(cv::Mat(h, w, CV_8UC1, i420_.data));
    cv::resize(planes_[2], cv::Mat(h / 2, w / 2, CV_8UC1, i420_.data + w * h), cv::Size(w / 2, h / 2), 0, 0, cv::INTER_AREA);
    cv::resize(planes_[1], cv::Mat(h / 2, w / 2, CV_8UC1, i420_.data + w * h * 5 / 4), cv::Size(w / 2, h / 2), 0, 0, cv::INTER_AREA);
  }

  std::string input_topic_, backend_;
  int quality_;
  int64_t min_period_ns_, last_stamp_ns_{0};
  bool hw_failed_{false};
  cv::Mat i420_, ycrcb_;
  std::vector<cv::Mat> planes_;
  std::unique_ptr<GstJpeg> encoder_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KaloJpeg>());
  rclcpp::shutdown();
  return 0;
}
