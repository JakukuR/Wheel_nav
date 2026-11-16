#include "wla_uvc_depth/core.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <iostream>
#include <cmath>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace wla::uvc {
using Image=sensor_msgs::msg::Image;
using Msg=Image::ConstSharedPtr;
using Clock=std::chrono::steady_clock;
using Json=nlohmann::json;
int64_t Stamp(const Msg& m) {return static_cast<int64_t>(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;}
cv::Mat Array(const Msg& m) {
  if(m->encoding!="bgr8"||m->step<m->width*3||m->data.size()!=static_cast<size_t>(m->height)*m->step)throw std::runtime_error("expected_bgr8_raw_image");
  // The shared ROS message remains owned by the work item for this view's lifetime.
  return cv::Mat(static_cast<int>(m->height),static_cast<int>(m->width),CV_8UC3,const_cast<uint8_t*>(m->data.data()),m->step);
}
Image Message(const cv::Mat& array,const std_msgs::msg::Header& header,const std::string& encoding) {
  Image m;m.header=header;m.width=array.cols;m.height=array.rows;m.encoding=encoding;m.is_bigendian=false;
  m.step=static_cast<uint32_t>(array.cols*array.elemSize());m.data.resize(static_cast<size_t>(m.step)*m.height);
  for(int i=0;i<array.rows;++i)std::memcpy(m.data.data()+static_cast<size_t>(i)*m.step,array.ptr(i),m.step);
  return m;
}
struct Arguments {std::string calibration;Options options;double skewMs=25.,maxRate=10.;};
Arguments Parse(const std::vector<std::string>& args) {
  Arguments a;a.options.rgbMode="full"; // Preserve the existing public pipeline default.
  for(size_t i=1;i<args.size();++i) {
    const auto key=args[i];
    if(key=="--help")throw std::runtime_error("uvc_depth_cpp --calibration rig.yaml --rgb-mode crop|full --algorithm sgbm|bm --disparities 128 --block-size 5 --max-skew-ms 25 --max-rate 10");
    if(i+1==args.size())throw std::runtime_error("missing_option_value: "+key);
    const auto value=args[++i];
    if(key=="--calibration")a.calibration=value;
    else if(key=="--rgb-mode")a.options.rgbMode=value;
    else if(key=="--algorithm")a.options.algorithm=value;
    else if(key=="--disparities")a.options.disparities=std::stoi(value);
    else if(key=="--block-size")a.options.block=std::stoi(value);
    else if(key=="--max-skew-ms")a.skewMs=std::stod(value);
    else if(key=="--max-rate")a.maxRate=std::stod(value);
    else throw std::runtime_error("unknown_option: "+key);
  }
  return a;
}
class Pipeline final:public rclcpp::Node {
 public:
  explicit Pipeline(Arguments args):Node("uvc_rgb_stereo_depth"),m_args(std::move(args)) {
    cv::setNumThreads(1);
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    m_args.calibration=declare_parameter("calibration_file",m_args.calibration,d);
    m_args.options.rgbMode=declare_parameter("rgb_mode",m_args.options.rgbMode,d);
    m_args.options.algorithm=declare_parameter("algorithm",m_args.options.algorithm,d);
    m_args.options.disparities=declare_parameter("disparities",m_args.options.disparities,d);
    m_args.options.block=declare_parameter("block_size",m_args.options.block,d);
    m_args.options.nearM=declare_parameter("near_m",.15,d);m_args.options.farM=declare_parameter("far_m",5.,d);
    m_args.skewMs=declare_parameter("max_skew_ms",m_args.skewMs,d);m_args.maxRate=declare_parameter("max_rate",m_args.maxRate,d);
    if(!std::isfinite(m_args.skewMs)||m_args.skewMs<=0||m_args.skewMs>35||!std::isfinite(m_args.maxRate)||m_args.maxRate<=0||m_args.maxRate>30||
      (m_args.options.rgbMode!="crop"&&m_args.options.rgbMode!="full"))throw std::runtime_error("invalid_mode_rate_or_sync_window");
    m_output=m_args.options.rgbMode=="crop"?cv::Size(640,480):cv::Size(640,360);
    if(!m_args.calibration.empty())m_core=std::make_unique<Core>(LoadCalibration(m_args.calibration),m_args.options);
    const auto qos=rclcpp::SensorDataQoS().keep_last(2);
    m_colorPub=create_publisher<Image>("/camera/processed/color/image_raw",qos);
    m_infoPub=create_publisher<sensor_msgs::msg::CameraInfo>("/camera/processed/color/camera_info",qos);
    m_depthPub=create_publisher<Image>("/camera/processed/aligned_depth_to_color/image_raw",qos);
    m_disparityPub=create_publisher<Image>("/camera/processed/disparity/image_raw",qos);
    m_disparityPreview=create_publisher<Image>("/camera/processed/disparity/preview",qos);
    m_depthPreview=create_publisher<Image>("/camera/processed/aligned_depth_to_color/preview",qos);
    m_leftPub=create_publisher<Image>("/camera/processed/left/image_rect",qos);
    m_rightPub=create_publisher<Image>("/camera/processed/right/image_rect",qos);
    m_statusPub=create_publisher<std_msgs::msg::String>("/camera/processed/status",1);
    m_rgbSub=create_subscription<Image>("/camera/dicota/color/image_raw",qos,[this](Msg m){receive(std::move(m),2);});
    m_leftSub=create_subscription<Image>("/camera/sunplus/left/image_raw",qos,[this](Msg m){receive(std::move(m),0);});
    m_rightSub=create_subscription<Image>("/camera/sunplus/right/image_raw",qos,[this](Msg m){receive(std::move(m),1);});
    m_timer=create_wall_timer(std::chrono::milliseconds(500),[this]{status();});
    m_worker=std::thread([this]{work();});
    RCLCPP_INFO(get_logger(),"native C++ %s, calibration=%s, RGB=%dx%d, match=640x360",m_args.options.algorithm.c_str(),m_args.calibration.empty()?"missing (2D only)":m_args.calibration.c_str(),m_output.width,m_output.height);
  }
  ~Pipeline() override {
    {std::lock_guard lock(m_mutex);m_closed=true;}m_condition.notify_all();if(m_worker.joinable())m_worker.join();
  }
  Pipeline(const Pipeline&)=delete;Pipeline& operator=(const Pipeline&)=delete;
  Pipeline(Pipeline&&)=delete;Pipeline& operator=(Pipeline&&)=delete;
 private:
  struct Pair {Msg left,right,rgb;};
  bool fresh(const Msg& m) const {const double age=(now().nanoseconds()-Stamp(m))*1e-9;return Stamp(m)>0&&age>=-.05&&age<=.3;}
  void receive(Msg m,int eye) {
    const std::array<std::string,3> frames={"sunplus_left_optical_frame","sunplus_right_optical_frame","dicota_color_optical_frame"};
    const auto size=eye==2?cv::Size(1920,1080):(m_core?m_core->stereoSize():cv::Size(1280,720));
    if(m->header.frame_id!=frames[eye]||m->width!=static_cast<unsigned>(size.width)||m->height!=static_cast<unsigned>(size.height)||!fresh(m))return;
    std::lock_guard lock(m_mutex);
    if(eye==2) {
      m_pendingColor=m;m_colors.push_back(m);while(m_colors.size()>6)m_colors.pop_front();m_condition.notify_one();
    } else if(m_core&&Stamp(m)>m_lastPair) {
      m_pairs[Stamp(m)][eye]=m;while(m_pairs.size()>6)m_pairs.erase(m_pairs.begin());
    }
    if(!m_core)return;
    for(auto it=m_pairs.rbegin();it!=m_pairs.rend();++it) {
      if(!it->second[0]||!it->second[1])continue;
      Msg rgb;int64_t best=INT64_MAX;
      for(const auto& candidate:m_colors)if(Stamp(candidate)>m_lastRgb) {
        const auto delta=std::abs(Stamp(candidate)-it->first);if(delta<best){best=delta;rgb=candidate;}
      }
      if(!rgb||best>m_args.skewMs*1e6)continue;
      m_pendingPair=Pair{it->second[0],it->second[1],rgb};m_lastPair=it->first;m_lastRgb=Stamp(rgb);
      m_pairs.erase(m_pairs.begin(),m_pairs.upper_bound(m_lastPair));m_condition.notify_one();break;
    }
  }
  void publishColor(const Msg& m) {
    if(!fresh(m)||Stamp(m)<=m_lastColorStamp)return;
    const auto pixels=m_core?m_core->color(Array(m)):RawColor(Array(m),m_output);
    if(!fresh(m))return;
    sensor_msgs::msg::CameraInfo info;info.header=m->header;info.width=m_output.width;info.height=m_output.height;
    if(m_core) {
      info.distortion_model="plumb_bob";info.d.assign(5,0.);const auto& k=m_core->colorK();
      for(int r=0;r<3;++r)for(int c=0;c<3;++c){info.k[r*3+c]=k.at<double>(r,c);info.p[r*4+c]=k.at<double>(r,c);info.r[r*3+c]=r==c?1.:0.;}
    }
    m_colorPub->publish(Message(pixels,m->header,"bgr8"));m_infoPub->publish(info);m_lastColorStamp=Stamp(m);
    std::lock_guard lock(m_mutex);m_lastColor=Clock::now();++m_colorCount;
  }
  void work() {
    auto next=Clock::now();
    for(;;) {
      Msg color;std::optional<Pair> pair;
      {std::unique_lock lock(m_mutex);m_condition.wait(lock,[this]{return m_closed||m_pendingColor||m_pendingPair.has_value();});if(m_closed)return;
        if(Clock::now()<next){m_condition.wait_until(lock,next,[this]{return m_closed;});if(m_closed)return;}
        color=std::move(m_pendingColor);pair=std::move(m_pendingPair);m_pendingPair.reset();}
      next=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1./m_args.maxRate));
      try {
        if(color)publishColor(color);
        if(!pair||!fresh(pair->left)||!fresh(pair->right)||!fresh(pair->rgb))continue;
        const auto start=Clock::now();auto result=m_core->depth(Array(pair->left),Array(pair->right));
        if(!fresh(pair->left)||!fresh(pair->right)||!fresh(pair->rgb))continue;
        auto header=pair->left->header;header.frame_id=pair->rgb->header.frame_id;
        m_depthPub->publish(Message(result.depth,header,"16UC1"));
        if(m_disparityPub->get_subscription_count())m_disparityPub->publish(Message(result.disparity,pair->left->header,"32FC1"));
        if(m_leftPub->get_subscription_count())m_leftPub->publish(Message(result.left,pair->left->header,"mono8"));
        if(m_rightPub->get_subscription_count())m_rightPub->publish(Message(result.right,pair->right->header,"mono8"));
        if(m_disparityPreview->get_subscription_count()) {
          cv::Mat gray,preview;result.disparity.convertTo(gray,CV_8U,255./m_args.options.disparities);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,result.disparity<=0);m_disparityPreview->publish(Message(preview,pair->left->header,"bgr8"));
        }
        if(m_depthPreview->get_subscription_count()) {
          cv::Mat gray,preview;result.depth.convertTo(gray,CV_8U,-255./(m_args.options.farM*1000),255);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,result.depth==0);m_depthPreview->publish(Message(preview,header,"bgr8"));
        }
        const double fraction=static_cast<double>(cv::countNonZero(result.depth))/result.depth.total();
        std::lock_guard lock(m_mutex);m_lastDepth=Clock::now();m_failure.clear();++m_depthCount;
        m_metrics={{"compute_ms",std::chrono::duration<double,std::milli>(Clock::now()-start).count()},
          {"host_skew_ms",(Stamp(pair->rgb)-Stamp(pair->left))*1e-6},{"valid_depth_fraction",fraction},
          {"stereo_stamp_ns",Stamp(pair->left)},{"paired_rgb_stamp_ns",Stamp(pair->rgb)}};
      } catch(const std::exception& e) {std::lock_guard lock(m_mutex);m_failure=e.what();RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what());}
    }
  }
  void status() {
    std::lock_guard lock(m_mutex);const auto n=Clock::now();
    const bool color_fresh=m_colorCount&&std::chrono::duration<double>(n-m_lastColor).count()<.3;
    const bool depth_fresh=m_depthCount&&std::chrono::duration<double>(n-m_lastDepth).count()<.3&&m_failure.empty();
    Json j={{"schema","wla-uvc-depth/v1"},{"backend","cpp_opencv"},{"calibrated",bool(m_core)},{"calibration_file",m_args.calibration},
      {"algorithm",m_args.options.algorithm},{"rgb_mode",m_args.options.rgbMode},{"color_width",m_output.width},{"color_height",m_output.height},
      {"matching_width",640},{"matching_height",360},{"depth_enabled",bool(m_core)},{"color_fresh",color_fresh},{"depth_fresh",depth_fresh},
      {"max_host_skew_ms",m_args.skewMs},{"max_rate",m_args.maxRate},{"near_m",m_args.options.nearM},{"far_m",m_args.options.farM},
      {"depth_encoding","16UC1_mm_zero_invalid"},{"timestamp_source","host_capture_receive_not_hardware_exposure"},
      {"cross_camera_sync","approximate_host_receive"},{"mount_tf_published_by_this_node",false},
      {"color_frames",m_colorCount},{"depth_frames",m_depthCount},{"metrics",m_metrics},
      {"reason",!m_failure.empty()?m_failure:!m_core?"missing_calibration_2d_only":!m_depthCount?"waiting_for_synced_frames":!depth_fresh?"stale_depth":"calibrated_software_only"}};
    std_msgs::msg::String message;message.data=j.dump();m_statusPub->publish(message);
  }
  Arguments m_args;cv::Size m_output;std::unique_ptr<Core> m_core;
  std::mutex m_mutex;std::condition_variable m_condition;bool m_closed=false;
  std::thread m_worker;Msg m_pendingColor;std::optional<Pair> m_pendingPair;
  std::deque<Msg> m_colors;std::map<int64_t,std::array<Msg,2>> m_pairs;
  int64_t m_lastPair=0,m_lastRgb=0,m_lastColorStamp=0;
  uint64_t m_colorCount=0,m_depthCount=0;Clock::time_point m_lastColor{},m_lastDepth{};
  std::string m_failure;Json m_metrics=Json::object();
  rclcpp::Publisher<Image>::SharedPtr m_colorPub,m_depthPub,m_disparityPub,m_disparityPreview,m_depthPreview,m_leftPub,m_rightPub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_infoPub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statusPub;
  rclcpp::Subscription<Image>::SharedPtr m_rgbSub,m_leftSub,m_rightSub;rclcpp::TimerBase::SharedPtr m_timer;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int result=0;
  try {auto node=std::make_shared<wla::uvc::Pipeline>(wla::uvc::Parse(rclcpp::remove_ros_arguments(argc,argv)));rclcpp::spin(node);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=1;}
  if(rclcpp::ok())rclcpp::shutdown();
  return result;
}
