#include "wla_uvc_depth/core.h"
#include "wla_uvc_depth/stereo_engine.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <condition_variable>
#include <chrono>
#include <mutex>
#include <thread>
#include <map>
#include <deque>
#include <optional>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <atomic>
namespace wla::uvc {
using Image=sensor_msgs::msg::Image;using Msg=Image::ConstSharedPtr;
using Json=nlohmann::json;using Clock=std::chrono::steady_clock;
int64_t Stamp(const std_msgs::msg::Header& h){return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;}
cv::Mat Array(const Msg& m){
  if(m->encoding!="bgr8"||m->step<m->width*3||m->data.size()!=size_t(m->height)*m->step)throw std::runtime_error("expected_bgr8");
  return {int(m->height),int(m->width),CV_8UC3,const_cast<uint8_t*>(m->data.data()),m->step};
}
Image Message(const cv::Mat& a,const std_msgs::msg::Header& h,const std::string& encoding){
  Image m;m.header=h;m.width=a.cols;m.height=a.rows;m.encoding=encoding;m.is_bigendian=false;m.step=a.cols*a.elemSize();m.data.resize(size_t(m.step)*m.height);
  for(int y=0;y<a.rows;++y)std::memcpy(m.data.data()+size_t(y)*m.step,a.ptr(y),m.step);
  return m;
}
struct Args {std::string engine,rig;double rate=10;bool color=true;};
Args Parse(const std::vector<std::string>& args){
  Args a;for(size_t i=1;i<args.size();++i){
    const auto key=args[i];if(key=="--no-color"){a.color=false;continue;}
    if(i+1==args.size())throw std::runtime_error("uvc_stereo_trt --engine file --calibration rig.yaml [--max-rate 10] [--no-color]");
    const auto v=args[++i];if(key=="--engine")a.engine=v;else if(key=="--calibration")a.rig=v;else if(key=="--max-rate")a.rate=std::stod(v);else throw std::runtime_error("unknown_option:"+key);
  }
  if(a.engine.empty()||a.rig.empty()||!std::isfinite(a.rate)||a.rate<=0||a.rate>15)throw std::runtime_error("invalid_stereo_arguments");
  return a;
}
Options GeometryOptions(){Options o;o.algorithm="none";o.rgbMode="full";return o;}
class StereoNode final:public rclcpp::Node {
 public:
  explicit StereoNode(Args args):Node("uvc_stereo_network"),m_args(std::move(args)),m_calibration(LoadCalibration(m_args.rig)),m_geometry(m_calibration,GeometryOptions()),m_engine(m_args.engine){
    cv::setNumThreads(1);const auto qos=rclcpp::SensorDataQoS().keep_last(2);
    m_depth=create_publisher<Image>("/camera/stereo/depth/image_raw",qos);
    m_disparity=create_publisher<Image>("/camera/stereo/disparity/image_raw",qos);
    m_preview=create_publisher<Image>("/camera/stereo/depth/preview",qos);
    m_info=create_publisher<sensor_msgs::msg::CameraInfo>("/camera/stereo/depth/camera_info",qos);
    m_points=create_publisher<sensor_msgs::msg::PointCloud2>("/camera/stereo/points",qos);
    m_colored=create_publisher<sensor_msgs::msg::PointCloud2>("/camera/stereo/points_color",qos);
    m_aligned=create_publisher<Image>("/camera/stereo/aligned_depth_to_color/image_raw",qos);
    m_alignedPreview=create_publisher<Image>("/camera/stereo/aligned_depth_to_color/preview",qos);
    m_status=create_publisher<std_msgs::msg::String>("/camera/stereo/status",1);
    m_left=create_subscription<Image>("/camera/sunplus/left/image_raw",qos,[this](Msg m){receive(m,0);});
    m_right=create_subscription<Image>("/camera/sunplus/right/image_raw",qos,[this](Msg m){receive(m,1);});
    if(m_args.color){
      m_rgb=create_subscription<Image>("/camera/processed/color/image_raw",qos,[this](Msg m){receive(m,2);});
      m_rgbInfo=create_subscription<sensor_msgs::msg::CameraInfo>("/camera/processed/color/camera_info",qos,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){
        if(m->width!=640||m->height!=360||m->header.frame_id!="dicota_color_optical_frame"||m->distortion_model!="plumb_bob"||m->d.size()!=5)return;
        for(double d:m->d)if(d!=0.)return;
        const auto& k=m_geometry.colorK();for(int y=0;y<3;++y)for(int x=0;x<3;++x)if(!std::isfinite(m->k[y*3+x])||std::abs(m->k[y*3+x]-k.at<double>(y,x))>1e-5)return;
        std::lock_guard lock(m_mutex);m_rgbInfos[Stamp(m->header)]=m;while(m_rgbInfos.size()>10)m_rgbInfos.erase(m_rgbInfos.begin());
      });
    }
    // Internal rectification only; no changes to the robot's CAD/control/mount TF.
    m_tf=std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;tf.header.stamp=now();tf.header.frame_id="sunplus_left_optical_frame";tf.child_frame_id="sunplus_left_rect_optical_frame";
    const cv::Mat r=m_geometry.leftR().t();tf2::Matrix3x3 rotation(r.at<double>(0,0),r.at<double>(0,1),r.at<double>(0,2),r.at<double>(1,0),r.at<double>(1,1),r.at<double>(1,2),r.at<double>(2,0),r.at<double>(2,1),r.at<double>(2,2));
    tf2::Quaternion q;rotation.getRotation(q);tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();m_tf->sendTransform(tf);
    m_timer=create_wall_timer(std::chrono::milliseconds(500),[this]{status();});m_worker=std::thread([this]{work();});
  }
  ~StereoNode()override{{std::lock_guard lock(m_mutex);m_closed=true;}m_condition.notify_one();if(m_worker.joinable())m_worker.join();}
  StereoNode(const StereoNode&)=delete;StereoNode& operator=(const StereoNode&)=delete;
  StereoNode(StereoNode&&)=delete;StereoNode& operator=(StereoNode&&)=delete;
 private:
  struct Pair {Msg left,right;};
  bool fresh(const Msg& m)const{const auto age=(now().nanoseconds()-Stamp(m->header))*1e-9;return Stamp(m->header)>0&&age>=-.05&&age<=.3;}
  void receive(const Msg& m,int eye){
    if(m_thermalPause.load()||!fresh(m)||m->encoding!="bgr8")return;
    const std::array<std::string,3> frames={"sunplus_left_optical_frame","sunplus_right_optical_frame","dicota_color_optical_frame"};
    if(m->header.frame_id!=frames[eye]||m->width!=(eye==2?640U:1280U)||m->height!=(eye==2?360U:720U))return;
    std::lock_guard lock(m_mutex);if(eye==2){m_colors.push_back(m);while(m_colors.size()>8)m_colors.pop_front();return;}
    const auto stamp=Stamp(m->header);if(stamp<=m_lastPair)return;
    m_pairs[stamp][eye]=m;while(m_pairs.size()>5)m_pairs.erase(m_pairs.begin());
    for(auto it=m_pairs.rbegin();it!=m_pairs.rend();++it)if(it->second[0]&&it->second[1]){
      m_pending=Pair{it->second[0],it->second[1]};m_lastPair=it->first;m_pairs.erase(m_pairs.begin(),m_pairs.upper_bound(m_lastPair));m_condition.notify_one();break;
    }
  }
  Msg colorFor(int64_t stamp){
    std::lock_guard lock(m_mutex);Msg best;int64_t delta=25000001;
    for(const auto& m:m_colors){const auto d=std::abs(Stamp(m->header)-stamp);if(d<delta&&fresh(m)&&m_rgbInfos.count(Stamp(m->header))){best=m;delta=d;}}
    return best;
  }
  void cloud(const cv::Mat& points,const Msg& left,const Msg& rgb){
    const bool color=rgb&&m_colored->get_subscription_count();
    const bool plain=m_points->get_subscription_count();if(!plain&&!color)return;
    sensor_msgs::msg::PointCloud2 msg;msg.header=left->header;msg.height=1;msg.is_dense=true;
    sensor_msgs::PointCloud2Modifier modify(msg);
    if(color)modify.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,"rgb",1,sensor_msgs::msg::PointField::FLOAT32);
    else modify.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);
    // Stride two only reduces exported cloud density; depth remains full 640x360.
    modify.resize(size_t((points.rows+1)/2)*((points.cols+1)/2));size_t count=0;
    const auto& k=m_geometry.colorK();const cv::Matx44d t(m_calibration.rgbFromLeft);
    const cv::Mat image=color?Array(rgb):cv::Mat();
    const auto zbuffer=color?ProjectDepth(points,k,m_calibration.rgbFromLeft,{640,360}):cv::Mat();
    for(int y=0;y<points.rows;y+=2)for(int x=0;x<points.cols;x+=2){
      const auto p=points.at<cv::Vec3f>(y,x);if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[2]<.15F||p[2]>5.F)continue;
      auto* row=msg.data.data()+count*msg.point_step;std::memcpy(row,p.val,12);
      if(color){uint32_t packed=0x808080;const auto c=t*cv::Vec4d(p[0],p[1],p[2],1);
        if(c[2]>.15&&c[2]<=5){const int u=cvRound(k.at<double>(0,0)*c[0]/c[2]+k.at<double>(0,2)),v=cvRound(k.at<double>(1,1)*c[1]/c[2]+k.at<double>(1,2));
          if(u>=0&&u<640&&v>=0&&v<360&&std::abs(zbuffer.at<uint16_t>(v,u)*.001-c[2])<=.01){const auto bgr=image.at<cv::Vec3b>(v,u);packed=(uint32_t(bgr[2])<<16)|(uint32_t(bgr[1])<<8)|bgr[0];}}
        std::memcpy(row+12,&packed,4);
      }++count;
    }
    modify.resize(count);if(color&&fresh(left)&&fresh(rgb))m_colored->publish(msg);
    if(plain&&fresh(left)){if(color){sensor_msgs::msg::PointCloud2 xyz;xyz.header=msg.header;xyz.height=1;xyz.is_dense=true;sensor_msgs::PointCloud2Modifier mod(xyz);mod.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);mod.resize(count);for(size_t i=0;i<count;++i)std::memcpy(xyz.data.data()+i*xyz.point_step,msg.data.data()+i*msg.point_step,12);if(fresh(left))m_points->publish(xyz);}else if(fresh(left))m_points->publish(msg);}
  }
  void work(){
    auto next=Clock::now();while(true){
      std::optional<Pair> pair;{std::unique_lock lock(m_mutex);m_condition.wait(lock,[this]{return m_closed||m_pending.has_value();});if(m_closed)return;
        if(Clock::now()<next){m_condition.wait_until(lock,next,[this]{return m_closed;});if(m_closed)return;}pair=std::move(m_pending);m_pending.reset();}
      next=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1./m_args.rate));
      if(m_thermalPause.load()||!pair||!fresh(pair->left)||!fresh(pair->right))continue;
      try{
        const auto start=Clock::now();auto [l,r]=m_geometry.rectify(Array(pair->left),Array(pair->right));const auto prediction=m_engine.infer(l,r);
        auto points=m_geometry.pointsFromDisparity(prediction.disparity);
        if(!fresh(pair->left)||!fresh(pair->right)){std::lock_guard lock(m_mutex);m_failure="inference_exceeded_300ms_source_age";continue;}
        auto rgb=colorFor(Stamp(pair->left->header));auto h=pair->left->header;h.frame_id="sunplus_left_rect_optical_frame";
        cv::Mat depth(360,640,CV_16U,cv::Scalar(0));
        const cv::Matx33d rr(m_geometry.leftR());size_t valid=0;
        for(int y=0;y<360;++y)for(int x=0;x<640;++x){const auto p=points.at<cv::Vec3f>(y,x);if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[2]<.15||p[2]>5)continue;const auto rect=rr*cv::Vec3d(p);if(rect[2]>=.15&&rect[2]<=5){depth.at<uint16_t>(y,x)=uint16_t(std::nearbyint(rect[2]*1000));++valid;}}
        if(!valid)throw std::runtime_error("no_valid_stereo_depth");
        if(!fresh(pair->left))continue;
        m_depth->publish(Message(depth,h,"16UC1"));
        sensor_msgs::msg::CameraInfo info;info.header=h;info.width=640;info.height=360;info.distortion_model="plumb_bob";info.d.assign(5,0.);const auto& p=m_geometry.leftP();for(int y=0;y<3;++y)for(int x=0;x<3;++x){info.k[y*3+x]=p.at<double>(y,x);info.p[y*4+x]=p.at<double>(y,x);info.r[y*3+x]=y==x?1:0;}m_info->publish(info);
        if(m_disparity->get_subscription_count()&&fresh(pair->left))m_disparity->publish(Message(prediction.disparity,h,"32FC1"));
        if(m_preview->get_subscription_count()){cv::Mat gray,preview;depth.convertTo(gray,CV_8U,-255./5000,255);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,depth==0);if(fresh(pair->left))m_preview->publish(Message(preview,h,"bgr8"));}
        if(m_aligned->get_subscription_count()||m_alignedPreview->get_subscription_count()){
          auto aligned=ProjectDepth(points,m_geometry.colorK(),m_calibration.rgbFromLeft,{640,360});auto ah=pair->left->header;ah.frame_id="dicota_color_optical_frame";
          if(m_aligned->get_subscription_count()&&fresh(pair->left))m_aligned->publish(Message(aligned,ah,"16UC1"));
          if(m_alignedPreview->get_subscription_count()){cv::Mat gray,preview;aligned.convertTo(gray,CV_8U,-255./5000,255);cv::applyColorMap(gray,preview,cv::COLORMAP_TURBO);preview.setTo(0,aligned==0);if(fresh(pair->left))m_alignedPreview->publish(Message(preview,ah,"bgr8"));}
        }
        cloud(points,pair->left,rgb);
        std::lock_guard lock(m_mutex);++m_count;m_last=Clock::now();m_failure.clear();m_metrics={{"gpu_ms",prediction.gpuMs},{"pipeline_ms",std::chrono::duration<double,std::milli>(Clock::now()-start).count()},{"valid_depth_fraction",double(valid)/depth.total()},{"stereo_stamp_ns",Stamp(pair->left->header)},{"rgb_color_available",bool(rgb)},{"rgb_stamp_ns",rgb?Stamp(rgb->header):0},{"host_rgb_skew_ms",rgb?(Stamp(rgb->header)-Stamp(pair->left->header))*1e-6:0.}};
      }catch(const std::exception& e){std::lock_guard lock(m_mutex);m_failure=e.what();RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what());}
    }
  }
  void status(){
    double cpu_temp=0,gpu_temp=0;std::ifstream cpu("/sys/class/thermal/thermal_zone0/temp"),gpu("/sys/class/thermal/thermal_zone1/temp");
    const bool temperatures_available=bool(cpu>>cpu_temp)&&bool(gpu>>gpu_temp);cpu_temp*=.001;gpu_temp*=.001;
    if(temperatures_available&&std::max(cpu_temp,gpu_temp)>=97.)m_thermalPause.store(true);
    std::lock_guard lock(m_mutex);const bool fresh=!m_thermalPause.load()&&m_count&&std::chrono::duration<double>(Clock::now()-m_last).count()<.5&&m_failure.empty();
    Json j={{"schema","wla-stereo-network/v1"},{"thermal_paused",m_thermalPause.load()},{"temperature_available",temperatures_available},{"cpu_c",cpu_temp},{"gpu_c",gpu_temp},{"thermal_stop_c",97},{"backend","cpp_tensorrt"},{"model","C-Fast-FoundationStereo"},{"engine",m_args.engine},{"calibration_file",m_args.rig},{"depth_source","synchronized_lower_stereo"},{"rgb_role","optional_coloring_only"},{"classical_matcher_enabled",false},{"matching_width",m_engine.networkSize().width},{"matching_height",m_engine.networkSize().width*360/640},{"output_width",640},{"output_height",360},{"network_width",m_engine.networkSize().width},{"network_height",m_engine.networkSize().height},{"padding_top",m_engine.networkSize().width/640.*12},{"padding_bottom",m_engine.networkSize().width/640.*12},{"near_m",.15},{"far_m",5},{"depth_fresh",fresh},{"depth_frames",m_count},{"max_rate",m_args.rate},{"timestamp_source","host_receive_stereo_pair_shared_stamp"},{"color_sync","approximate_host_receive_25ms_not_hardware_exposure"},{"point_cloud_frame","sunplus_left_optical_frame"},{"point_cloud_stride",2},{"ground_filter_enabled",false},{"obstacle_control_connected",false},{"metrics",m_metrics},{"reason",m_thermalPause.load()?"thermal_pause_restart_after_cooling":!m_failure.empty()?m_failure:fresh?"experimental_calibrated_stereo":"waiting_for_fresh_stereo"}};
    std_msgs::msg::String msg;msg.data=j.dump();m_status->publish(msg);
  }
  std::atomic_bool m_thermalPause{false};
  Args m_args;Calibration m_calibration;Core m_geometry;StereoEngine m_engine;
  std::mutex m_mutex;std::condition_variable m_condition;std::thread m_worker;bool m_closed=false;
  std::map<int64_t,std::array<Msg,2>> m_pairs;std::optional<Pair> m_pending;std::deque<Msg> m_colors;
  std::map<int64_t,sensor_msgs::msg::CameraInfo::ConstSharedPtr> m_rgbInfos;int64_t m_lastPair=0;uint64_t m_count=0;Clock::time_point m_last{};std::string m_failure;Json m_metrics=Json::object();
  rclcpp::Publisher<Image>::SharedPtr m_depth,m_disparity,m_preview,m_aligned,m_alignedPreview;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_info;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points,m_colored;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_status;rclcpp::Subscription<Image>::SharedPtr m_left,m_right,m_rgb;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_rgbInfo;rclcpp::TimerBase::SharedPtr m_timer;std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf;
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);int result=0;try{auto node=std::make_shared<wla::uvc::StereoNode>(wla::uvc::Parse(rclcpp::remove_ros_arguments(argc,argv)));rclcpp::spin(node);}catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=1;}if(rclcpp::ok())rclcpp::shutdown();return result;}
