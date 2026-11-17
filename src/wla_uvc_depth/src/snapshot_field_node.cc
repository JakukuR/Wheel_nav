#include "wla_uvc_depth/snapshot_field.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
namespace wla::uvc {
using Json=nlohmann::json;
class SnapshotFieldNode final:public rclcpp::Node {
 public:
  SnapshotFieldNode():Node("stereo_snapshot_field"){
    rate_=declare_parameter("update_hz",5.0);
    if(!std::isfinite(rate_)||rate_<1||rate_>10)throw std::invalid_argument("update_hz_out_of_range");
    publisher_=create_publisher<std_msgs::msg::String>("/r680/debug/stereo/field",rclcpp::SensorDataQoS().keep_last(1));
    timer_=create_wall_timer(std::chrono::duration<double>(1./rate_),[this]{tick();});
  }
 private:
  static int64_t stamp(const std_msgs::msg::Header& h){return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;}
  void send(Json j){std_msgs::msg::String m;m.data=j.dump();publisher_->publish(m);}
  void tick(){
    if(!publisher_->get_subscription_count()){depthSub_.reset();infoSub_.reset();depth_.reset();info_.reset();return;}
    if(!depthSub_){
      depthSub_=create_subscription<sensor_msgs::msg::Image>("/camera/stereo/depth/image_raw",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Image::ConstSharedPtr m){depth_=m;});
      infoSub_=create_subscription<sensor_msgs::msg::CameraInfo>("/camera/stereo/depth/camera_info",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){info_=m;});
    }
    Json j={{"schema","stereo-snapshot-field/v1"},{"available",false},{"frame","sunplus_left_rect_optical_frame"},{"temporal_fusion",false},{"motion_compensated",false},{"navigation_connected",false},{"ground_removed",false},{"person_removed",false},{"update_hz_limit",rate_}};
    auto invalid=[&](const char* reason){j["available"]=false;j["reason"]=reason;send(std::move(j));};
    if(!depth_||!info_){invalid("waiting_for_depth_and_info");return;}
    const auto& d=*depth_;const auto& k=*info_;const auto ns=stamp(d.header);const double age=(now().nanoseconds()-ns)*1e-9;
    j["stamp"]=ns*1e-9;j["source_age_ms"]=age*1000;
    if(ns<=0||age<-.02||age>.30){invalid("stale_depth");return;}
    if(d.header.frame_id!="sunplus_left_rect_optical_frame"||k.header.frame_id!=d.header.frame_id||d.width!=k.width||d.height!=k.height||std::abs(stamp(k.header)-ns)>100000000LL){invalid("info_frame_size_or_time_mismatch");return;}
    if(d.width<2||d.height<2||d.width>1920||d.height>1080||d.encoding!="16UC1"||d.is_bigendian||d.step<d.width*2||d.step%2||d.data.size()!=size_t(d.step)*d.height){invalid("invalid_depth_layout");return;}
    if(k.distortion_model!="plumb_bob"||k.d.size()!=5){invalid("requires_rectified_camera");return;}
    for(double v:k.d)if(v!=0){invalid("requires_rectified_camera");return;}
    for(double v:k.k)if(!std::isfinite(v)){invalid("invalid_intrinsics");return;}
    if(k.k[0]<=1||k.k[4]<=1||k.k[0]>10000||k.k[4]>10000||std::abs(k.k[2])>2*d.width||std::abs(k.k[5])>2*d.height||k.k[1]!=0||k.k[3]!=0||k.k[6]!=0||k.k[7]!=0||k.k[8]!=1){invalid("invalid_intrinsics");return;}
    if(ns==lastStamp_)return; // Never republish an old snapshot as a new observation.
    lastStamp_=ns;
    const auto begin=std::chrono::steady_clock::now();
    cv::Mat depth(d.height,d.width,CV_16UC1,const_cast<uint8_t*>(d.data.data()),d.step);
    cv::Matx33d intrinsic;std::copy(k.k.begin(),k.k.end(),intrinsic.val);
    const auto& g=field_.update(depth,intrinsic);
    Json slices=Json::array(),points=Json::array();size_t known=0,occupied=0;
    for(size_t i=0;i<g.state.size();++i){known+=g.state[i]!=0;occupied+=g.state[i]==2;}
    // Integer voxel indices save bandwidth; browser reconstructs camera coordinates.
    const size_t step=std::max(size_t(1),(occupied+3999)/4000);size_t ordinal=0;
    for(int z=0;z<g.nz;++z)for(int y=0;y<g.ny;++y)for(int x=0;x<g.nx;++x)if(g.state[g.index(x,y,z)]==2&&ordinal++%step==0)points.push_back(Json::array({x,y,z}));
    for(const int y:{15,19,23}){Json state=Json::array(),distance=Json::array();
      for(int z=0;z<g.nz;++z)for(int x=0;x<g.nx;++x){auto i=g.index(x,y,z);state.push_back(g.state[i]);if(!g.state[i])distance.push_back(nullptr);else distance.push_back(int(std::lround(g.distance[i]*1000)));}
      slices.push_back({{"y_m",g.y0+(y+.5F)*g.resolution},{"state",std::move(state)},{"distance_mm",std::move(distance)}});
    }
    j["available"]=true;j["reason"]="current_frame_only";j["resolution_m"]=g.resolution;j["shape"]={g.nx,g.ny,g.nz};j["origin_m"]={g.x0,g.y0,g.z0};j["max_distance_m"]=g.maxDistance;
    j["occupied_voxels"]=occupied;j["observed_voxels"]=known;j["surface_voxels"]=std::move(points);j["slices"]=std::move(slices);
    j["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    j["source_age_ms"]=(now().nanoseconds()-ns)*1e-6;
    if(j["source_age_ms"].get<double>()>300){invalid("compute_exceeded_age_limit");return;}
    send(std::move(j));
  }
  double rate_;int64_t lastStamp_=0;SnapshotField field_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<wla::uvc::SnapshotFieldNode>());}catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("snapshot_field"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
