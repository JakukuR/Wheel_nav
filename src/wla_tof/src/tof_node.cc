#include "wla_tof/protocol.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <algorithm>
#include <set>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace wla::tof {
namespace {
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
// Firmware labels for sensor_id 0..3 (LPn GPIO10..13). Not physical positions.
constexpr std::array<const char*,4> FIRMWARE_LABELS{"front_left","front_right","rear_left","rear_right"};
constexpr std::array<const char*,4> MOUNTS{"front_left","front_right","rear_left","rear_right"};
using Mat3=std::array<double,9>;  // row-major
Mat3 Rpy(double r,double p,double y) {  // R=Rz(y)Ry(p)Rx(r), URDF/CAD convention
  const double cr=std::cos(r),sr=std::sin(r),cp=std::cos(p),sp=std::sin(p),cy=std::cos(y),sy=std::sin(y);
  return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr, sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr, -sp,cp*sr,cp*cr};
}
// Optical (x right, y down, z = boresight) -> mount link (x = boresight, y left, z up).
constexpr Mat3 MOUNT_FROM_OPTICAL{0,0,1, -1,0,0, 0,-1,0};
Mat3 Mul(const Mat3& a,const Mat3& b) {
  Mat3 m{};for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) m[3*i+j]+=a[3*i+k]*b[3*k+j];return m;
}
struct Mount { std::string name; std::array<double,3> xyz{}, rpy{}; Mat3 rotation{}; };
sensor_msgs::msg::PointField Field(const char* name,uint32_t offset,uint8_t type) {
  sensor_msgs::msg::PointField f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;return f;
}
struct Channel {
  std::optional<Frame> frame;
  Clock::time_point received{};
  int64_t receivedNs{};
  uint64_t frames{}, duplicate{}, gaps{}, restarts{}, valid{};
};
class Serial {
 public:
  explicit Serial(const std::string& device) {
    m_fd=::open(device.c_str(),O_RDONLY|O_NONBLOCK|O_NOCTTY|O_CLOEXEC);
    if(m_fd<0) throw std::runtime_error("serial_open: "+std::string(std::strerror(errno)));
    try {
      if(flock(m_fd,LOCK_EX|LOCK_NB)<0) throw std::runtime_error("serial_has_other_owner");
      if(ioctl(m_fd,TIOCEXCL)<0) throw std::runtime_error("serial_exclusive_failed");
      m_exclusive=true;
      if(tcgetattr(m_fd,&m_original)<0) throw std::runtime_error("serial_getattr");
      m_restore=true;
      auto options=m_original;cfmakeraw(&options);
      options.c_cflag&=~(HUPCL|CRTSCTS|PARENB|CSTOPB|CSIZE);
      options.c_cflag|=CS8|CLOCAL|CREAD;
      cfsetispeed(&options,B921600);cfsetospeed(&options,B921600);
      options.c_cc[VMIN]=0;options.c_cc[VTIME]=0;
      if(tcsetattr(m_fd,TCSANOW,&options)<0) throw std::runtime_error("serial_setattr");
    } catch(...) { close();throw; }
  }
  ~Serial() { close(); }
  Serial(const Serial&)=delete;Serial& operator=(const Serial&)=delete;
  Serial(Serial&&)=delete;Serial& operator=(Serial&&)=delete;
  [[nodiscard]] ssize_t read(std::span<uint8_t> bytes) const { return ::read(m_fd,bytes.data(),bytes.size()); }
  [[nodiscard]] bool disconnected() const {
    pollfd descriptor{m_fd,POLLIN,0};
    return ::poll(&descriptor,1,0)>0 && (descriptor.revents&(POLLHUP|POLLERR|POLLNVAL));
  }
 private:
  void close() noexcept {
    if(m_fd<0) return;
    if(m_restore) tcsetattr(m_fd,TCSANOW,&m_original);
    if(m_exclusive) ioctl(m_fd,TIOCNXCL);
    ::close(m_fd);m_fd=-1;
  }
  int m_fd{-1};bool m_restore{},m_exclusive{};termios m_original{};
};
}
class TofNode final: public rclcpp::Node {
 public:
  TofNode():Node("tof_serial") {
    if constexpr(std::endian::native!=std::endian::little) throw std::runtime_error("little_endian_required");
    m_device=declare_parameter<std::string>("device","/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_28:84:85:9E:C4:68-if00");
    m_fov=declare_parameter<double>("preview_fov_deg",45.);
    m_stale=declare_parameter<double>("stale_after_s",.3);
    if(m_stale<=0 || m_stale>1 || !std::isfinite(m_stale)) throw std::invalid_argument("invalid_stale_after_s");
    m_minDistance=declare_parameter<double>("min_distance_m",.15);
    m_maxDistance=declare_parameter<double>("max_distance_m",4.);
    (void)PreviewPoints(Frame{},m_fov,m_minDistance,m_maxDistance);
    m_base=declare_parameter<std::string>("base_frame","base_Link");
    // sensor_id 0..3 -> physical mount. Default = firmware labels; the robot config overrides it.
    const auto names=declare_parameter<std::vector<std::string>>("sensor_mounts",{MOUNTS.begin(),MOUNTS.end()});
    if(names.size()!=4 || std::set<std::string>(names.begin(),names.end()).size()!=4) throw std::invalid_argument("sensor_mounts_need_4_unique");
    m_mappingSource=declare_parameter<std::string>("mapping_source","firmware_labels_unverified");
    m_geometrySource=declare_parameter<std::string>("geometry_source","none");
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for(size_t i=0;i<4;++i) {
      if(std::find(MOUNTS.begin(),MOUNTS.end(),names[i])==MOUNTS.end()) throw std::invalid_argument("unknown_mount: "+names[i]);
      auto& m=m_mount[i];m.name=names[i];
      const auto xyz=declare_parameter<std::vector<double>>("mount_"+m.name+"_xyz",{0.,0.,0.});
      const auto rpy=declare_parameter<std::vector<double>>("mount_"+m.name+"_rpy",{0.,0.,0.});
      if(xyz.size()!=3 || rpy.size()!=3) throw std::invalid_argument("mount_pose_needs_3_values: "+m.name);
      std::copy(xyz.begin(),xyz.end(),m.xyz.begin());std::copy(rpy.begin(),rpy.end(),m.rpy.begin());
      m.rotation=Mul(Rpy(m.rpy[0],m.rpy[1],m.rpy[2]),MOUNT_FROM_OPTICAL);
      const std::string prefix="/r680/tof/"+m.name;
      m_points[i]=create_publisher<sensor_msgs::msg::PointCloud2>(prefix+"/points",rclcpp::SensorDataQoS());
      m_raw[i]=create_publisher<std_msgs::msg::String>(prefix+"/raw",rclcpp::SensorDataQoS());
      transforms.push_back(Transform(m_base,"tof_"+m.name+"_link",m.xyz,m.rpy));
      transforms.push_back(Transform("tof_"+m.name+"_link",OpticalFrame(i),{0.,0.,0.},{-M_PI/2,0.,-M_PI/2}));
    }
    m_tf=std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    m_tf->sendTransform(transforms);
    m_merged=create_publisher<sensor_msgs::msg::PointCloud2>("/r680/tof/points_raw",rclcpp::SensorDataQoS());
    m_status=create_publisher<std_msgs::msg::String>("/r680/tof/status",10);
    reconnect();
    m_poll=create_wall_timer(std::chrono::milliseconds(2),[this]{poll();});
    m_report=create_wall_timer(std::chrono::milliseconds(200),[this]{report();});
    m_mergeTimer=create_wall_timer(std::chrono::milliseconds(66),[this]{publishMerged();});
    RCLCPP_INFO(get_logger(),"TF4 read-only receiver %s; mounts %s/%s/%s/%s (%s); geometry %s; base %s",m_device.c_str(),
      m_mount[0].name.c_str(),m_mount[1].name.c_str(),m_mount[2].name.c_str(),m_mount[3].name.c_str(),
      m_mappingSource.c_str(),m_geometrySource.c_str(),m_base.c_str());
  }
 private:
  std::string OpticalFrame(size_t i) const { return "tof_"+m_mount[i].name+"_optical_frame"; }
  geometry_msgs::msg::TransformStamped Transform(const std::string& parent,const std::string& child,
      const std::array<double,3>& xyz,const std::array<double,3>& rpy) {
    geometry_msgs::msg::TransformStamped t;t.header.stamp=now();t.header.frame_id=parent;t.child_frame_id=child;
    t.transform.translation.x=xyz[0];t.transform.translation.y=xyz[1];t.transform.translation.z=xyz[2];
    const double hr=rpy[0]/2,hp=rpy[1]/2,hy=rpy[2]/2;
    const double cr=std::cos(hr),sr=std::sin(hr),cp=std::cos(hp),sp=std::sin(hp),cy=std::cos(hy),sy=std::sin(hy);
    t.transform.rotation.x=sr*cp*cy-cr*sp*sy;t.transform.rotation.y=cr*sp*cy+sr*cp*sy;
    t.transform.rotation.z=cr*cp*sy-sr*sp*cy;t.transform.rotation.w=cr*cp*cy+sr*sp*sy;
    return t;
  }
  // All fresh channels in base_frame: x/y/z, distance_m, sigma_mm, zone, sensor_id (point_step 24).
  void publishMerged() {
    const auto now_steady=Clock::now();
    size_t total=0;std::array<bool,4> use{};int64_t oldest_ns=now().nanoseconds();
    for(size_t i=0;i<4;++i) {
      const auto& c=m_channels[i];
      use[i]=c.frame && std::chrono::duration<double>(now_steady-c.received).count()<=m_stale;
      if(use[i]) {total+=m_last[i].size();oldest_ns=std::min(oldest_ns,c.receivedNs);}
    }
    if(!std::any_of(use.begin(),use.end(),[](bool b){return b;})) return;  // no data: no fake empty cloud
    using F=sensor_msgs::msg::PointField;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp=rclcpp::Time(oldest_ns);cloud.header.frame_id=m_base;cloud.height=1;cloud.width=total;
    cloud.fields={Field("x",0,F::FLOAT32),Field("y",4,F::FLOAT32),Field("z",8,F::FLOAT32),Field("distance_m",12,F::FLOAT32),
      Field("sigma_mm",16,F::UINT16),Field("zone",18,F::UINT16),Field("sensor_id",20,F::UINT8)};
    cloud.point_step=24;cloud.row_step=cloud.point_step*cloud.width;cloud.is_bigendian=false;cloud.is_dense=true;
    cloud.data.assign(cloud.row_step,0);
    size_t k=0;
    for(size_t i=0;i<4;++i) {
      if(!use[i]) continue;
      const auto& m=m_mount[i];const auto& R=m.rotation;
      for(const auto& p:m_last[i]) {
        const float x=float(R[0]*p.x+R[1]*p.y+R[2]*p.z+m.xyz[0]);
        const float y=float(R[3]*p.x+R[4]*p.y+R[5]*p.z+m.xyz[1]);
        const float z=float(R[6]*p.x+R[7]*p.y+R[8]*p.z+m.xyz[2]);
        auto* d=cloud.data.data()+24*k++;const uint8_t id=uint8_t(i);
        std::memcpy(d,&x,4);std::memcpy(d+4,&y,4);std::memcpy(d+8,&z,4);std::memcpy(d+12,&p.distance,4);
        std::memcpy(d+16,&p.sigma,2);std::memcpy(d+18,&p.zone,2);std::memcpy(d+20,&id,1);
      }
    }
    m_merged->publish(cloud);
  }
  void reconnect() {
    if(Clock::now()<m_nextConnect) return;
    m_nextConnect=Clock::now()+std::chrono::seconds(1);
    try {m_serial=std::make_unique<Serial>(m_device);m_error.clear();}
    catch(const std::exception& e) {m_error=e.what();}
  }
  void poll() {
    if(!m_serial) {reconnect();return;}
    if(m_serial->disconnected()) {m_error="serial_disconnected";m_serial.reset();return;}
    std::array<std::optional<Frame>,4> latest;
    std::array<uint8_t,4096> bytes;
    for(int iteration=0;iteration<16;++iteration) {
      const auto count=m_serial->read(bytes);
      if(count<0) {
        if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR) break;
        m_error="serial_read: "+std::string(std::strerror(errno));m_serial.reset();break;
      }
      if(count==0) break;
      m_bytes+=uint64_t(count);
      for(const auto& f:m_decoder.feed(std::span(bytes.data(),size_t(count)))) {
        auto& c=m_channels[f.sensor];
        if(f.readUs==0 || (f.version==2 && (f.txUs<f.readUs || f.detectedUs>f.readUs))) {++m_badTiming;continue;}
        if(c.frame && c.frame->boot==f.boot) {
          if(f.sequence<=c.frame->sequence || f.readUs<=c.frame->readUs) {++c.duplicate;continue;}
          c.gaps+=uint64_t(f.sequence-c.frame->sequence-1);
        } else if(c.frame) ++c.restarts;
        c.frame=f;c.received=Clock::now();c.receivedNs=now().nanoseconds();++c.frames;latest[f.sensor]=f;
      }
    }
    for(size_t i=0;i<4;++i) if(latest[i]) publish(*latest[i]);
  }
  void publish(const Frame& f) {
    const auto stamp=now();
    const auto points=PreviewPoints(f,m_fov,m_minDistance,m_maxDistance);
    m_channels[f.sensor].valid=points.size();
    m_last[f.sensor]=points;
    const std::string frame=OpticalFrame(f.sensor);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp=stamp;cloud.header.frame_id=frame;cloud.height=1;cloud.width=points.size();
    using F=sensor_msgs::msg::PointField;
    cloud.fields={Field("x",0,F::FLOAT32),Field("y",4,F::FLOAT32),Field("z",8,F::FLOAT32),Field("distance_m",12,F::FLOAT32),Field("sigma_mm",16,F::UINT16),Field("zone",18,F::UINT16)};
    cloud.point_step=20;cloud.row_step=cloud.point_step*cloud.width;cloud.is_bigendian=false;cloud.is_dense=true;
    cloud.data.resize(cloud.row_step);
    for(size_t i=0;i<points.size();++i) {
      const auto& p=points[i];auto* data=cloud.data.data()+20*i;
      std::memcpy(data,&p.x,4);std::memcpy(data+4,&p.y,4);std::memcpy(data+8,&p.z,4);
      std::memcpy(data+12,&p.distance,4);std::memcpy(data+16,&p.sigma,2);std::memcpy(data+18,&p.zone,2);
    }
    m_points[f.sensor]->publish(cloud);
    Json raw={{"schema","wla-tof-raw/v1"},{"sensor_id",f.sensor},{"firmware_label",FIRMWARE_LABELS[f.sensor]},{"mount",m_mount[f.sensor].name},
      {"host_receive_stamp_ns",stamp.nanoseconds()},{"timestamp_source","host_complete_frame_receive_not_exposure"},
      {"boot_id",f.boot},{"sequence",f.sequence},{"protocol_version",f.version},{"flags",f.flags},
      {"detected_us",f.detectedUs},{"read_us",f.readUs},{"last_not_ready_us",f.notReadyUs},{"tx_start_us",f.txUs},
      {"errors",f.errors},{"tx_drops",f.drops},{"distance_mm",f.distance},{"sigma_mm",f.sigma},{"status",f.status},{"targets",f.targets}};
    std_msgs::msg::String msg;msg.data=raw.dump();m_raw[f.sensor]->publish(msg);
  }
  void report() {
    Json channels=Json::array();
    for(size_t i=0;i<4;++i) {
      const auto& c=m_channels[i];
      const double age=c.frame?std::chrono::duration<double>(Clock::now()-c.received).count():-1.;
      Json channel={{"name",m_mount[i].name},{"mount",m_mount[i].name},{"firmware_label",FIRMWARE_LABELS[i]},{"sensor_id",i},{"frames",c.frames},{"fresh",c.frame && age<=m_stale},
        {"age_s",c.frame?Json(age):Json(nullptr)},{"valid_points_last",c.valid},{"sequence_gaps",c.gaps},
        {"duplicates_rejected",c.duplicate},{"boot_changes",c.restarts},{"firmware_health",m_decoder.health(i)}};
      if(c.frame) {channel["firmware_errors"]=c.frame->errors;channel["firmware_tx_drops"]=c.frame->drops;}
      channels.push_back(channel);
    }
    Json status={{"schema","wla-tof-status/v1"},{"backend","cpp"},{"device",m_device},{"bytes",m_bytes},
      {"serial_error",m_error},{"firmware_diagnostics_last",m_decoder.diagnostics()},
      {"crc_errors",m_decoder.crcErrors()},{"header_errors",m_decoder.headerErrors()},
      {"bad_timing_frames",m_badTiming},{"geometry_calibrated",false},{"geometry_source",m_geometrySource},
      {"physical_channel_mapping",m_mappingSource},{"base_frame",m_base},{"merged_topic","/r680/tof/points_raw"},{"min_distance_m",m_minDistance},{"max_distance_m",m_maxDistance},
      {"preview_fov_deg",m_fov},{"preview_geometry","equal_angular_zone_centres; distance_is_optical_Z"},
      {"timestamp_source","host_complete_frame_receive_not_exposure"},{"tf_published",true},
      {"follow_input_enabled",false},{"channels",channels}};
    std_msgs::msg::String msg;msg.data=status.dump();m_status->publish(msg);
  }
  Decoder m_decoder;
  std::unique_ptr<Serial> m_serial;
  std::array<Channel,4> m_channels;
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,4> m_points;
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr,4> m_raw;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_status;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_merged;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf;
  std::array<Mount,4> m_mount;
  std::array<std::vector<Point>,4> m_last;
  std::string m_base,m_mappingSource,m_geometrySource;
  rclcpp::TimerBase::SharedPtr m_poll,m_report,m_mergeTimer;
  std::string m_device,m_error;double m_fov{},m_stale{},m_minDistance{},m_maxDistance{};uint64_t m_bytes{},m_badTiming{};
  Clock::time_point m_nextConnect{};
};
}  // namespace wla::tof
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<wla::tof::TofNode>());}
  catch(const std::exception& error) {std::fprintf(stderr,"tof_serial: %s\n",error.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
