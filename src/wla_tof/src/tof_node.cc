#include "wla_tof/protocol.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
constexpr std::array<const char*,4> NAMES{"front_left","front_right","rear_left","rear_right"};
sensor_msgs::msg::PointField Field(const char* name,uint32_t offset,uint8_t type) {
  sensor_msgs::msg::PointField f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;return f;
}
struct Channel {
  std::optional<Frame> frame;
  Clock::time_point received{};
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
    (void)PreviewPoints(Frame{},m_fov);
    for(size_t i=0;i<4;++i) {
      const std::string prefix=std::string("/r680/tof/")+NAMES[i];
      m_points[i]=create_publisher<sensor_msgs::msg::PointCloud2>(prefix+"/points_preview",rclcpp::SensorDataQoS());
      m_raw[i]=create_publisher<std_msgs::msg::String>(prefix+"/raw",rclcpp::SensorDataQoS());
    }
    m_status=create_publisher<std_msgs::msg::String>("/r680/tof/status",10);
    reconnect();
    m_poll=create_wall_timer(std::chrono::milliseconds(2),[this]{poll();});
    m_report=create_wall_timer(std::chrono::milliseconds(200),[this]{report();});
    RCLCPP_INFO(get_logger(),"TF4 read-only C++ receiver, uncalibrated optical preview only: %s",m_device.c_str());
  }
 private:
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
        c.frame=f;c.received=Clock::now();++c.frames;latest[f.sensor]=f;
      }
    }
    for(size_t i=0;i<4;++i) if(latest[i]) publish(*latest[i]);
  }
  void publish(const Frame& f) {
    const auto stamp=now();
    const auto points=PreviewPoints(f,m_fov);
    m_channels[f.sensor].valid=points.size();
    const std::string frame=std::string("tof_")+NAMES[f.sensor]+"_preview_optical_frame";
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
    Json raw={{"schema","wla-tof-raw/v1"},{"sensor_id",f.sensor},{"logical_name",NAMES[f.sensor]},
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
      Json channel={{"name",NAMES[i]},{"sensor_id",i},{"frames",c.frames},{"fresh",c.frame && age<=m_stale},
        {"age_s",c.frame?Json(age):Json(nullptr)},{"valid_points_last",c.valid},{"sequence_gaps",c.gaps},
        {"duplicates_rejected",c.duplicate},{"boot_changes",c.restarts},{"firmware_health",m_decoder.health(i)}};
      if(c.frame) {channel["firmware_errors"]=c.frame->errors;channel["firmware_tx_drops"]=c.frame->drops;}
      channels.push_back(channel);
    }
    Json status={{"schema","wla-tof-status/v1"},{"backend","cpp"},{"device",m_device},{"bytes",m_bytes},
      {"serial_error",m_error},{"firmware_diagnostics_last",m_decoder.diagnostics()},
      {"crc_errors",m_decoder.crcErrors()},{"header_errors",m_decoder.headerErrors()},
      {"bad_timing_frames",m_badTiming},{"geometry_calibrated",false},{"physical_channel_mapping_verified",false},
      {"preview_fov_deg",m_fov},{"preview_geometry","equal_angular_zone_centres; distance_is_optical_Z"},
      {"timestamp_source","host_complete_frame_receive_not_exposure"},{"tf_published",false},
      {"follow_input_enabled",false},{"channels",channels}};
    std_msgs::msg::String msg;msg.data=status.dump();m_status->publish(msg);
  }
  Decoder m_decoder;
  std::unique_ptr<Serial> m_serial;
  std::array<Channel,4> m_channels;
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,4> m_points;
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr,4> m_raw;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_status;
  rclcpp::TimerBase::SharedPtr m_poll,m_report;
  std::string m_device,m_error;double m_fov{},m_stale{};uint64_t m_bytes{},m_badTiming{};
  Clock::time_point m_nextConnect{};
};
}  // namespace wla::tof
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<wla::tof::TofNode>());}
  catch(const std::exception& error) {std::fprintf(stderr,"tof_serial: %s\n",error.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
