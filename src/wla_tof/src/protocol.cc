#include "wla_tof/protocol.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace wla::tof {
namespace {
constexpr std::array<uint8_t,4> MAGIC{'T','F','4','!'};
constexpr std::array<const char*,4> NAMES{"front_left","front_right","rear_left","rear_right"};
uint16_t U16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1])<<8; }
uint32_t U32(const uint8_t* p) { return uint32_t(U16(p)) | uint32_t(U16(p+2))<<16; }
uint64_t U64(const uint8_t* p) { return uint64_t(U32(p)) | uint64_t(U32(p+4))<<32; }
}
uint32_t Crc32(std::span<const uint8_t> data) noexcept {
  uint32_t value=UINT32_MAX;
  for(const auto byte:data) {
    value^=byte;
    for(int bit=0;bit<8;++bit) value=(value>>1)^(0xedb88320U & (0U-(value&1U)));
  }
  return ~value;
}
void Decoder::noise(uint8_t byte) {
  if(byte=='#') m_line.clear();
  if(byte=='\n') {
    for(size_t i=0;i<NAMES.size();++i)
      if(m_line.starts_with(std::string("# HEALTH name=")+NAMES[i]+" ")) m_health[i]=m_line;
    if(m_line.starts_with("# ") && !m_line.starts_with("# HEALTH ")) {
      m_diagnostics.push_back(m_line);
      if(m_diagnostics.size()>32) m_diagnostics.pop_front();
    }
    m_line.clear();
  } else if(byte!='\r') {
    if(byte>=32 && byte<=126 && m_line.size()<256) m_line.push_back(char(byte));
    else m_line.clear();
  }
}
std::vector<Frame> Decoder::feed(std::span<const uint8_t> data) {
  if(data.size()>65536) throw std::invalid_argument("serial_chunk_too_large");
  m_buffer.insert(m_buffer.end(),data.begin(),data.end());
  std::vector<Frame> frames;
  while(true) {
    const auto start=std::search(m_buffer.begin(),m_buffer.end(),MAGIC.begin(),MAGIC.end());
    const auto count=start==m_buffer.end() ? (m_buffer.size()>3?m_buffer.size()-3:0) : size_t(start-m_buffer.begin());
    for(size_t i=0;i<count;++i) noise(m_buffer[i]);
    m_buffer.erase(m_buffer.begin(),m_buffer.begin()+count);
    if(m_buffer.size()<12) break;
    const auto* p=m_buffer.data();
    const size_t size=p[4]==1?444:(p[4]==2?460:0);
    if(size==0 || p[5]>=4 || p[7]!=0 || U16(p+8)!=size || U16(p+10)!=0) {
      ++m_headerErrors; m_buffer.erase(m_buffer.begin()); continue;
    }
    if(m_buffer.size()<size) break;
    if(Crc32(std::span(p,size-4))!=U32(p+size-4)) {
      ++m_crcErrors; m_buffer.erase(m_buffer.begin()); continue;
    }
    Frame f;
    f.version=p[4]; f.sensor=p[5]; f.flags=p[6]; f.boot=U32(p+12); f.sequence=U32(p+16);
    f.checks=U32(p+20); f.errors=U32(p+28); f.drops=U32(p+32); f.stream=p[36];
    f.detectedUs=U64(p+40); f.readUs=U64(p+48);
    if(f.version==2) { f.notReadyUs=U64(p+56); f.txUs=U64(p+64); }
    const size_t offset=f.version==1?56:72;
    for(size_t zone=0;zone<64;++zone) {
      const auto distance=U16(p+offset+zone*2);
      f.distance[zone]=int16_t(distance<32768?int32_t(distance):int32_t(distance)-65536);
      f.sigma[zone]=U16(p+offset+128+zone*2);
      f.status[zone]=p[offset+256+zone]; f.targets[zone]=p[offset+320+zone];
    }
    frames.push_back(f);
    m_buffer.erase(m_buffer.begin(),m_buffer.begin()+size);
  }
  return frames;
}
std::vector<Point> PreviewPoints(const Frame& f,double fov_deg,double min_distance_m,double max_distance_m) {
  if(!std::isfinite(fov_deg) || fov_deg<=0 || fov_deg>=170) throw std::invalid_argument("invalid_preview_fov");
  if(!std::isfinite(min_distance_m) || !std::isfinite(max_distance_m) || min_distance_m<.02 || min_distance_m>=max_distance_m || max_distance_m>4.) throw std::invalid_argument("invalid_distance_limits");
  std::vector<Point> points;
  const double half=fov_deg*std::numbers::pi/360.;
  for(size_t zone=0;zone<64;++zone) {
    const int distance=f.distance[zone];
    if(f.targets[zone]==0 || f.status[zone]!=5 || double(distance)<min_distance_m*1000 || double(distance)>max_distance_m*1000) continue;
    const double z=distance*.001;
    const double ax=((double(zone%8)+.5)/8*2-1)*half;
    const double ay=((double(zone/8)+.5)/8*2-1)*half;
    points.push_back({float(z*std::tan(ax)),float(z*std::tan(ay)),float(z),float(z),f.sigma[zone],uint16_t(zone)});
  }
  return points;
}
}  // namespace wla::tof
