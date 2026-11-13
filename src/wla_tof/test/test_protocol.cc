#include "wla_tof/protocol.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace wla::tof;
namespace {
void Require(bool condition,const char* why) { if(!condition) throw std::runtime_error(why); }
void Put(std::vector<uint8_t>& p,size_t at,uint64_t value,size_t size) {
  for(size_t i=0;i<size;++i) p.at(at+i)=uint8_t(value>>(8*i));
}
std::vector<uint8_t> Packet(int version,int sensor=1) {
  const size_t offset=version==1?56:72;
  std::vector<uint8_t> p(offset+388);
  p[0]='T';p[1]='F';p[2]='4';p[3]='!';p[4]=uint8_t(version);p[5]=uint8_t(sensor);p[6]=8;
  Put(p,8,p.size(),2);Put(p,12,123,4);Put(p,16,456,4);
  Put(p,40,11000,8);Put(p,48,12000,8);
  if(version==2) {Put(p,56,10000,8);Put(p,64,12500,8);}
  for(size_t i=0;i<64;++i) {
    Put(p,offset+2*i,1000,2);Put(p,offset+128+2*i,i,2);
    p[offset+256+i]=5;p[offset+320+i]=1;
  }
  Put(p,p.size()-4,Crc32(std::span(p.data(),p.size()-4)),4);
  return p;
}
}
int main(int argc,char** argv) {
  try {
    const std::string check="123456789";
    Require(Crc32(std::span(reinterpret_cast<const uint8_t*>(check.data()),check.size()))==0xcbf43926U,"known CRC32 vector");
    for(int version:{1,2}) for(int sensor=0;sensor<4;++sensor) {
      auto p=Packet(version,sensor);Decoder d;std::vector<Frame> frames;
      for(const auto byte:p) {auto fs=d.feed(std::span(&byte,1));frames.insert(frames.end(),fs.begin(),fs.end());}
      Require(frames.size()==1 && frames[0].sensor==sensor && frames[0].sequence==456,"split packet decoding");
      Require(frames[0].distance[63]==1000 && frames[0].sigma[63]==63,"payload offsets");
      Require(frames[0].readUs==12000,"MCU timestamp offset");
      Require(frames[0].txUs==(version==1?0:12500),"v2 timestamp offset");
      const auto points=PreviewPoints(frames[0],45.);
      Require(points.size()==64 && points[0].x<0 && points[0].y<0 && points[63].x>0 && points[63].y>0,"optical preview directions");
      Require(points[0].z==1.F && points[63].zone==63,"metric Z and zone id");
      auto f=frames[0];f.distance[0]=-1;f.status[1]=9;f.targets[2]=0;f.distance[3]=4001;f.distance[4]=19;
      Require(PreviewPoints(f,45.).size()==59,"invalid targets must not turn into zero points");
    }
    Decoder d;
    auto bad=Packet(2);bad[100]^=1;
    auto good=Packet(2);
    const std::string health="# HEALTH name=front_left mode=polling online=0 frames=0\r\n";
    std::vector<uint8_t> stream{'x',0,255};stream.insert(stream.end(),bad.begin(),bad.end());
    stream.insert(stream.end(),health.begin(),health.end());stream.insert(stream.end(),good.begin(),good.end());
    auto frames=d.feed(stream);
    Require(frames.size()==1 && d.crcErrors()==1,"CRC reject and recover");
    Require(d.health(0).find("online=0")!=std::string::npos,"interleaved health text");
    const std::string diagnostic="# INIT name=rear_right status=255\n";
    for(int i=0;i<40;++i) (void)d.feed(std::span(reinterpret_cast<const uint8_t*>(diagnostic.data()),diagnostic.size()));
    Require(d.diagnostics().size()==32 && d.diagnostics().back().find("status=255")!=std::string::npos,"bounded startup diagnostics");
    auto header=Packet(2);header[5]=4;Decoder h;
    Require(h.feed(header).empty() && h.headerErrors()>0,"reject unknown sensor id");
    std::cout<<"protocol_and_projection: passed\n";
    if(argc==2) {
      std::ifstream file(argv[1],std::ios::binary);
      Require(bool(file),"recording open");std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file),{}};
      Decoder recorded;std::array<uint64_t,4> counts{};uint64_t total=0;Frame last;
      for(size_t at=0;at<bytes.size();at+=127) {
        for(const auto& f:recorded.feed(std::span(bytes.data()+at,std::min(size_t(127),bytes.size()-at)))) {
          ++counts[f.sensor];++total;last=f;
        }
      }
      std::cout<<"recording frames="<<total<<" crc_errors="<<recorded.crcErrors()<<" header_errors="<<recorded.headerErrors()<<" per_id=";
      for(auto n:counts) std::cout<<n<<",";
      std::cout<<" last_seq="<<last.sequence<<" last_read_us="<<last.readUs<<"\n";
    }
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
}
