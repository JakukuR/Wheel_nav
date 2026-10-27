#include "wla_follow_transport/policy.hpp"
#include "wla_follow_transport/channel.hpp"
#include <iostream>
#include <stdexcept>
using namespace wla_follow;
void check(bool condition,const char *message) { if(!condition) throw std::runtime_error(message); }
OutputGate gate() {
  OutputGate g;g.enabled=g.active=g.authority=g.observations=true;
  g.request="task";g.detector_session="detector";g.detector_epoch=2;
  g.observation_stamp=980000000;g.lease_until=g.authority_until=11.;g.wheel_received=10.;
  g.delivered(990000000);return g;
}
Json result() {
  return {{"sequence",1},{"generation",0},{"motion_generation",0},{"request_id","task"},
    {"detector_session","detector"},{"detector_epoch",2},{"generated_at",10.},
    {"valid_until",10.15},{"frame_ns",990000000},{"observation_ns",980000000},
    {"linear",.2},{"angular",.1},{"permitted",true}};
}
int main() {
  try {
    auto g=gate();auto r=result();check(g.accept(r,10.01,1000000000),"fresh result rejected");
    check(g.blocker(10.02,1010000000,true).empty(),"fresh output blocked");
    check(!g.accept(r,10.03,1020000000),"replay accepted");
    check(g.blocker(10.151,1141000000,true)=="transport_result_expired","worker hang did not expire output");
    check(g.blocker(10.02,1010000000,false)=="wheel_odom_stale","invalid wheel accepted");
    g.estop=true;check(g.blocker(10.02,1010000000,true)=="local_estop","estop did not block");
    {auto test=gate();const auto cancel=test.cancel_generation;test.invalidate();
     check(test.cancel_generation==cancel && test.active,"reference change cancelled the task");
     test.revoke();check(test.cancel_generation==cancel+1,"revocation did not advance cancel generation");}
    g.estop=false;g.revoke();check(!g.active && !g.accept(r,10.03,1020000000),"revocation resurrected");
    for(const char *field:{"request_id","detector_session"}) {
      auto test=gate();auto bad=result();bad[field]="old";check(!test.accept(bad,10.01,1000000000),"identity mismatch accepted");
    }
    for(const char *field:{"generation","motion_generation","detector_epoch"}) {
      auto test=gate();auto bad=result();bad[field]=99;check(!test.accept(bad,10.01,1000000000),"generation mismatch accepted");
    }
    for(auto value:{10.0,10.4,std::numeric_limits<double>::infinity()}) {
      auto test=gate();auto bad=result();bad["valid_until"]=value;check(!test.accept(bad,10.01,1000000000),"invalid deadline accepted");
    }
    for(auto value:{std::numeric_limits<double>::quiet_NaN(),2.0}) {
      auto test=gate();auto bad=result();bad["linear"]=value;check(!test.accept(bad,10.01,1000000000),"invalid command accepted");
    }
    {auto test=gate();auto bad=result();bad["frame_ns"]=989999999;check(!test.accept(bad,10.01,1000000000),"undelivered frame accepted");}
    {auto test=gate();auto bad=result();bad["observation_ns"]=990000000;check(!test.accept(bad,10.01,1000000000),"invented observation accepted");}
    {auto test=gate();auto bad=result();bad["permitted"]=false;check(!test.accept(bad,10.01,1000000000),"worker veto ignored");}
    {auto test=gate();test.enabled=false;check(test.blocker(10.,1000000000,true)=="motion_disabled","preview enabled motion");}
    {auto test=gate();test.authority_until=10.;check(test.blocker(10.,1000000000,true)=="motion_permission_unavailable","authority timeout ignored");}
    {auto test=gate();test.lease_until=10.;check(test.blocker(10.,1000000000,true)=="task_inactive_or_expired","lease timeout ignored");}
    {auto test=gate();test.observations=false;check(test.blocker(10.,1000000000,true)=="target_observations_stale","disabled detection ignored");}
    Motion m;check(m.update(1.,{1.,0.,0.},1.),"initial motion rejected");
    check(m.update(1.1,{1.,0.,1.},1.1),"motion rejected");Vec3 p;
    check(m.pose_at(1.1,p) && std::abs(p[0]-.1*std::cos(.025))<1e-12,"trapezoidal integration changed");
    check(!m.pose_at(1.16,p),"extrapolation limit relaxed");
    check(!m.update(1.2,{2.,0.,0.},1.2) && m.generation==1 && m.samples.empty(),"invalid velocity did not reset");
    Packet packet;packet.metadata={{"test",true}};uint8_t data[]={0,1,2,255};packet.add("data",data,4);
    std::vector<uint8_t> slot(SLOT_BYTES);auto length=packet.write(slot.data());auto decoded=Packet::read(slot.data(),length);
    check(decoded.blob("data")==std::vector<uint8_t>({0,1,2,255}),"shared packet roundtrip failed");
    decoded.metadata["blobs"]["data"]["size"]=SLOT_BYTES;
    bool rejected=false;try {decoded.blob("data");}catch(const std::exception&){rejected=true;}
    check(rejected,"out-of-bounds blob accepted");
    std::cout<<"transport policy, motion and packet boundary checks passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
  return 0;
}
