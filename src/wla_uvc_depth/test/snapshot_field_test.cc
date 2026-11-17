#include "wla_uvc_depth/snapshot_field.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace wla::uvc;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
int main(){try{
  SnapshotField field;cv::Mat depth(216,384,CV_16U,cv::Scalar(1000));
  const cv::Matx33d k(280,0,191.5,0,280,107.5,0,0,1);
  const auto& g=field.update(depth,k);
  require(g.state[g.index(30,20,10)]==1,"visible region in front of plane is free");
  require(g.state[g.index(30,20,20)]==2,"1m surface is occupied");
  require(g.state[g.index(30,20,30)]==0&&std::isnan(g.distance[g.index(30,20,30)]),"behind plane remains unknown");
  require(g.distance[g.index(30,20,20)]<0,"inside sign");
  // Verify the three-pass EDT against an independent brute-force nearest seed.
  for(const auto& xyz:std::vector<cv::Vec3i>{{30,20,10},{29,19,16},{25,18,17},{30,20,20}}){
    const auto index=g.index(xyz[0],xyz[1],xyz[2]);const auto state=g.state[index];require(state>0,"sample observed");
    float best=std::numeric_limits<float>::infinity();
    for(int z=0;z<g.nz;++z)for(int y=0;y<g.ny;++y)for(int x=0;x<g.nx;++x)
      if(g.state[g.index(x,y,z)]==(state==2?1:2))best=std::min(best,float((x-xyz[0])*(x-xyz[0])+(y-xyz[1])*(y-xyz[1])+(z-xyz[2])*(z-xyz[2])));
    const float expected=std::min(g.maxDistance,std::sqrt(best)*g.resolution)*(state==2?-1:1);
    require(std::abs(g.distance[index]-expected)<1e-6,"EDT agrees with brute force");
  }
  depth.setTo(2000);const auto& moved=field.update(depth,k);
  require(moved.state[moved.index(30,20,20)]==1,"previous surface removed on next frame");
  depth(cv::Rect(150,70,84,76)).setTo(0);const auto& hole=field.update(depth,k);
  require(hole.state[hole.index(30,20,10)]==0,"invalid depth does not clear free space");
  depth.setTo(5000);const auto& far=field.update(depth,k);
  require(far.state[far.index(30,20,30)]==1&&far.distance[far.index(30,20,30)]==1,"no surface in volume produces capped free distance");
  depth.setTo(0);const auto& empty=field.update(depth,k);
  for(size_t i=0;i<empty.state.size();++i)require(empty.state[i]==0&&std::isnan(empty.distance[i]),"missing frame cannot retain an old map");
  std::cout<<"single-frame visibility, unknown, signed EDT and no-history contracts passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
