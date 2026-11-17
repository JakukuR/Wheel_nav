#include "wla_uvc_depth/snapshot_field.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace wla::uvc {
namespace { constexpr float INF=1e12F; }
SnapshotField::SnapshotField(){
  const size_t n=grid_.nx*grid_.ny*grid_.nz;
  grid_.state.resize(n);grid_.distance.resize(n);occupied_.resize(n);free_.resize(n);
  const int m=std::max({grid_.nx,grid_.ny,grid_.nz});line_.resize(m);output_.resize(m);sites_.resize(m);boundary_.resize(m+1);
}
// Exact separable squared Euclidean distance transform of voxel-centre seeds.
// All-INF lines are handled explicitly, avoiding INF-INF cancellation.
void SnapshotField::transform(std::vector<float>& a){
  const auto line=[&](int n){
    int k=-1;
    for(int q=0;q<n;++q){
      if(line_[q]>=INF)continue;
      float s=-INF;
      while(k>=0){const int p=sites_[k];s=((line_[q]+q*q)-(line_[p]+p*p))/(2.F*(q-p));if(s>boundary_[k])break;--k;}
      ++k;sites_[k]=q;boundary_[k]=k==0?-INF:s;boundary_[k+1]=INF;
    }
    if(k<0){std::fill(output_.begin(),output_.begin()+n,INF);return;}
    int j=0;for(int q=0;q<n;++q){while(j<k&&boundary_[j+1]<q)++j;const int d=q-sites_[j];output_[q]=d*d+line_[sites_[j]];}
  };
  auto& g=grid_;
  for(int z=0;z<g.nz;++z)for(int y=0;y<g.ny;++y){for(int x=0;x<g.nx;++x)line_[x]=a[g.index(x,y,z)];line(g.nx);for(int x=0;x<g.nx;++x)a[g.index(x,y,z)]=output_[x];}
  for(int z=0;z<g.nz;++z)for(int x=0;x<g.nx;++x){for(int y=0;y<g.ny;++y)line_[y]=a[g.index(x,y,z)];line(g.ny);for(int y=0;y<g.ny;++y)a[g.index(x,y,z)]=output_[y];}
  for(int y=0;y<g.ny;++y)for(int x=0;x<g.nx;++x){for(int z=0;z<g.nz;++z)line_[z]=a[g.index(x,y,z)];line(g.nz);for(int z=0;z<g.nz;++z)a[g.index(x,y,z)]=output_[z];}
}
const FieldGrid& SnapshotField::update(const cv::Mat& depth,const cv::Matx33d& k){
  if(depth.empty()||depth.type()!=CV_16UC1||k(0,0)<=1||k(1,1)<=1||k(0,0)>10000||k(1,1)>10000)throw std::invalid_argument("invalid_depth_or_intrinsics");
  for(double v:k.val)if(!std::isfinite(v))throw std::invalid_argument("nonfinite_intrinsics");
  auto& g=grid_;std::fill(g.state.begin(),g.state.end(),0);
  // Mark ALL measured endpoints first, so a free-space sample cannot erase a hit.
  for(int v=0;v<depth.rows;++v)for(int u=0;u<depth.cols;++u){
    const float z=depth.at<uint16_t>(v,u)*.001F;if(z<.15F||z>5.F)continue;
    const float px=(u-k(0,2))*z/k(0,0),py=(v-k(1,2))*z/k(1,1);
    const int x=int(std::floor((px-g.x0)/g.resolution)),y=int(std::floor((py-g.y0)/g.resolution)),zi=int(std::floor((z-g.z0)/g.resolution));
    if(x>=0&&x<g.nx&&y>=0&&y<g.ny&&zi>=0&&zi<g.nz)g.state[g.index(x,y,zi)]=2;
  }
  // Projective visibility, no clearing behind a surface or from missing depth.
  const float margin=std::sqrt(3.F)*g.resolution*.5F;
  for(int z=0;z<g.nz;++z){const float pz=g.z0+(z+.5F)*g.resolution;if(pz<.15F)continue;
    for(int y=0;y<g.ny;++y)for(int x=0;x<g.nx;++x){const size_t i=g.index(x,y,z);if(g.state[i]==2)continue;
      const float px=g.x0+(x+.5F)*g.resolution,py=g.y0+(y+.5F)*g.resolution;
      const int u=int(std::floor(k(0,0)*px/pz+k(0,2)+.5)),v=int(std::floor(k(1,1)*py/pz+k(1,2)+.5));
      if(u<0||v<0||u>=depth.cols||v>=depth.rows)continue;
      const float measured=depth.at<uint16_t>(v,u)*.001F;
      if(measured>=.15F&&measured<=5.F&&pz+margin<measured)g.state[i]=1;
    }
  }
  for(size_t i=0;i<g.state.size();++i){occupied_[i]=g.state[i]==2?0:INF;free_[i]=g.state[i]==1?0:INF;}
  transform(occupied_);transform(free_);
  for(size_t i=0;i<g.state.size();++i){
    if(!g.state[i])g.distance[i]=std::numeric_limits<float>::quiet_NaN();
    else {const float squared=g.state[i]==2?free_[i]:occupied_[i];const float d=std::min(g.maxDistance,std::sqrt(squared)*g.resolution);g.distance[i]=g.state[i]==2?-d:d;}
  }
  return g;
}
}
