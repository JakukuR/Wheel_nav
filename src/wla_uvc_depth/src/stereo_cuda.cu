#include "stereo_cuda.h"
#include <cmath>
namespace wla::uvc {
namespace {
// Input: USB right half is physical left; both eyes are mounted upside down.
__device__ int Pixel(const uint8_t* raw,int channels,int eye,int x,int y,int c){
  if(x<0||x>=1280||y<0||y>=720)return 0;
  const int u=(eye==0?2559:1279)-x,v=719-y;
  return raw[(v*2560+u)*channels+(channels==1?0:c)];
}
__device__ int Sample(const uint8_t* raw,int channels,int eye,float x,float y,int c){
  // Match OpenCV INTER_LINEAR's 1/32-pixel remap table and uint8 rounding.
  const int qx=__float2int_rn(x*32),qy=__float2int_rn(y*32);
  const int ix=qx>>5,iy=qy>>5,fx=qx&31,fy=qy&31;
  return (Pixel(raw,channels,eye,ix,iy,c)*(32-fx)*(32-fy)
    +Pixel(raw,channels,eye,ix+1,iy,c)*fx*(32-fy)
    +Pixel(raw,channels,eye,ix,iy+1,c)*(32-fx)*fy
    +Pixel(raw,channels,eye,ix+1,iy+1,c)*fx*fy+512)>>10;
}
__global__ void Prepare(const uint8_t* raw,int channels,const float* maps,float* l,float* r,uint8_t* preview,GpuLayout s){
  const int i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=s.width*s.networkHeight)return;
  const int x=i%s.width,y=i/s.width,ry=max(0,min(s.height-1,y-s.padTop)),m=ry*s.width+x;
  for(int eye=0;eye<2;++eye){
    float* out=eye?r:l;const float mx=maps[(eye*2)*s.width*s.height+m],my=maps[(eye*2+1)*s.width*s.height+m];
    for(int c=0;c<3;++c){const int v=Sample(raw,channels,eye,mx,my,c);out[(2-c)*s.width*s.networkHeight+i]=v;
      if(eye==0&&y>=s.padTop&&y<s.padTop+s.height)preview[m*3+c]=v;}
  }
}
__global__ void Reproject(float* disp,GpuGeometry g,float* points,uint16_t* depth,GpuLayout s){
  const int i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=s.width*s.height)return;
  const float x=i%s.width,y=i/s.width;
  float d=disp[i+s.padTop*s.width];
  if(!isfinite(d)||d<=0){d=0;disp[i+s.padTop*s.width]=0;}
  const float w=g.q[12]*x+g.q[13]*y+g.q[14]*d+g.q[15];
  const float a=(g.q[0]*x+g.q[1]*y+g.q[2]*d+g.q[3])/w;
  const float b=(g.q[4]*x+g.q[5]*y+g.q[6]*d+g.q[7])/w;
  const float z=(g.q[8]*x+g.q[9]*y+g.q[10]*d+g.q[11])/w;
  const float px=g.inverseR[0]*a+g.inverseR[1]*b+g.inverseR[2]*z;
  const float py=g.inverseR[3]*a+g.inverseR[4]*b+g.inverseR[5]*z;
  const float pz=g.inverseR[6]*a+g.inverseR[7]*b+g.inverseR[8]*z;
  const bool valid=isfinite(d)&&d>0&&isfinite(px)&&isfinite(py)&&isfinite(pz)&&isfinite(z)&&z>0;
  points[3*i]=valid?px:nanf("");points[3*i+1]=valid?py:0;points[3*i+2]=valid?pz:0;
  depth[i]=valid&&z>=.15f&&z<=5.f&&pz>=.15f&&pz<=5.f?__float2int_rn(z*1000.f):0;
}
}
void PrepareStereo(const uint8_t* raw,int channels,const float* maps,float* l,float* r,uint8_t* preview,cudaStream_t stream,GpuLayout s){Prepare<<<(s.width*s.networkHeight+255)/256,256,0,stream>>>(raw,channels,maps,l,r,preview,s);}
void ReprojectStereo(float* d,const GpuGeometry& g,float* p,uint16_t* z,cudaStream_t stream,GpuLayout s){Reproject<<<(s.width*s.height+255)/256,256,0,stream>>>(d,g,p,z,s);}
}
