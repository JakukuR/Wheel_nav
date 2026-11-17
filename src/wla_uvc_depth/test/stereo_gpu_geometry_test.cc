#include "../src/stereo_cuda.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

void Check(cudaError_t result) {
  if(result!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(result));
}
int main() {
  float *disparity=nullptr,*points=nullptr;uint16_t* depth=nullptr;
  try {
    for(const auto layout:{wla::uvc::GpuLayout{},wla::uvc::GpuLayout{384,216,288,36}}){
    const int plane=layout.width*layout.height,offset=layout.padTop*layout.width;
    const float scale=layout.width/640.f;
    std::vector<float> input(layout.width*layout.networkHeight,10.f*scale),xyz(plane*3);
    std::vector<uint16_t> mm(plane);
    input[offset]=std::numeric_limits<float>::quiet_NaN();
    input[offset+1]=std::numeric_limits<float>::infinity();
    input[offset+2]=-1;input[offset+3]=0;
    Check(cudaMalloc(reinterpret_cast<void**>(&disparity),input.size()*sizeof(float)));
    Check(cudaMalloc(reinterpret_cast<void**>(&points),xyz.size()*sizeof(float)));
    Check(cudaMalloc(reinterpret_cast<void**>(&depth),mm.size()*sizeof(uint16_t)));
    Check(cudaMemcpy(disparity,input.data(),input.size()*sizeof(float),cudaMemcpyHostToDevice));
    // Rectified pinhole Q: fx=100 pixels, baseline=.1m, identity rectification.
    wla::uvc::GpuGeometry geometry{{1,0,0,-320,0,1,0,-180,0,0,0,100,0,0,10,0},{1,0,0,0,1,0,0,0,1}};
    geometry.q[3]=-(320.f+.5f)*scale+.5f;geometry.q[7]=-(180.f+.5f)*scale+.5f;geometry.q[11]=100.f*scale;
    wla::uvc::ReprojectStereo(disparity,geometry,points,depth,nullptr,layout);
    Check(cudaGetLastError());Check(cudaDeviceSynchronize());
    Check(cudaMemcpy(input.data(),disparity,input.size()*sizeof(float),cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(xyz.data(),points,xyz.size()*sizeof(float),cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(mm.data(),depth,mm.size()*sizeof(uint16_t),cudaMemcpyDeviceToHost));
    for(int i=0;i<4;++i)
      if(input[offset+i]!=0||mm[i]!=0||std::isfinite(xyz[3*i]))
        throw std::runtime_error("invalid disparity must produce zero disparity/depth and invalid point");
    if(mm[4]!=1000||std::abs(xyz[4*3+2]-1.f)>1e-6)
      throw std::runtime_error("valid disparity must preserve metric depth");
    cudaFree(disparity);cudaFree(points);cudaFree(depth);disparity=nullptr;points=nullptr;depth=nullptr;
    }
    std::cout<<"GPU invalid disparity and metric depth contract passed\n";
    return 0;
  } catch(const std::exception& error) {
    cudaFree(disparity);cudaFree(points);cudaFree(depth);
    std::cerr<<error.what()<<'\n';return 1;
  }
}
