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
    std::vector<float> input(640*384,10.f),xyz(640*360*3);
    std::vector<uint16_t> mm(640*360);
    input[12*640]=std::numeric_limits<float>::quiet_NaN();
    input[12*640+1]=std::numeric_limits<float>::infinity();
    input[12*640+2]=-1;input[12*640+3]=0;
    Check(cudaMalloc(reinterpret_cast<void**>(&disparity),input.size()*sizeof(float)));
    Check(cudaMalloc(reinterpret_cast<void**>(&points),xyz.size()*sizeof(float)));
    Check(cudaMalloc(reinterpret_cast<void**>(&depth),mm.size()*sizeof(uint16_t)));
    Check(cudaMemcpy(disparity,input.data(),input.size()*sizeof(float),cudaMemcpyHostToDevice));
    // Rectified pinhole Q: fx=100 pixels, baseline=.1m, identity rectification.
    wla::uvc::GpuGeometry geometry{{1,0,0,-320,0,1,0,-180,0,0,0,100,0,0,10,0},{1,0,0,0,1,0,0,0,1}};
    wla::uvc::ReprojectStereo(disparity,geometry,points,depth,nullptr);
    Check(cudaGetLastError());Check(cudaDeviceSynchronize());
    Check(cudaMemcpy(input.data(),disparity,input.size()*sizeof(float),cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(xyz.data(),points,xyz.size()*sizeof(float),cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(mm.data(),depth,mm.size()*sizeof(uint16_t),cudaMemcpyDeviceToHost));
    for(int i=0;i<4;++i)
      if(input[12*640+i]!=0||mm[i]!=0||std::isfinite(xyz[3*i]))
        throw std::runtime_error("invalid disparity must produce zero disparity/depth and invalid point");
    if(mm[4]!=1000||std::abs(xyz[4*3+2]-1.f)>1e-6)
      throw std::runtime_error("valid disparity must preserve metric depth");
    cudaFree(disparity);cudaFree(points);cudaFree(depth);
    std::cout<<"GPU invalid disparity and metric depth contract passed\n";
    return 0;
  } catch(const std::exception& error) {
    cudaFree(disparity);cudaFree(points);cudaFree(depth);
    std::cerr<<error.what()<<'\n';return 1;
  }
}
