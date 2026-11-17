#pragma once
#include <cuda_runtime_api.h>
#include <cstdint>
namespace wla::uvc {
struct GpuLayout { int width=640,height=360,networkHeight=384,padTop=12; };
struct GpuGeometry { float q[16]; float inverseR[9]; };
void PrepareStereo(const uint8_t* packed,int channels,const float* maps,
                   float* left,float* right,uint8_t* preview,cudaStream_t stream,GpuLayout layout={});
void ReprojectStereo(float* disparity,const GpuGeometry& geometry,
                     float* points,uint16_t* depth,cudaStream_t stream,GpuLayout layout={});
}
