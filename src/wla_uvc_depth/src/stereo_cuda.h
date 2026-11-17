#pragma once
#include <cuda_runtime_api.h>
#include <cstdint>
namespace wla::uvc {
struct GpuGeometry { float q[16]; float inverseR[9]; };
void PrepareStereo(const uint8_t* packed,int channels,const float* maps,
                   float* left,float* right,uint8_t* preview,cudaStream_t stream);
void ReprojectStereo(float* disparity,const GpuGeometry& geometry,
                     float* points,uint16_t* depth,cudaStream_t stream);
}
